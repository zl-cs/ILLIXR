#!/usr/bin/env python
from __future__ import annotations

import os
import copy
from pathlib import Path
from typing import Optional, List, Any, NamedTuple, Union, Iterable, TypeVar, Type, Iterator, Final, Mapping, cast, Callable, Tuple, Generic, Dict
import contextlib
import bisect
import multiprocessing
from dataclasses import dataclass, field, asdict
import subprocess
import operator
import random
import itertools
import tempfile
import shutil

import yaml
import pandas as pd
import charmonium.cache as ch_cache
import charmonium.time_block as ch_time_block
from tqdm import tqdm
import numpy as np
import quaternion
import matplotlib.pyplot as plt

from load_gt import get_gt_pose_fns
from coordinate_transformations import ov_transform, gt_transform
from visualize_pose import visualize_2d, visualize_ts
from util import Subcommand
from common import NoiseConfig, ApproxConfig, Config, Results
from video_dist import compute_video_dists, video_dists
import cpu_timer3_reader
import sqlite_record_reader

print("initialized")

cache_path = Path() / ".cache" / "pathsv2"
cache_path.mkdir(parents=True, exist_ok=True)


rest_of_yaml_config: Final[Mapping[str, Any]] = dict(
    plugin_groups=[dict(
        plugin_group=[],
    )],
    data=dict(
        subpath="mav0",
        relative_to=dict(
            archive_path=dict(
                download_url="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_02_medium/V1_02_medium.zip",
            ),
        ),
    ),
    demo_data="demo_data",
    loader=dict(
        name="native",
        ILLIXR_RUN_DURATION=30,
    ),
    profile="opt",
)


@ch_cache.decor(ch_cache.FileStore.create(cache_path))
def get_apitrace() -> Path:
    git = Subcommand("git")
    make = Subcommand("make")
    cmake = Subcommand("cmake")
    num_cpu = max(multiprocessing.cpu_count() - 1, 1)

    apitrace_dir = cache_path / "apitrace"
    if apitrace_dir.exists():
        shutil.rmtree(apitrace_dir)
    git.clone("https://github.com/apitrace/apitrace", apitrace_dir)
    build_dir = apitrace_dir / "build"
    cmake(S=apitrace_dir, B=build_dir, D="CMAKE_BUILD_TYPE=RelWithDebInfo")
    make(C=build_dir, j=num_cpu)
    return (build_dir / "apitrace").resolve()


def env_sanitize(env: Mapping[str, Any]) -> Mapping[str, str]:
    def env_sanitize_val(val: Any) -> str:
        if isinstance(val, bool):
            return "1" if val else "0"
        else:
            return str(val)
    return {
        key: env_sanitize_val(val)
        for key, val in env.items()
    }

@ch_cache.decor(ch_cache.FileStore.create(cache_path))
@ch_time_block.decor()
def run_illixr(config: Config) -> Results:
    apitrace = get_apitrace()

    yaml_config = copy.deepcopy(rest_of_yaml_config)

    plugin_group = yaml_config["plugin_groups"][0]["plugin_group"]

    assert int(bool(config.gt_slam)) + int(bool(config.approx_slam_config)) + int(bool(config.noise_slam_config)) == 1
    if config.gt_slam:
        plugin_group.append(dict(path="pose_lookup"))
    elif config.approx_slam_config:
        plugin_group.append(dict(path="offline_imu_cam"))
        plugin_group.append(dict(path="open_vins"))
        plugin_group.append(dict(path="gtsam_integrator"))
        plugin_group.append(dict(path="pose_prediction"))
    elif config.noise_slam_config:
        plugin_group.append("pose_deviation")

    if config.capture_frames:
        plugin_group.append(dict(path="gldemo"))
        plugin_group.append(dict(path="timewarp_gl"))
        plugin_group.append(dict(path="frame_logger"))
        yaml_config["loader"]["command"] = f"{apitrace!s} trace --output=trace %a"

    if config.capture_poses:
        plugin_group.append(dict(path="slam_logger"))

    with tempfile.TemporaryDirectory() as _tmpdir:
        config_path = Path(_tmpdir) / "config.yaml"
        with config_path.open("w") as f:
            yaml.dump(yaml_config, f)

        env = copy.deepcopy(os.environ)
        if config.approx_slam_config:
            env.update(env_sanitize(asdict(config.approx_slam_config)))
        if config.noise_slam_config:
            env.update(env_sanitize(asdict(config.noise_slam_config)))
        env["ILLIXR_TIMEWARP_DISABLE"] = str(int(
            not (config.approx_slam_config is not None and config.approx_slam_config.use_tw)
        ))
        env["CPU_TIMER3_ENABLE"] = str(int(config.cpu_timer3_enable))

        with ch_time_block.ctx("runner.sh"):
            subprocess.run(
                ["./runner.sh", Path("configs") / config_path],
                check=True,
                env=env,
            )

    if config.capture_poses:
        poses = (
            pd.read_csv("data/poses.csv", index_col=0)
            .rename(columns={
                "position_x": "pos_x",
                "position_y": "pos_y",
                "position_z": "pos_z",
                "orientation_w": "ori_w",
                "orientation_x": "ori_x",
                "orientation_y": "ori_y",
                "orientation_z": "ori_z",
            })
        )
    else:
        poses = pd.DataFrame([])

    if config.capture_frames:
        with ch_time_block.ctx("apitrace dump"):
            # Run apitrace
            trace_path = cache_path / f"trace-{random.randint(0, 2**256):x}"
            trace_path.mkdir()
            Subcommand(get_apitrace())(
                "dump-images",
                Path("trace").resolve(),
                _subprocess_kwargs=dict(cwd=trace_path, capture_output=True)
            )
            frames = pd.read_csv("data/frames.csv", index_col=0)
            frames["path"] = sorted(list(trace_path.glob("*.png")))[:len(frames)]
    else:
        frames = pd.DataFrame([])

    if config.cpu_timer3_enable:
        plugin_names = sqlite_record_reader.read_table(Path("./metrics/"), "plugin_name", ["plugin_id"])
        cpu_timer3 = cpu_timer3_reader.read_dir()
        # comment should be interpreted as a plugin_name where plugin_comment_mask
        plugin_comment_mask = (cpu_timer3["function_name"] == "_p_one_iteration") | (cpu_timer3["function_name"] == "invoke_callbacks")
        # merge cpu_timer3 with plugin_names where it makes sense
        # assign back into "comment"
        cpu_timer3.loc[plugin_comment_mask, "comment"] = (
            cpu_timer3
            .loc[plugin_comment_mask, ["comment"]]
            .assign(**{"plugin_id": lambda df: df["comment"].astype(int)})
            .merge(plugin_names, how="left", validate="m:1", left_on="plugin_id", right_index=True)
            ["plugin_name"]
        )
    else:
        cpu_timer3 = pd.DataFrame()

    if not poses.empty:
        assert "pos_x" in poses.columns
    return Results(
        config=config,
        poses=poses,
        frames=frames,
        cpu_timer3=cpu_timer3,
    )


def compute_pose_error(
        poses: pd.DataFrame,
        gt_pos: Callable[[np.ndarray[np.float32]], np.ndarray[np.float32]],
        gt_ori: Callable[[np.ndarray[np.float32]], np.ndarray[np.float32]],
) -> pd.DataFrame:


    def vector_distance(X: np.ndarray[np.float64], Y: np.ndarray[np.float64]) -> np.ndarray[np.float64]:
        result = ((X - Y)**2).sum(axis=1)
        assert result.shape == (len(X),)
        return result

    def quaternion_distance(X: np.ndarray[np.float64], Y: np.ndarray[np.float64]) -> np.ndarray[np.float64]:
        dot = np.sum(quaternion.as_float_array(X) * quaternion.as_float_array(Y), axis=1)
        assert dot.shape == (len(X),)
        return np.arccos(2*dot**2 - 1)

    poses = poses.copy()
    poses["pos_err"] = vector_distance(
        poses[["pos_x", "pos_y", "pos_z"]],
        gt_pos(poses.index),
    )
    poses["ori_err"] = quaternion_distance(
        quaternion.as_quat_array(poses[["ori_w", "ori_x", "ori_y", "ori_z"]].to_numpy()),
        gt_ori(poses.index),
    )
    return poses


def compute_gt_frame_no(frames: pd.DataFrame, gt_frames: pd.DataFrame, start_time: float) -> pd.DataFrame:
    frames = frames.copy()
    frames["gt_path"] = None
    frames["gt_frame_no"] = 0

    for time, path in frames["path"].iteritems():
        gt_frame_no = bisect.bisect_left(gt_frames.index, time)
        if gt_frame_no >= len(gt_frames):
            gt_frame_no = len(gt_frames) - 1
        if gt_frame_no + 1 < len(gt_frames) and gt_frames.index[gt_frame_no + 1] - time < time - gt_frames.index[gt_frame_no]:
            gt_frame_no += 1
        frames.loc[time, "gt_frame_no"] = gt_frame_no
        frames.loc[time, "gt_path"] = gt_frames.iloc[gt_frame_no]["path"]

    return frames


gt_results = None
gt_pos, gt_ori = None, None

def run_illixr_with_post(config: Config) -> Results:
    results = cast(Results, run_illixr(config))
    if not results.poses.empty:
        results.poses = gt_transform(results.poses)
        global gt_pos, gt_ori
        if gt_pos is None:
            gt_pos, gt_ori = get_gt_pose_fns()
        results.poses = compute_pose_error(results.poses, gt_pos, gt_ori)
    if not results.frames.empty:
        global gt_results
        if gt_results is None:
            gt_results = run_illixr(Config(capture_frames=True, gt_slam=True))
        results.frames = compute_gt_frame_no(
            results.frames,
            gt_results.frames,
            results.config.start_time,
        )
        results = compute_video_dists(results)
    return results

def pd_reduce(op: Union[str, Callable[[pd.Series, pd.Series], pd.Series]], masks: Iterable[pd.Series], dtype=None) -> pd.Series:
    op_names = {
        "or": operator.or_,
        "and": operator.and_,
    }
    if isinstance(op, str):
        try:
            op2 = op_names[op]
        except KeyError:
            raise KeyError(f"Unknown operator: {op}")
    else:
        op2 = op

    masks_it = iter(masks)
    try:
        mask = next(masks_it)
    except StopIteration:
        return pd.Series([], dtype=dtype)
    else:
        ret = mask
    for mask in masks_it:
        ret = op2(ret, mask)
    return ret

@ch_time_block.decor(print_start=False)
def compute_ov_time_mask(cpu_timer3: pd.DataFrame) -> pd.DataFrame:
    ov_function_names = {
        # ov_core/src/track/Grider_FAST.h
        "Grider_FAST::perform_griding_worker",

        # ov_core/src/track/TrackKLT.cpp
        "cv::equalizeHist", "cv::buildOpticalFlowPyramid", "perform_matching",

        # ov_core/src/track/TrackDescriptor.cpp
        "cv::ORB::compute", "robust_match", "Grider_FAST::perform_griding",
    }
    switchboard_function_names = {"_p_one_iteration", "invoke_callbacks"}
    ov_plugins = {"timewarp_gl", "slam2"}
    # Unfortunately, I don't know a pandas way of checking (df["function_name"] in set_of_things) as a bool mask
    ov_time_mask = cpu_timer3.apply(lambda row: (
        row["function_name"] in ov_function_names or (row["function_name"] in switchboard_function_names and row["comment"] in ov_plugins)
    ), axis=1)
    # ov_time_mask = (
    #     pd_reduce("or", [cpu_timer3["function_name"] == dict(cpu_timer3["function_name"].cat.codes).get(fn_name, 1000) for fn_name in ov_function_names])
    #     | (
    #         pd_reduce("or", [cpu_timer3["function_name"] == dict(cpu_timer3["function_name"].cat.codes).get(fn_name, 1000) for fn_name in switchboard_function_names])
            # & pd_reduce("or", [row["comment"] == plugin for plugin in ov_plugins])
    #     )
    # )
    return ov_time_mask


@ch_cache.decor(ch_cache.FileStore.create(cache_path))
def get_approx_results(approx_configs: List[ApproxConfig]) -> Tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    approx_results = dict(tqdm((
        (
            approx_config,
            run_illixr_with_post(Config(
                capture_poses=True,
                capture_frames=True,
                approx_slam_config=approx_config,
            )),
        )
        for approx_config in approx_configs
    ), total=len(approx_configs), desc="trials"))
    return approx_results

approx_results = get_approx_results([
    # ApproxConfig(downsample_cameras=True),
    # ApproxConfig(try_zupt=False),
    ApproxConfig(num_pts=200),
    ApproxConfig(num_pts=50),
    ApproxConfig(use_rk4_integration=False),
    ApproxConfig(use_stereo=False),
    ApproxConfig(use_klt=False),

    ApproxConfig(num_pts=200, use_tw=False),
    ApproxConfig(num_pts=50, use_tw=False),
    ApproxConfig(use_rk4_integration=False, use_tw=False),
    ApproxConfig(use_stereo=False, use_tw=False),
    ApproxConfig(use_klt=False, use_tw=False),
])

approx_results_poses = pd.concat({
    approx_config: results.poses
    for approx_config, results in approx_results.items()
}, names=["approx_config"])

approx_results_frames = pd.concat({
    approx_config: results.frames
    for approx_config, results in approx_results.items()
}, names=["approx_config"])

result_cpu_timer3s = cpu_timer3_reader.normalize_cats([results.cpu_timer3 for results in approx_results.values()])
approx_results_times = pd.concat({
    approx_config: cpu_timer3
    for approx_config, cpu_timer3 in zip(approx_results.keys(), result_cpu_timer3s)
}, names=["approx_config"])


# fig, ax = visualize_2d(str_approx_results, gt_ori, "pos", ("x", "y"))
# fig.show()

# fig, ax = visualize_2d(str_approx_results, gt_ori, "ori", ("x", "y"))
# fig.show()

# fig, ax = visualize_ts(str_approx_results, "pos_err")
# fig.show()

# fig, ax = visualize_ts(str_approx_results, "ori_err")
# fig.show()

def var_sum(df: pd.DataFrame, iid: List[str], columns: Optional[List[str]] = None) -> float:
    """Returns the variance of the distribution of the sum, assuming each tuple of values of columns of iid is an i.i.d. observation"""
    columns2 = columns if columns else [column for column in df.columns if column not in iid]
    return df.groupby(iid).agg({
        col_name: lambda col_data: col_data.var() * col_data.count()
        for col_name in columns2
    }).dropna().sum()

def std_sum(df: pd.DataFrame, iid: List[str], columns: Optional[List[str]] = None) -> float:
    """See var_sum"""
    return np.sqrt(var_sum(df, iid, columns))

approx_df = pd.DataFrame.from_records(
    dict(
        approx_config=approx_config,
        pos_err_mean=results.poses["pos_err"].mean(),
        pos_err_std =results.poses["pos_err"].std (),
        ori_err_mean=results.poses["ori_err"].mean(),
        ori_err_std =results.poses["ori_err"].std (),
        ov_time_mean=results.cpu_timer3[compute_ov_time_mask(results.cpu_timer3)]["cpu_time"].sum() / results.cpu_timer3["cpu_time"].sum(),
        ov_time_std =std_sum(results.cpu_timer3.reset_index(), ["data_file", "thread_id", "function_name", "comment"], ["cpu_time"])["cpu_time"] / results.cpu_timer3["cpu_time"].sum(),
        **dict(itertools.chain.from_iterable([
            [
                (f"{video_dist}_mean", results.frames[video_dist].mean()),
                (f"{video_dist}_std" , results.frames[video_dist].std ()),
            ]
            for video_dist in video_dists
        ]))
    )
    for approx_config, results in approx_results.items()
).set_index("approx_config", verify_integrity=True).rename(columns={
    "mean_feature_dist_mean": "homography_dist_mean",
    "mean_feature_dist_std": "homography_dist_std",
})

# fig, ax = visualize_ts(approx_results_frames.loc[:, "ssim"])
# fig.show()

# fig, ax = visualize_ts(approx_results_frames.loc[:, "mean_feature_dist"])
# fig.show()

# fig, ax = visualize_ts(approx_results_frames.loc[:, "max_feature_dist"])
# fig.show()

# fig, ax = visualize_ts(approx_results_poses.loc[:, "pos_err"])
# fig.show()

video_dists = ["ssim", "homography_dist"]
all_columns = ["pos_err", "ori_err", "ov_time"] + video_dists
import seaborn as sns
columns_to_colors = lambda columns: [
    sns.color_palette("hls", len(all_columns))[all_columns.index(column)]
    for column in columns
]

def plot_bars(approx_df, columns: List[str], colors: List[Any], normed: bool, std: bool =True) -> None:
    fig, ax = plt.subplots()

    # intergroup_gap = 0.1
    # intergroup_gap = 0.0
    # width = 1.0

    for i, approx_config in enumerate(approx_df.index):
        for j, (column, color) in enumerate(zip(columns, colors)):
            pos = i * len(columns) + j
            scale = approx_df.loc[(), f"{column}_mean"].max() if normed else 1
            height   = approx_df.loc[approx_config, f"{column}_mean"] / scale
            yerr = approx_df.loc[approx_config, f"{column}_std" ] / scale if std else None
            ax.bar(pos, height, yerr=yerr, color=color, label=(column if i == 0 else None))
    ax.set_xticks(np.arange(len(approx_df.index)) * len(columns))
    ax.set_xticklabels([
        str(approx_config).replace(",", "\n")
        for approx_config in approx_df.index
    ])
    ax.set_xlabel("Approximation Configuration")
    ax.legend()
    fig.subplots_adjust(bottom=0.2)
    width, height = fig.get_size_inches()
    fig.set_size_inches(width + 2, height)
    return fig, ax

gen_graphs = False
if gen_graphs:
    fig, ax = plot_bars(
        approx_df,
        ["ov_time"],
        columns_to_colors(["ov_time"]),
        normed=False
    )
    ax.set_ylabel("CPU time (rel. to total, lower is better)\nError-bars show one std. dev.")
    fig.savefig("cs527_report/times.png", dpi=300)

    fig, ax = plot_bars(
        approx_df,
        ["pos_err", "ori_err"],
        columns_to_colors(["pos_err", "ori_err"]),
        normed=False,
        std=False
    )
    ax.set_ylabel("Position and orientation error (cm and rad)")
    fig.savefig("cs527_report/slam_errors.png", dpi=300)

    fig, ax = plot_bars(
        approx_df,
        video_dists,
        columns_to_colors(video_dists),
        normed=True,
    )
    ax.set_ylabel("System-level errors (higher is better)\nError-bars show one std. dev.")
    ax.set_ylim(0, 1.2)
    fig.savefig("cs527_report/system_errors.png", dpi=300)

    # fig, ax = plot_bars(
    #     approx_df,
    #     ["pos_err", "ori_err", *video_dists],
    #     columns_to_colors(["pos_err", "ori_err", *video_dists]),
    #     normed=True,
    # )
    # ax.set_ylim(0, 1.2)
    # fig.savefig("cs527_report/system_errors.png", dpi=300)


