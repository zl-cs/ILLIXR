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
from cpu_timer3_parser import parse_dir as cpu_timer3_parse_dir

print("initialized")

cache_path = Path() / ".cache" / "pathsv2"
cache_path.mkdir(parents=True, exist_ok=True)


rest_of_yaml_config: Final[Mapping[str, Any]] = dict(
    plugin_groups=[dict(
        plugin_group=[]
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
        ILLIXR_RUN_DURATION=15,
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



@contextlib.contextmanager
def write_yaml(config: Config) -> Iterator[Path]:
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
        yield config_path

approx_configs = [
    # ApproxConfig(downsample_cameras=True),
    # ApproxConfig(try_zupt=False),
    ApproxConfig(num_pts=200),
    ApproxConfig(num_pts=50),
    ApproxConfig(use_rk4_integration=False),
    ApproxConfig(use_stereo=False),
    ApproxConfig(use_klt=False),

    ApproxConfig(num_pts=200, disable_tw=True),
    ApproxConfig(num_pts=50),
    ApproxConfig(use_rk4_integration=False),
    ApproxConfig(use_stereo=False),
    ApproxConfig(use_klt=False),
]


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
    env = copy.deepcopy(os.environ)
    if config.approx_slam_config:
        env.update(env_sanitize(asdict(config.approx_slam_config)))
    if config.noise_slam_config:
        env.update(env_sanitize(asdict(config.noise_slam_config)))
    env["ILLIXR_TIMEWARP_DISABLE"] = str(int(not config.use_timewarp))
    env["CPU_TIMER3_ENABLE"] = str(int(config.cpu_timer3_enable))

    with write_yaml(config) as config_path:
        with ch_time_block.ctx("runner.sh"):
            subprocess.run(
                ["./runner.sh", Path("configs") / config_path],
                check=True,
                env=env,
            )

    pose_columns = ["position_x", "position_y", "position_z", "orientation_w", "orientation_x", "orientation_y", "orientation_z"]
    poses = pd.DataFrame([], columns=pose_columns)
    if config.capture_poses:
        poses_file = "data/poses.csv"
        poses = pd.read_csv(poses_file, index_col=0)
    assert list(poses.columns) == pose_columns

    with ch_time_block.ctx("apitrace dump"):
        frame_columns = ["frame_path"]
        frames = pd.DataFrame([], columns=frame_columns)
        if config.capture_frames:
            frames_file = "data/frames.csv"
            frames = pd.read_csv(frames_file, index_col=0)
            trace_path = cache_path / f"trace-{random.randint(0, 2**256):x}"
            trace_path.mkdir()
            Path("trace").rename(trace_path / "trace")
            apitrace = get_apitrace()
            Subcommand(apitrace)("dump-images", (trace_path / "trace").resolve(), _subprocess_kwargs=dict(cwd=trace_path, capture_output=True))
            frame_paths = list(trace_path.glob("*.png"))
            frames["frame_path"] = sorted(frame_paths)[:len(frames)]
        assert list(frames.columns) == frame_columns

    cpu_timer3 = pd.DataFrame
    if common.cpu_timer3_enable:
        cpu_timer3 = cpu_timer3_parse_dir(Path(".cpu_timer3"))

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


gt_pos, gt_ori = get_gt_pose_fns()

gt_result = None


def run_illixr_with_post(config: Config, is_gt: bool) -> Results:
    results = cast(Results, run_illixr(config))
    if not results.poses.empty:
        results.poses = results.poses.rename(columns={
            "position_x": "pos_x",
            "position_y": "pos_y",
            "position_z": "pos_z",
            "orientation_w": "ori_w",
            "orientation_x": "ori_x",
            "orientation_y": "ori_y",
            "orientation_z": "ori_z",
        })
        results.poses = gt_transform(results.poses)
        if not is_gt:
            results.poses = compute_pose_error(results.poses, gt_pos, gt_ori)
    if not results.frames.empty:
        results.frames = results.frames.rename(columns={
            "frame_path": "path",
        })
        if not is_gt:
            assert gt_result is not None
            results.frames = compute_gt_frame_no(
                results.frames,
                gt_result.frames,
                results.config.start_time,
            )
            results = compute_video_dists(results)
    return results

print("gt_result")
gt_result = run_illixr_with_post(Config(capture_frames=True, gt_slam=True), is_gt=True)

approx_results = dict(tqdm((
    (
        approx_config,
        run_illixr_with_post(Config(
            capture_poses=True,
            capture_frames=True,
            approx_slam_config=approx_config,
        ), is_gt=False),
    )
    for approx_config in approx_configs
), total=len(approx_configs), desc="trials"))

approx_results_poses = pd.concat({
    approx_config: result.poses
    for approx_config, result in approx_results.items()
})

approx_results_frames = pd.concat({
    approx_config: result.frames
    for approx_config, result in approx_results.items()
})

approx_results_times = pd.concat({
    approx_config: result.cpu_timer3
    for approx_config, result in approx_results.items()
})


# fig, ax = visualize_2d(str_approx_results, gt_ori, "pos", ("x", "y"))
# fig.show()

# fig, ax = visualize_2d(str_approx_results, gt_ori, "ori", ("x", "y"))
# fig.show()

# fig, ax = visualize_ts(str_approx_results, "pos_err")
# fig.show()

# fig, ax = visualize_ts(str_approx_results, "ori_err")
# fig.show()

fig, ax = visualize_ts({
    str(approx_config): result.frames
    for approx_config, result in approx_results.items()
}, "ssim")
fig.show()

fig, ax = visualize_ts({
    str(approx_config): result.frames
    for approx_config, result in approx_results.items()
}, "ssim")
fig.show()

approx_df = pd.DataFrame.from_records(
    dict(
        approx_config=approx_config,
        pos_err_mean=result.pose["pos_err"].mean(),
        pos_err_std=result.poses["pos_err"].std(),
        ori_err_mean=result.poses["ori_err"].mean(),
        ori_err_std=result.poses["ori_err"].std(),
        **dict(iterable.chain.from_iterables([
            [
                (f"{video_dist}_mean", result.frames[video_dist].mean())
                (f"{video_dist}_std", result.frames[video_dist].std())
            ]
            for video_dist in video_dists
        ]))
    )
    for approx_config, result in approx_results.items()
).set_index(approx_config, verify_integrity=True)

columns = ["pos_err", "ori_err"] + video_dists

def plot_bars() -> None:
    fig, ax = plt.subplots()

    for i, approx_config in enumerate(approx_df.index):
        for j, column in enumerate(columns):
            pos = i * len(columns) + j
            height = approdx_df.loc[approx_config, f"{column}_mean"]
            yerr = approdx_df.loc[approx_config, f"{column}_std"]
            ax.bar(x, height, yerr=yerr, label=column)
    ax.set_xticks(xs + len(bars)*width / 2)
    ax.set_xticklabels([
        str(approx_config).replace(",", "\n")
        for approx_config in approx_df.index
    ])
    ax.set_xlabel("Approximation Configuration")
    ax.legend()
    # fig.show()
    plt.show()

plot_bars()
import IPython; IPython.embed()
