#!/usr/bin/env python
from __future__ import annotations

import os
import copy
from pathlib import Path
from typing import Optional, List, Any, NamedTuple, Union, Iterable, TypeVar, Type, Iterator, Final, Mapping, cast, Callable, Tuple
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
from skimage.metrics import structural_similarity
from skimage.io import imread, imsave

from load_gt import get_gt_pose_fns
from coordinate_transformations import ov_transform, gt_transform
from visualize_pose import visualize_2d, visualize_ts
from util import Subcommand

print("initialized")

cache_path = Path() / ".cache" / "pathsv2"
cache_path.mkdir(parents=True, exist_ok=True)

git = Subcommand("git")
make = Subcommand("make")
cmake = Subcommand("cmake")
num_cpu = max(multiprocessing.cpu_count() - 1, 1)

@dataclass(frozen=True)
class NoiseConfig:
    position_offset: float = field(default=0)
    position_stddev: float = field(default=0)
    orientation_offset: float = field(default=0)
    orientation_stddev: float = field(default=0)

@dataclass(frozen=True)
class ApproxConfig:
    # downsample_cameras: bool = field(default=False)
    num_pts: int = field(default=150)
    use_rk4_integration: bool = field(default=True)
    # try_zupt: bool = field(default=False)
    use_stereo: bool = field(default=True)
    use_klt: bool = field(default=True)

    def __str__(self) -> str:
        ret = []
        if self.num_pts != ApproxConfig().num_pts:
            ret.append(f"pts={self.num_pts}")
        if not self.use_rk4_integration:
            ret.append("!rk4")
        if not self.use_stereo:
            ret.append("!stereo")
        if not self.use_klt:
            ret.append("!klt")
        if not ret:
            return "exact"
        else:
            return ",".join(ret)

    @classmethod
    def sample(Class: Type[ApproxConfig]) -> Iterable[ApproxConfig]:
        # yield ApproxConfig(downsample_cameras=True)
        # yield ApproxConfig(try_zupt=False)
        yield ApproxConfig(num_pts=200)
        # yield ApproxConfig(num_pts=100)
        # yield ApproxConfig(num_pts=50)
        # yield ApproxConfig(use_rk4_integration=False)
        # yield ApproxConfig(use_stereo=False)
        # yield ApproxConfig(use_klt=False)
        # yield ApproxConfig(num_pts=200, use_klt=False)
        # yield ApproxConfig(use_stereo=False, use_klt=False)

@dataclass(frozen=True)
class Config:
    capture_frames: bool = field(default=False)
    capture_poses: bool = field(default=False)
    noise_slam_config: Optional[NoiseConfig] = field(default_factory=lambda: None)
    approx_slam_config: Optional[ApproxConfig] = field(default_factory=lambda: None)
    gt_slam: bool = field(default=False)

    @contextlib.contextmanager
    def write_yaml(self) -> Iterator[Path]:
        apitrace = get_apitrace()

        yaml_config = copy.deepcopy(rest_of_yaml_config)

        plugin_group = yaml_config["plugin_groups"][0]["plugin_group"]

        assert int(bool(self.gt_slam)) + int(bool(self.approx_slam_config)) + int(bool(self.noise_slam_config)) == 1
        if self.gt_slam:
            plugin_group.append(dict(path="pose_lookup"))
        elif self.approx_slam_config:
            plugin_group.append(dict(path="offline_imu_cam"))
            plugin_group.append(dict(path="open_vins"))
            plugin_group.append(dict(path="gtsam_integrator"))
            plugin_group.append(dict(path="pose_prediction"))
        elif self.noise_slam_config:
            plugin_group.append("pose_deviation")

        if self.capture_frames:
            plugin_group.append(dict(path="gldemo"))
            plugin_group.append(dict(path="timewarp_gl"))
            plugin_group.append(dict(path="frame_logger"))
            yaml_config["loader"]["command"] = f"{apitrace!s} trace --output=trace %a"

        if self.capture_poses:
            plugin_group.append(dict(path="slam_logger"))

        with tempfile.TemporaryDirectory() as _tmpdir:
            config_path = Path(_tmpdir) / "config.yaml"
            with config_path.open("w") as f:
                yaml.dump(yaml_config, f)
            yield config_path


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


@dataclass
class Results:
    poses: pd.DataFrame # frame_no, time, orientation_err, position err
    frames: pd.DataFrame # frame_no, time, path, nearest_gt_time, gt_path, err
    config: Config

@ch_cache.decor(ch_cache.FileStore.create(cache_path))
def get_apitrace() -> Path:
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
    print(config)
    env = copy.deepcopy(os.environ)
    if config.approx_slam_config:
        env.update(env_sanitize(asdict(config.approx_slam_config)))
    if config.noise_slam_config:
        env.update(env_sanitize(asdict(config.noise_slam_config)))

    with config.write_yaml() as config_path:
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

    return Results(
        poses=poses,
        frames=frames,
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


@ch_cache.decor(ch_cache.FileStore.create(cache_path))
def compute_ssim_error(frames: pd.DataFrame, gt_frames: pd.DataFrame) -> pd.DataFrame:
    (cache_path / "ssim").mkdir(exist_ok=True)

    frames = frames.copy()
    frames["ssim"] = np.zeros(len(frames))
    frames["ssim_path"] = [None for _ in range(len(frames))]
    frames["gt_frame_no"] = [0 for _ in range(len(frames))]

    start = bisect.bisect_left(frames.index, frames.index[0] + 2e9)
    real_frames = frames.iloc[start:]

    for time, path in tqdm(real_frames["path"].iteritems(), total=len(real_frames), unit="frames"):
        gt_frame_no = bisect.bisect_left(gt_frames.index, time)
        if gt_frames.index[gt_frame_no + 1] - time < time - gt_frames.index[gt_frame_no]:
            gt_frame_no += 1

        real_img = imread(path)
        gt_img = imread(gt_frames.iloc[gt_frame_no]["path"])

        frames.loc[time, "gt_frame_no"] = gt_frame_no
        frames.loc[time, "ssim_path"] = cache_path / "ssim" / f"{random.randint(0, 2**256):x}.png"

        save_img = True

        if save_img and random.randint(0, 50) == 0:
            frames.loc[time, "ssim"], ssim_img = structural_similarity(
                real_img,
                gt_img,
                multichannel=True,
                full=True,
            )
            ssim_img = ((ssim_img - ssim_img.min()) * 255 / ssim_img.max()).astype(np.uint8)
            imsave(frames.loc[time, "ssim_path"], ssim_img)
        else:
            frames.loc[time, "ssim"] = structural_similarity(
                real_img,
                gt_img,
                multichannel=True,
            )

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
            results.frames = compute_ssim_error(results.frames, gt_result.frames)
    return results

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
    for approx_config in ApproxConfig.sample()
), total=len(list(ApproxConfig.sample())), desc="trials"))

str_approx_results = {
    str(approx_config): result.poses
    for approx_config, result in approx_results.items()
}

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

def plot_bars() -> None:
    labels = [str(approx_config).replace(",", "\n") for approx_config in approx_results.keys()]
    width = 0.4
    xs = np.arange(len(labels))

    fig, ax = plt.subplots()
    bars: List[Tuple[str, Callable[[Results], pd.Series]]] = [
        ("pos_err", lambda result: result.poses["pos_err"]),
        ("ori_err", lambda result: result.poses["ori_err"]),
        ("ssim", lambda result: result.frames["ssim"]),
    ]
    for i, (name, series_fn) in enumerate(bars):
        
        height = [series_fn(result).mean() for result in approx_results.values()]
        yerr = [series_fn(result).std() for result in approx_results.values()]
        ax.bar(xs + i*width, height, width=width, yerr=yerr, label=name)
    ax.set_xticks(xs + len(bars)*width / 2)
    ax.set_xticklabels(labels)
    ax.set_xlabel("Approximation Configuration")
    ax.legend()
    fig.show()

plot_bars()
