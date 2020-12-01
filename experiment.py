#!/usr/bin/env python
from __future__ import annotations

import os
import copy
from pathlib import Path
from typing import Optional, List, Any, NamedTuple, Union, Iterable, TypeVar, Type, Iterator, Final, Mapping, cast
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
        yield ApproxConfig(num_pts=200)
        yield ApproxConfig(num_pts=100)
        yield ApproxConfig(num_pts=50)
        yield ApproxConfig(use_rk4_integration=False)
        # yield ApproxConfig(try_zupt=False)
        yield ApproxConfig(use_stereo=False)
        yield ApproxConfig(use_klt=False)
        yield ApproxConfig(num_pts=200, use_klt=False)
        yield ApproxConfig(use_stereo=False, use_klt=False)

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
        poses_file = "poses.csv"
        poses = pd.read_csv(poses_file, index_col=0)
    assert list(poses.columns) == pose_columns

    with ch_time_block.ctx("apitrace dump"):
        frame_columns = ["frame_path"]
        frames = pd.DataFrame([], columns=frame_columns)
        if config.capture_frames:
            trace_path = cache_path / f"trace-{random.randint(0, 2**256):x}"
            trace_path.mkdir()
            Path("trace").rename(trace_path / "trace")
            apitrace = get_apitrace()
            Subcommand(apitrace)("dump-images", (trace_path / "trace").resolve(), _subprocess_kwargs=dict(cwd=trace_path, capture_output=True))
            frames_file = "frames.csv"
            frames2 = pd.read_csv(frames_file, index_col=0)
            os.remove(frames_file)
            frames["frame_path"] = sorted(trace_path.glob("*.png"))
        assert list(frames.columns) == frame_columns

    return Results(
        poses=poses,
        frames=frames,
    )

def run_illixr_with_post(config: Config) -> Results:
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
    return results

gt_pos, gt_ori = get_gt_pose_fns()

gt_result = run_illixr_with_post(Config(capture_frames=True, gt_slam=True))

approx_results = dict(
    (
        (
            approx_config,
            run_illixr_with_post(Config(
                capture_poses=True,
                capture_frames=True,
                approx_slam_config=approx_config,
            )),
        )
        for approx_config in ApproxConfig.sample()
    )
)

approx_poses = {
    str(approx_config): result.poses
    for approx_config, result in approx_results.items()
}

visualize_2d({"approx": approx_poses["pts=100"]}, gt_pos, "pos", ("x", "y"))
plt.show()

# visualize_2d(approx_poses, gt_ori, "ori", ("x", "y"))
# plt.show()

def vector_distance(X: np.ndarray[np.float64], Y: np.ndarray[np.float64]) -> np.ndarray[np.float64]:
    result = ((X - Y)**2).sum(axis=1)
    assert result.shape == (len(X),)
    return result

def quaternion_distance(X: np.ndarray[np.float64], Y: np.ndarray[np.float64]) -> np.ndarray[np.float64]:
    dot = np.sum(quaternion.as_float_array(X) * quaternion.as_float_array(Y), axis=1)
    assert dot.shape == (len(X),)
    return np.arccos(2*dot**2 - 1)


for config, result in approx_results.items():
    result.poses["pos_err"] = vector_distance(
        result.poses[["pos_x", "pos_y", "pos_z"]],
        gt_pos(result.poses.index) - gt_pos(result.poses.index[0]),
    )
    result.poses["ori_err"] = quaternion_distance(
        quaternion.as_quat_array(result.poses[["ori_w", "ori_x", "ori_y", "ori_z"]].to_numpy()),
        gt_ori(result.poses.index),
    )

# visualize_ts(approx_poses, "pos_err")
# plt.show()

# visualize_ts(approx_poses, "ori_err")
# plt.show()

labels = [str(approx_config).replace(",", "\n") for approx_config in approx_results.keys()]
width = 0.4
xs = np.arange(len(labels))

fig, ax = plt.subplots()
height = [approx_config.poses["pos_err"].mean() for approx_config in approx_results.values()]
yerr = [0.1*approx_config.poses["pos_err"].std() for approx_config in approx_results.values()]
ax.bar(xs + 0*width, height, width=width, yerr=yerr, label="Position Err")
height = [approx_config.poses["ori_err"].mean() for approx_config in approx_results.values()]
yerr = [approx_config.poses["ori_err"].std() for approx_config in approx_results.values()]
ax.bar(xs + 1*width, height, width=width, yerr=yerr, label="Orientation Err")
ax.set_xticks(xs + 0.5*width)
ax.set_xticklabels(labels)
ax.set_xlabel("Approximation Configuration")
ax.legend()
plt.show()
