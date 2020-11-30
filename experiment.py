#!/usr/bin/env python
from __future__ import annotations

import os
import copy
from pathlib import Path
from typing import Optional, List, Any, NamedTuple, Union, Iterable, TypeVar, Type, Iterator, Final, Mapping
import contextlib
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

cache_path = Path() / ".cache" / "pathsv2"
cache_path.mkdir(parents=True, exist_ok=True)

T = TypeVar("T")
def flatten(it: Iterable[Iterable[T]]) -> Iterable[T]:
    return itertools.chain.from_iterable(it)

class Subcommand:
    def __init__(
            self,
            command: Union[List[str], str, Path],
            short_flag: str = "-",
            long_flag: str = "-",
            kwarg_flag_sep: Optional[str] = None,
            parent: Optional[Subcommand] = None,
    ) -> None:
        if isinstance(command, Path):
            self.commands = [str(command)]
        elif isinstance(command, str):
            self.commands = [command]
        elif isinstance(command, list):
            self.commands = command
        else:
            raise TypeError(f"Type of {command} ({type(command)}) is bad")
        self.short_flag = short_flag
        self.long_flag = long_flag
        self.kwarg_flag_sep = kwarg_flag_sep
        self.parent = parent

    def to_flag(self, flag: str) -> str:
        if len(flag) == 1:
            return f"{self.short_flag}{flag}"
        else:
            return f"{self.long_flag}{flag}"

    def to_kwarg(self, var: str, obj: Any) -> List[str]:
        if isinstance(obj, bool):
            if obj:
                return [self.to_flag(var)]
            else:
                return []
        else:
            if self.kwarg_flag_sep:
                return [f"{self.to_flag(var)}{self.kwarg_flag_sep}{self.to_arg(obj)}"]
            else:
                return [self.to_flag(var), self.to_arg(obj)]

    def to_arg(self, obj: Any) -> str:
        return str(obj)

    def get_command_line(self) -> List[str]:
        parent_command = self.parent.get_command_line() if self.parent else []
        return parent_command + self.commands

    def __call__(self, *args: Any, _capture_output: bool = False, _text: bool = False, **kwargs: Any) -> str:
        kwargs_flags = list(flatten(self.to_kwarg(var, val) for var, val in kwargs.items()))
        args_flags = list(map(self.to_arg, args))
        command = self.get_command_line() + args_flags + kwargs_flags
        proc = subprocess.run(command, check=True, capture_output=_capture_output, text=_text)
        return proc

    def __getattr__(self, name: str) -> Subcommand:
        return Subcommand(
            command=name,
            parent=self,
            short_flag=self.short_flag,
            long_flag=self.long_flag,
            kwarg_flag_sep=self.kwarg_flag_sep,
        )

# TODO: support git(C=blah).clone()

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

    @classmethod
    def sample(Class: Type[ApproxConfig]) -> Iterable[ApproxConfig]:
        # yield ApproxConfig(downsample_cameras=True)
        yield ApproxConfig(num_pts=100)
        yield ApproxConfig(num_pts=70)
        yield ApproxConfig(use_rk4_integration=False)
        # yield ApproxConfig(try_zupt=False)
        yield ApproxConfig(use_stereo=False)
        yield ApproxConfig(use_klt=False)

@dataclass(frozen=True)
class Config:
    capture_frames: bool = field(default=False)
    capture_poses: bool = field(default=False)
    noise_slam_config: Optional[NoiseConfig] = field(default_factory=lambda: None)
    approx_slam_config: Optional[ApproxConfig] = field(default_factory=lambda: None)
    gt_slam: bool = field(default=False)

@ch_cache.decor(ch_cache.FileStore.create(cache_path))
def get_apitrace(apitrace_dir: Path) -> Path:
    try:
        git.clone("https://github.com/apitrace/apitrace", apitrace_dir)
        build_dir = apitrace_dir / "build"
        cmake(S=apitrace_dir, B=build_dir, D="CMAKE_BUILD_TYPE=RelWithDebInfo")
        make(C=build_dir, j=num_cpu)
        return build_dir / "apitrace"
    except Exception as e:
        shutil.rmtree(apitrace_dir)
        raise e

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

@contextlib.contextmanager
def write_config(config: Config) -> Iterator[Path]:
    apitrace = get_apitrace(cache_path / "apitrace")

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

class Results:
    poses: pd.DataFrame # frame_no, time, orientation_err, position err
    frames: pd.DataFrame # frame_no, time, path, nearest_gt_time, gt_path, err

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
def run(config: Config) -> None:
    env = copy.deepcopy(os.environ)
    if config.approx_slam_config:
        env.update(env_sanitize(asdict(config.approx_slam_config)))
    if config.noise_slam_config:
        env.update(env_sanitize(asdict(config.noise_slam_config)))

    with write_config(config) as config_path:
        subprocess.run(
        ["./runner.sh", Path("configs") / config_path],
            check=True,
            env=env,
        )

    poses = pd.DataFrame([], columns=["frame_no", "time", "position_x", "position_y", "position_z", "orientation_w", "orientation_x", "orientation_y", "orientation_z"])
    if config.capture_poses:
        poses_file = "poses.csv"
        poses = pd.read_csv(poses_file)
        os.remove(poses_file)

    frames = pd.DataFrame([], columns=["frame_no", "time", "frame_path"])
    if config.capture_frames:
        trace_path = cache_path / f"trace-{random.randint(0, 2**256):x}"
        Path("trace").rename(trace_path)
        frames_file = "frames.csv"
        frames = pd.read_csv(frames_file)
        os.remove(frames_file)
        frames["frame_path"] = [
            Path(trace_path / f"{frame_no}.png")
            for frame_no in frames["frame_no"]
        ]

ground_truth_result = run(Config(capture_frames=True, capture_poses=True, gt_slam=True))

approx_results = tqdm(
    (
        run(Config(capture_poses=True, capture_frames=True, approx_slam_config=approx_config))
        for approx_config in ApproxConfig.sample()
    ),
    size=len(list(ApproxConfig.sample())),
    descr="Approx config"
)
