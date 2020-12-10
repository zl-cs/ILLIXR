from typing import Optional
from dataclasses import dataclass, field
import pandas as pd

@dataclass(frozen=True)
class NoiseConfig:
    position_offset: float = field(default=0)
    position_stddev: float = field(default=0)
    orientation_offset: float = field(default=0)
    orientation_stddev: float = field(default=0)

@dataclass(frozen=True)
class ApproxConfig:
    # downsample_cameras: bool = field(default=False)
    # try_zupt: bool = field(default=False)
    num_pts: int = field(default=150)
    use_rk4_integration: bool = field(default=True)
    use_stereo: bool = field(default=True)
    use_klt: bool = field(default=True)
    use_tw: bool = field(default=True)

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
        if not use_tw:
            ret.append("!tw")
        if not ret:
            return "exact"
        else:
            return ",".join(ret)

@dataclass(frozen=True)
class Config:
    capture_frames: bool = field(default=False)
    capture_poses: bool = field(default=False)
    noise_slam_config: Optional[NoiseConfig] = field(default_factory=lambda: None)
    approx_slam_config: Optional[ApproxConfig] = field(default_factory=lambda: None)
    gt_slam: bool = field(default=False)
    start_time: float = 5e9


@dataclass
class Results:
    poses: pd.DataFrame # frame_no, time, orientation_err, position err
    frames: pd.DataFrame # frame_no, time, path, nearest_gt_time, gt_path, err
    config: Config

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Results):
            return self.config == other.config
        else:
            return False

    def __ne__(self, other: object) -> bool:
        return not self == other

    def __hash__(self) -> int:
        return hash(self.config)
