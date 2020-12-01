from __future__ import annotations

from typing import Tuple, Callable, List

from scipy.interpolate import interp1d
import pandas as pd
import numpy as np
import quaternion as quat

from coordinate_transformations import gt_transform

def get_gt_poses() -> pd.DataFrame:
    gt_poses_file = ".cache/paths/http%%c%%s%%srobotics.ethz.ch%%s~asl-datasets%%sijrr_euroc_mav_dataset%%svicon_room1%%sV1_02_medium%%sV1_02_medium.zip/mav0/state_groundtruth_estimate0/data.csv"
    gt_poses = pd.read_csv(gt_poses_file)
    real_columns = "time pos_x pos_y pos_z ori_w ori_x ori_y ori_z".split(" ")
    unused_columns: List[str] = gt_poses.columns[len(real_columns):]
    gt_poses = gt_poses.drop(unused_columns, axis=1)
    gt_poses.columns = pd.Index(real_columns)
    gt_poses = gt_poses.set_index("time", drop=True, verify_integrity=True)
    return gt_transform(gt_poses)

def get_gt_pose_fns() -> Tuple[Callable[[float], np.ndarray[np.float32]], Callable[[float], np.ndarray[np.float32]]]:
    gt_poses = get_gt_poses()
    gt_pos = interp1d(gt_poses.index, gt_poses[["pos_x", "pos_y", "pos_z"]], axis=0)
    gt_ori_ = interp1d(gt_poses.index, gt_poses[["ori_w", "ori_x", "ori_y", "ori_z"]], axis=0)
    gt_ori = lambda t: quat.as_quat_array(gt_ori_(t))
    return gt_pos, gt_ori
