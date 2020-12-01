import pandas as pd
import numpy as np
import quaternion as quat


# See pose_lookup/plugin.cpp
target_initial_ori = np.quaternion(1, 0, 0, 0)
actual_initial_ori = np.quaternion(0.161996,0.789985,-0.205376,0.554528)
ori_offset = target_initial_ori * actual_initial_ori.inverse()
ori_offset_mat = quat.as_rotation_matrix(ori_offset)
target_initial_pos = np.array([0.5, 0.5, 0.5])
actual_initial_pos = np.array([0.515356,1.996773,0.971104])
pos_offset = target_initial_pos - ori_offset_mat @ actual_initial_pos


assert np.allclose(np.norm(target_initial_ori), 1)
assert np.allclose(np.norm(actual_initial_ori), 1)
assert np.allclose(np.norm(ori_offset), 1)
assert np.allclose(pos_offset + ori_offset_mat @ actual_initial_pos, target_initial_pos)
assert np.allclose(ori_offset * actual_initial_ori, target_initial_ori)


# See align-trajectory/src/main.cpp
# These are dumped in alignMatrix.txt
# After main is run poses.csv, produced by non-approximate OpenVINS with slam_logger
def gt_transform(poses: pd.DataFrame) -> pd.DataFrame:
    poses = poses.copy()
    pos = poses[["pos_x", "pos_y", "pos_z"]].to_numpy().T
    ori = quat.from_float_array(poses[["ori_w", "ori_x", "ori_y", "ori_z"]].to_numpy())
    poses[["pos_x", "pos_y", "pos_z"]] = \
        pos_offset + (ori_offset_mat @ pos).T
    poses[["ori_w", "ori_x", "ori_y", "ori_z"]] = quat.as_float_array(
        ori_offset * ori
    )
    return poses


s_ESTtoGT = 1.0
R_ESTtoGT = np.array([[0.911125, 0.412131, 0.0], [-0.412131, 0.911125, 0.0], [0.0, 0.0, 1.0]])
# q_ESTtoGT = np.quaternion(0.977529, 0, 0, 0.210802)
q_ESTtoGT = np.quaternion(0.977528687278213, -0, -0, -0.210802432500513)
assert np.allclose(quat.as_rotation_matrix(q_ESTtoGT), R_ESTtoGT)
t_ESTinGT = np.array([0.58844, 2.02848, 0.962197])


def ov_transform(poses: pd.DataFrame) -> pd.DataFrame:
    poses = poses.copy()
    pos = poses[["pos_x", "pos_y", "pos_z"]].to_numpy().T
    ori = quat.from_float_array(poses[["ori_w", "ori_x", "ori_y", "ori_z"]].to_numpy())
    poses[["pos_x", "pos_y", "pos_z"]] = s_ESTtoGT*(R_ESTtoGT @ pos).T+t_ESTinGT
    poses[["ori_w", "ori_x", "ori_y", "ori_z"]] = quat.as_float_array(q_ESTtoGT * ori)
    return poses
