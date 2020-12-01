def angle_between(X: np.ndarray, Y: np.ndarray) -> float:
    return np.arccos(np.dot(X, Y) / (norm(X) * norm(Y)))

def quaternion_to(X: np.ndarray, Y: np.ndarray) -> np.quaternion:
    axis = np.cross(X, Y)
    return quaternion.from_rotation_vector(axis * angle_between(X, Y) / norm(axis))

def regularize(poses: pd.DataFrame) -> pd.DataFrame:
    t = poses.index.searchsorted(1403715524912143104 + 5e9)
    assert 0 < t < len(poses.index)

    poses[["pos_x", "pos_y", "pos_z"]] -= poses.iloc[t][["pos_x", "pos_y", "pos_z"]]
    assert np.allclose(poses.iloc[t][["pos_x", "pos_y", "pos_z"]], [0, 0, 0])

    t = poses.index.searchsorted(6e9)
    assert t < len(poses.index)

    ori_offset = quaternion_to(poses.iloc[t][["pos_x", "pos_y", "pos_z"]], [1, 0, 0])
    poses[["pos_x", "pos_y", "pos_z"]] = \
        poses[["pos_x", "pos_y", "pos_z"]].to_numpy() @ quaternion.as_rotation_matrix(ori_offset).T
    poses[["ori_w", "ori_x", "ori_y", "ori_z"]] = quaternion.as_float_array(
        ori_offset * quaternion.as_quat_array(poses[["ori_w", "ori_x", "ori_y", "ori_z"]].to_numpy())
    )
    assert np.allclose(angle_between(poses.iloc[t][["pos_x", "pos_y", "pos_z"]], [1, 0, 0]), 0)
    return poses
