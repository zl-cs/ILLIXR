from __future__ import annotations

from typing import Mapping, Callable, Optional, Tuple, Iterable
import itertools

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mayavi import mlab
import quaternion as quat

def visualize_2d(
        label_to_pose: Mapping[str, pd.DataFrame],
        gt_func: Optional[Callable[[float], np.ndarray[np.float32]]] = None,
        kind: str = "pos",
        plane: Tuple[str, str] = ("x", "y"),
) -> Tuple[plt.Figure, plt.Axes]:
    if not label_to_pose:
        initial_index = np.array([0.0])
    else:
        initial_index = list(label_to_pose.values())[0].index.to_numpy()

    fig = plt.figure()
    ax = fig.subplots(1, 1)
    axis0, axis1 = plane
    for label, poses in label_to_pose.items():
        ax.plot(
            poses[f"{kind}_{axis0}"],
            poses[f"{kind}_{axis1}"],
            label=label,
            alpha=0.3,
        )
    if gt_func:
        gt: np.ndarray[np.float32] = gt_func(initial_index)
        if kind == "ori":
            gt = quat.as_float_array(gt)
        axis_labels = {"pos": "xyz", "ori": "wxyz"}[kind]
        axis_num0 = axis_labels.find(axis0)
        axis_num1 = axis_labels.find(axis1)
        ax.plot(gt[:, axis_num0], gt[:, axis_num1], label="gt")
    ax.set_xlabel(f"{plane[0]} position (m)")
    ax.set_ylabel(f"{plane[1]} position (m)")
    ax.legend()
    return fig, ax

def visualize_ts(
        data: pd.Series,
        column: str,
) -> Tuple[plt.Figure, plt.Axes]:
    if not data:
        index = np.array([0.0])
    else:
        index = data.index.levels[1]

    fig = plt.figure()
    ax = fig.subplots(1, 1)
    for label in data.index.levels[0]:
        ax.plot(
            (data.index[label] - index[0]) / 1e9,
            data.loc[(label,)],
            label=label,
            alpha=0.3,
        )
    ax.set_title(data.name)
    ax.legend()
    return fig, ax

def visualize_1d_ts(
        label_to_pose: Mapping[str, pd.DataFrame],
        gt_func: Optional[Callable[[float], np.ndarray[np.float32]]] = None,
        kind: str = "pos",
        axis: str = "z",
) -> Tuple[plt.Figure, plt.Axes]:
    if not label_to_pose:
        initial_index = np.array([0.0])
    else:
        initial_index = list(label_to_pose.values())[0].index.to_numpy()
    fig, ax = visualize_ts(label_to_pose, f"{kind}_{axis}")
    if gt_func:
        gt = gt_func(initial_index)
        axis_labels = {"pos": "xyz", "ori": "wxyz"}[kind]
        axis_num = axis_labels.find(axis)
        ax.plot(
            (initial_index - initial_index[0]) / 1e9,
            gt[:, axis_num],
            label="gt",
        )
    return fig, ax

def visualize_3d(poses: pd.DataFrame) -> None:
    pos = poses[["pos_x", "pos_y", "pos_z"]].to_numpy()
    print(pos.min(axis=0), pos.max(axis=0))
    ts = poses.index.to_numpy()
    assert pos.shape == ts.shape + (3,)
    dt = np.diff(ts, axis=0).mean()
    vel = np.diff(pos, axis=0, prepend=np.array([pos[0]])) / (np.diff(ts, axis=0, prepend=np.array([ts[0] - dt]))[:, np.newaxis] / 1e9)
    assert vel.shape == pos.shape
    speed = np.sqrt(np.sum(vel**2, axis=1))
    c = len(pos) - 150
    mlab.plot3d(pos[:c, 0], pos[:c, 1], pos[:c, 2], speed[:c], colormap='hot', tube_radius=None)
    mlab.points3d(pos[0, 0], pos[0, 1], pos[0, 2], color=(0, 1, 0), scale_factor=0.1, opacity=0.5)
    mlab.points3d(pos[c, 0], pos[c, 1], pos[c, 2], color=(1, 0, 0), scale_factor=0.1, opacity=0.5)
    mlab.orientation_axes()
    mlab.colorbar()

    ball = mlab.points3d(pos[0, 0], pos[0, 1], pos[0, 2], color=(1, 1, 0), scale_factor=0.1, opacity=0.5)
    @mlab.animate(delay=20, ui=True) # type: ignore
    def animate() -> Iterable[None]:
        for i in itertools.cycle(range(0, c, 20)):
            ball.mlab_source.set(
                x = pos[i, 0],
                y = pos[i, 1],
                z = pos[i, 2],
            )
            yield

    animation = animate()
    mlab.show()

if __name__ == '__main__':
    from load_gt import get_gt_poses
    poses = get_gt_poses()
    visualize_3d(poses)
