"""Utility functions for visualization in Mujoco viewer."""

from typing import Optional

import mujoco as mj
import mujoco.viewer as mjv
import numpy as np
from scipy.spatial.transform import Rotation as R

# Constants for visualization
DEFAULT_ARROW_SIZE = [0.01, 0.01, 0.01]
DEFAULT_ARROW_WIDTH = 0.005
RGBA_COLORS = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]  # Red, Green, Blue


def quat_mul(x, y):
    """
    Performs quaternion multiplication on arrays of quaternions

    :param x: tensor of quaternions of shape (..., Nb of joints, 4)
    :param y: tensor of quaternions of shape (..., Nb of joints, 4)
    :return: The resulting quaternions
    """
    x0, x1, x2, x3 = x[..., 0:1], x[..., 1:2], x[..., 2:3], x[..., 3:4]
    y0, y1, y2, y3 = y[..., 0:1], y[..., 1:2], y[..., 2:3], y[..., 3:4]

    res = np.concatenate([
        y0 * x0 - y1 * x1 - y2 * x2 - y3 * x3,
        y0 * x1 + y1 * x0 - y2 * x3 + y3 * x2,
        y0 * x2 + y1 * x3 + y2 * x0 - y3 * x1,
        y0 * x3 - y1 * x2 + y2 * x1 + y3 * x0], axis=-1)

    return res


def draw_frame(
    pos: np.ndarray,
    mat: np.ndarray,
    v,
    size: float,
    joint_name: Optional[str] = None,
    orientation_correction: R = R.from_euler("xyz", [0, 0, 0]),
    pos_offset: np.ndarray = np.array([0, 0, 0]),
) -> None:
    """Draw a coordinate frame in Mujoco viewer.

    Args:
        pos: Position of the frame origin (3D array).
        mat: Rotation matrix (3x3 array).
        v: Mujoco viewer instance.
        size: Size of the frame axes.
        joint_name: Optional name label for the frame.
        orientation_correction: Optional rotation correction to apply.
        pos_offset: Optional position offset to apply.
    """
    corrected_pos = pos + pos_offset
    fix_matrix = orientation_correction.as_matrix()

    for i in range(3):
        geom = v.user_scn.geoms[v.user_scn.ngeom]
        mj.mjv_initGeom(
            geom,
            type=mj.mjtGeom.mjGEOM_ARROW,
            size=DEFAULT_ARROW_SIZE,
            pos=corrected_pos,
            mat=mat.flatten(),
            rgba=RGBA_COLORS[i],
        )
        if joint_name is not None:
            geom.label = joint_name

        axis_direction = (mat @ fix_matrix)[:, i]
        mj.mjv_connector(
            v.user_scn.geoms[v.user_scn.ngeom],
            type=mj.mjtGeom.mjGEOM_ARROW,
            width=DEFAULT_ARROW_WIDTH,
            from_=corrected_pos,
            to=corrected_pos + size * axis_direction,
        )
        v.user_scn.ngeom += 1
