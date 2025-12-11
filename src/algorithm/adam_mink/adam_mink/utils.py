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


def quat_mul_single(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Fast quaternion multiplication for single quaternions (w, x, y, z format).
        
    Args:
        q1: First quaternion (w, x, y, z)
        q2: Second quaternion (w, x, y, z)
            
    Returns:
        Result quaternion (w, x, y, z)
    """
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
        
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,  # w
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,  # x
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,  # y
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,  # z
    ], dtype=np.float64)
    
def quat_rotate_vector(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """Rotate a vector by a quaternion using pure numpy (w, x, y, z format).

    Args:
        quat: Rotation quaternion (w, x, y, z)
        vec: Vector to rotate (x, y, z)
            
    Returns:
        Rotated vector (x, y, z)
    """
    vec_quat = np.array([0.0, vec[0], vec[1], vec[2]], dtype=np.float64)    
    q_conj = np.array([quat[0], -quat[1], -quat[2], -quat[3]], dtype=np.float64)
    temp = quat_mul_single(quat, vec_quat)
    rotated_vec = quat_mul_single(temp, q_conj)
    return rotated_vec[1:4]


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
