import math

import numpy as np


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> list[list[float]]:
    """Convert a unit quaternion to a 3x3 rotation matrix."""
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def _matrix_to_quat(m: list[list[float]]) -> list[float]:
    """Convert a 3x3 rotation matrix to a unit quaternion."""
    m00, m01, m02 = m[0]
    m10, m11, m12 = m[1]
    m20, m21, m22 = m[2]

    trace = m00 + m11 + m22

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return [0.0, 0.0, 0.0, 1.0]

    inv_norm = 1.0 / norm
    return [x * inv_norm, y * inv_norm, z * inv_norm, w * inv_norm]


def _mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    """Multiply two 3x3 matrices."""
    return [
        [a[0][0] * b[0][j] + a[0][1] * b[1][j] + a[0][2] * b[2][j] for j in range(3)],
        [a[1][0] * b[0][j] + a[1][1] * b[1][j] + a[1][2] * b[2][j] for j in range(3)],
        [a[2][0] * b[0][j] + a[2][1] * b[1][j] + a[2][2] * b[2][j] for j in range(3)],
    ]


def _mat_vec_mul(a: list[list[float]], v: list[float]) -> list[float]:
    """Multiply a 3x3 matrix by a 3D vector."""
    return [
        a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
        a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
        a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
    ]


def normalize(v):
    return v / np.linalg.norm(v)


def pose_to_matrix(pos, quat):

    x, y, z, w = quat

    rot_mat = np.array(
        [
            [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
        ]
    )

    transform = np.eye(4)
    transform[:3, :3] = rot_mat
    transform[:3, 3] = pos

    return transform


def matrix_to_pose(transform_matrix):

    pos = transform_matrix[:3, 3]
    rot_mat = transform_matrix[:3, :3]
    qx, qy, qz, qw = _matrix_to_quat(rot_mat.tolist())

    return pos, [qx, qy, qz, qw]


def make_arm_frame(arm_dir):

    z = np.array([0, 0, 1])

    x = normalize(arm_dir)

    y = normalize(np.cross(z, x))

    z = np.cross(x, y)

    rot_mat = np.stack([x, y, z], axis=1)

    return rot_mat


def invert(transform_matrix):

    rot_mat = transform_matrix[:3, :3]
    t = transform_matrix[:3, 3]

    transform_inv = np.eye(4)

    transform_inv[:3, :3] = rot_mat.T
    transform_inv[:3, 3] = -rot_mat.T @ t

    return transform_inv
