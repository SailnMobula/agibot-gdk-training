"""Poses as 4x4 matrices.

T_a_b maps a point given in frame b into frame a, so p_a = T_a_b @ p_b. GDK quaternions have
the order x, y, z, w. Positions are in metres.
"""

from __future__ import annotations

import math

import numpy as np

Matrix = np.ndarray


def matrix(position: tuple[float, float, float], quaternion: tuple[float, float, float, float]) -> Matrix:
    x, y, z, w = np.asarray(quaternion, dtype=float) / np.linalg.norm(quaternion)
    result = np.eye(4)
    result[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    result[:3, 3] = position
    return result


def from_pose(pose: object) -> Matrix:
    """Converts an agibot_gdk.Pose (position, orientation) to a matrix."""
    p, q = pose.position, pose.orientation
    return matrix((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def from_transform(transform: object) -> Matrix:
    """Converts an agibot_gdk.Transform (translation, rotation) to a matrix."""
    t, q = transform.translation, transform.rotation
    return matrix((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


def fill_pose(pose: object, transform: Matrix) -> None:
    """Writes a matrix into an agibot_gdk.Pose."""
    pose.position.x, pose.position.y, pose.position.z = position(transform)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion(transform)


def inverse(transform: Matrix) -> Matrix:
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -transform[:3, :3].T @ transform[:3, 3]
    return result


def position(transform: Matrix) -> tuple[float, float, float]:
    return tuple(float(v) for v in transform[:3, 3])


def quaternion(transform: Matrix) -> tuple[float, float, float, float]:
    """Returns the rotation as a quaternion x, y, z, w.

    The component with the largest magnitude is computed from the diagonal and the other three
    from it. This stays accurate for rotations near 180 degrees, where w is close to zero.
    """
    r = transform[:3, :3]
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0:
        w = math.sqrt(1 + trace) / 2
        x, y, z = (r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w)
    elif r[0, 0] >= r[1, 1] and r[0, 0] >= r[2, 2]:
        x = math.sqrt(1 + r[0, 0] - r[1, 1] - r[2, 2]) / 2
        y, z, w = (r[0, 1] + r[1, 0]) / (4 * x), (r[0, 2] + r[2, 0]) / (4 * x), (r[2, 1] - r[1, 2]) / (4 * x)
    elif r[1, 1] >= r[2, 2]:
        y = math.sqrt(1 - r[0, 0] + r[1, 1] - r[2, 2]) / 2
        x, z, w = (r[0, 1] + r[1, 0]) / (4 * y), (r[1, 2] + r[2, 1]) / (4 * y), (r[0, 2] - r[2, 0]) / (4 * y)
    else:
        z = math.sqrt(1 - r[0, 0] - r[1, 1] + r[2, 2]) / 2
        x, y, w = (r[0, 2] + r[2, 0]) / (4 * z), (r[1, 2] + r[2, 1]) / (4 * z), (r[1, 0] - r[0, 1]) / (4 * z)
    return float(x), float(y), float(z), float(w)


def rpy(transform: Matrix) -> tuple[float, float, float]:
    """Returns roll, pitch and yaw in rad. Use it to display an orientation."""
    r = transform[:3, :3]
    return (math.atan2(r[2, 1], r[2, 2]), math.asin(max(-1.0, min(1.0, -r[2, 0]))), math.atan2(r[1, 0], r[0, 0]))


def slerp(a: tuple[float, ...], b: tuple[float, ...], t: float) -> tuple[float, float, float, float]:
    qa, qb = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    dot = float(qa @ qb)
    if dot < 0:
        qb, dot = -qb, -dot
    if dot > 0.9995:
        q = qa + t * (qb - qa)
    else:
        angle = math.acos(dot)
        q = (math.sin((1 - t) * angle) * qa + math.sin(t * angle) * qb) / math.sin(angle)
    return tuple(float(v) for v in q / np.linalg.norm(q))


def line(start: Matrix, goal: Matrix, step: float) -> list[Matrix]:
    """Returns poses on a straight line from start to goal, at most step metres apart, ending at goal."""
    a, b = np.asarray(position(start)), np.asarray(position(goal))
    count = max(1, math.ceil(float(np.linalg.norm(b - a)) / step))
    qa, qb = quaternion(start), quaternion(goal)
    return [matrix(tuple(a + (b - a) * i / count), slerp(qa, qb, i / count)) for i in range(1, count + 1)]
