"""Checks the pose helpers, in particular quaternions near a rotation of 180 degrees."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gdk_training import geometry  # noqa: E402

HALF_TURNS = [(1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0.7071068, -0.7071068, 0, 0), (0.6, 0, -0.8, 0),
              (0.5858436, -0.02614889, 0.80973243, 0.02089957)]


@pytest.mark.parametrize("quaternion", HALF_TURNS)
def test_quaternion_round_trip_near_a_half_turn(quaternion: tuple[float, float, float, float]) -> None:
    pose = geometry.matrix((0.1, 0.2, 0.3), quaternion)
    assert np.allclose(geometry.matrix((0.1, 0.2, 0.3), geometry.quaternion(pose)), pose, atol=1e-6)


def test_quaternion_round_trip_for_random_rotations() -> None:
    for quaternion in np.random.default_rng(0).normal(size=(2000, 4)):
        pose = geometry.matrix((0, 0, 0), tuple(quaternion))
        assert np.allclose(geometry.matrix((0, 0, 0), geometry.quaternion(pose)), pose, atol=1e-9)


def test_line_keeps_the_orientation_of_a_half_turn_pose() -> None:
    start = geometry.matrix((0.6, 0.3, 1.0), (0.7071068, -0.7071068, 0, 0))
    goal = start.copy()
    goal[:3, 3] += (0, 0, 0.05)
    for waypoint in geometry.line(start, goal, 0.002):
        assert np.allclose(waypoint[:3, :3], start[:3, :3], atol=1e-6)


def test_inverse_undoes_a_pose() -> None:
    pose = geometry.matrix((0.4, -0.2, 1.1), (0.1, 0.2, 0.3, 0.9))
    assert np.allclose(geometry.inverse(pose) @ pose, np.eye(4))
