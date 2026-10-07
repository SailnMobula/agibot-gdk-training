"""Runs every solution against the fake GDK and checks that every exercise is still open."""

from __future__ import annotations

import importlib.util
import os
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "fake")]

import agibot_gdk  # noqa: E402

from gdk_training import geometry  # noqa: E402

ENV = os.environ | {"PYTHONPATH": f"{ROOT}:{ROOT / 'fake'}", "SHOW_ONCE": "1", "FAKE_INSTANT": "1"}
RUNS = [
    ("01_joints/joints.py", [], "idx27_arm_l_joint7"),
    ("02_status/status.py", [], "control_mode"),
    ("03_frames_and_forces/forces.py", [], "arm_l_end_link"),
    ("04_joint_moves/move.py", [], "control mode was 1"),
    ("05_gripper/gripper.py", [], "omnipicker"),
    ("06_servo/servo.py", ["idx27_arm_l_joint7"], "largest error"),
    ("07_ee_pose/ee_pose.py", ["left", "0", "0", "0.05"], "miss in mm"),
    ("08_tf/frames.py", [], "get_tf_from_base_link"),
    ("09_camera/camera.py", [], "head_depth: (400, 640) uint16"),
    ("10_camera_tf/project.py", [], "in camera"),
]


def run(folder: str, script: str, arguments: list[str], stdin: str = "") -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, str(ROOT / folder / script), *arguments], env=ENV, input=stdin,
                          capture_output=True, text=True, timeout=60)


def load(script: str) -> ModuleType:
    """Imports a solution script as a module. Dataclasses need the module in sys.modules."""
    spec = importlib.util.spec_from_file_location(Path(script).stem, ROOT / "solutions" / script)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("script, arguments, expected", RUNS)
def test_solution_runs(script: str, arguments: list[str], expected: str) -> None:
    result = run("solutions", script, arguments)
    assert result.returncode == 0, result.stderr
    assert expected in result.stdout


@pytest.mark.parametrize("script, arguments, expected", RUNS)
def test_exercise_is_open(script: str, arguments: list[str], expected: str) -> None:
    result = run("exercises", script, arguments)
    assert "NotImplementedError" in result.stderr


def test_prompt_moves_saves_and_returns(tmp_path: Path) -> None:
    lines = "idx13_head_joint3 0.2\nsave a\nhead 0 0 0\ngo a\nidx13_head_joint3 9\nq\n"
    script = tmp_path / "move.py"
    script.write_text((ROOT / "solutions/04_joint_moves/move.py").read_text())
    result = subprocess.run([sys.executable, str(script)], env=ENV, input=lines, capture_output=True, text=True,
                            timeout=60)
    assert (tmp_path / "poses.json").exists()
    assert result.returncode == 0, result.stderr
    assert result.stdout.count("result 0") == 3
    assert "outside" in result.stdout
    assert "+0.200" in result.stdout.rsplit("idx13_head_joint3", 1)[1]


def test_joint_move_is_refused_in_cartesian_impedance_mode() -> None:
    move = load("04_joint_moves/move.py")
    robot = agibot_gdk.Robot()
    robot.set_control_mode(2)
    with pytest.raises(RuntimeError):
        move.move(robot, {"idx11_head_joint1": 0.1}, 0.2)
    assert move.ensure_position_mode(robot) == 2
    assert move.move(robot, {"idx11_head_joint1": 0.1}, 0.2) == 0


def test_gripper_is_dropped_after_a_pose_stream() -> None:
    gripper, ee_pose = load("05_gripper/gripper.py"), load("07_ee_pose/ee_pose.py")
    robot = agibot_gdk.Robot()
    ee_pose.send(robot, "left", ee_pose.end_effector(robot, "left"), ee_pose.end_effector(robot, "right"))
    assert gripper.command(robot, gripper.CLOSED, gripper.CLOSED) == 0
    assert agibot_gdk._world.gripper_target["left"] == gripper.OPEN


def test_projection_lands_on_the_end_effector() -> None:
    project = load("10_camera_tf/project.py")
    robot, tf = agibot_gdk.Robot(), agibot_gdk.TF()
    robot.move_head_joint([0.2, 0.1, 0.0], [1.0] * 3)
    world = agibot_gdk._world
    expected = geometry.inverse(world.head_pose() @ agibot_gdk.HEAD_TO_CAMERA) @ world.end_effector_pose("l")[:, 3]
    point = project.camera_from_base(tf) @ project.end_effector_points(robot)["left"]
    assert np.allclose(point, expected)
    u, v = project.project(point, agibot_gdk.HEAD_INTRINSIC)
    assert 0 <= u < 640 and 0 <= v < 400


def test_one_gripper_closes_and_the_other_stays() -> None:
    gripper = load("05_gripper/gripper.py")
    world = agibot_gdk._world
    world.tool_blocked_until = 0.0
    world.gripper = {"left": gripper.OPEN, "right": gripper.OPEN}
    world.gripper_target = dict(world.gripper)
    assert gripper.close_one(agibot_gdk.Robot(), "left") == 0
    assert world.gripper_target == {"left": gripper.CLOSED, "right": gripper.OPEN}


@pytest.mark.parametrize("arguments", [["left", "0.09"], ["up", "0", "0", "0.05"], ["left", "0", "0", "0.2"], []])
def test_end_effector_script_rejects_invalid_arguments(arguments: list[str]) -> None:
    result = run("solutions", "07_ee_pose/ee_pose.py", arguments)
    assert result.returncode != 0
    assert "usage" in result.stderr or "offset" in result.stderr


def test_speed_above_the_cap_is_refused() -> None:
    result = run("solutions", "04_joint_moves/move.py", [], stdin="speed 2\nq\n")
    assert "at most 0.5 rad/s" in result.stdout


def test_joint_type_rejects_a_missing_field() -> None:
    joints = load("01_joints/joints.py")
    with pytest.raises(TypeError, match="effort"):
        joints.Joint(name="idx11_head_joint1", position=0.0, velocity=0.0)


def test_prompt_refuses_a_line_without_a_joint_name() -> None:
    result = run("solutions", "04_joint_moves/move.py", [], stdin="0.3\nq\n")
    assert "pairs of a joint name and an angle" in result.stdout
    assert "result 0" not in result.stdout
