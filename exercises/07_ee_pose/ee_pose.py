"""Exercise 7, end-effector pose. Moves one end effector on a straight line in base_link.

    python3 exercises/07_ee_pose/ee_pose.py left 0 0 0.05          offset dx dy dz in m
    python3 exercises/07_ee_pose/ee_pose.py left 0 0 -0.05 --grip   also commands the gripper after the move
"""

from __future__ import annotations

import sys
import time

import agibot_gdk
import numpy as np

from gdk_training import geometry, show

FRAMES = {"left": "arm_l_end_link", "right": "arm_r_end_link"}
RATE_HZ = 50.0
LIFE_TIME = 0.02
STEP_M = 0.002
HOLD_S = 1.0
MAX_OFFSET_M = 0.10
USAGE = "usage: ee_pose.py <left|right> <dx> <dy> <dz> [--grip], offsets in m"


def end_effector(robot: agibot_gdk.Robot, side: str) -> geometry.Matrix:
    """Reads the current pose of one end effector.

    Args:
        side: "left" or "right". FRAMES maps the side to the frame name.

    Returns:
        The pose of the end-effector frame in base_link as a 4x4 matrix.
    """
    # TODO: Find the frame in frame_names of get_motion_control_status, take the entry of
    # frame_poses at the same index and convert it with geometry.from_pose.
    raise NotImplementedError


def send(robot: agibot_gdk.Robot, side: str, target: geometry.Matrix, idle: geometry.Matrix) -> int:
    """Sends one pose for the end effector of one arm. The call does not wait for the arm.

    Args:
        side: The arm that moves, "left" or "right".
        target: Pose for the end effector of that arm, in base_link.
        idle: Current pose of the other end effector. The request needs a pose for both arms.

    Returns:
        The result of end_effector_pose_control, 0 on success.
    """
    # TODO: Fill one EndEffectorPose with LIFE_TIME and the group kLeftArm or kRightArm
    # from agibot_gdk.EndEffectorControlGroup. Write target and idle into the two pose attributes
    # with geometry.fill_pose, then send the request.
    raise NotImplementedError


def gripper_positions(robot: agibot_gdk.Robot) -> list[float]:
    """Returns the position of the left and the right gripper in rad."""
    end_state = robot.get_end_state()
    return [end_state[f"{side}_end_state"]["end_states"][0]["position"] for side in ("left", "right")]


def time_gripper(robot: agibot_gdk.Robot, position: float) -> None:
    """Commands both grippers every 0.5 s and prints the time until they start to move."""
    before = gripper_positions(robot)
    started = time.monotonic()
    while time.monotonic() - started < 10.0:
        request = agibot_gdk.JointStates()
        request.group, request.target_type = "dual_tool", "omnipicker"
        states = [agibot_gdk.JointState(), agibot_gdk.JointState()]
        for state in states:
            state.position = position
        request.states, request.nums = states, 2
        result = robot.move_ee_pos(request)
        time.sleep(0.5)
        moved = max(abs(now - then) for now, then in zip(gripper_positions(robot), before))
        print(f"t {time.monotonic() - started:4.1f} s  move_ee_pos returned {result}  moved {moved:.3f} rad")
        if moved > 0.05:
            return


def arguments() -> tuple[str, np.ndarray]:
    """Returns the side and the offset from the command line and ends the script on invalid input."""
    words = [word for word in sys.argv[1:] if word != "--grip"]
    try:
        side, offset = words[0], np.array([float(word) for word in words[1:]])
    except (IndexError, ValueError):
        sys.exit(USAGE)
    if side not in FRAMES or offset.shape != (3,):
        sys.exit(USAGE)
    if np.linalg.norm(offset) > MAX_OFFSET_M:
        sys.exit(f"The offset is longer than {MAX_OFFSET_M} m.")
    return side, offset


def stream(robot: agibot_gdk.Robot, side: str, target: geometry.Matrix, idle: geometry.Matrix) -> None:
    """Sends one pose and raises when the GDK does not accept it."""
    if send(robot, side, target, idle) != 0:
        raise RuntimeError("end_effector_pose_control did not return 0")


def main() -> None:
    side, offset = arguments()
    other = "right" if side == "left" else "left"
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    robot = agibot_gdk.Robot()
    time.sleep(2.0)
    try:
        start, idle = end_effector(robot, side), end_effector(robot, other)
        goal = start.copy()
        goal[:3, 3] += offset
        rate = show.Rate(RATE_HZ)
        for waypoint in geometry.line(start, goal, STEP_M):
            stream(robot, side, waypoint, idle)
            rate.sleep()
        for _ in range(int(HOLD_S * RATE_HZ)):
            stream(robot, side, goal, idle)
            rate.sleep()
        reached = end_effector(robot, side)
        print(show.fields({"start": geometry.position(start), "goal": geometry.position(goal),
                           "reached": geometry.position(reached),
                           "miss in mm": float(np.linalg.norm(reached[:3, 3] - goal[:3, 3]) * 1000)}))
        if "--grip" in sys.argv:
            time_gripper(robot, 0.0 if gripper_positions(robot)[0] < -0.4 else -0.785)
    finally:
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
