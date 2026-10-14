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

GRIPPER_OPEN = -0.785  # rad
GRIPPER_CLOSED = 0.0  # rad
GRIPPER_HALF_OPEN = -0.4  # rad, a gripper below this counts as open
GRIPPER_TIMEOUT_S = 10.0  # give up when the gripper has not moved by then
GRIPPER_RETRY_S = 0.5  # pause between two gripper commands
GRIPPER_MOVED = 0.05  # rad, a change larger than this means the gripper moves


def end_effector(robot: agibot_gdk.Robot, side: str) -> geometry.Matrix:
    """Reads the current pose of one end effector.

    Args:
        side: "left" or "right". FRAMES maps the side to the frame name.

    Returns:
        The pose of the end-effector frame in base_link as a 4x4 matrix.
    """
    # TODO: Find the frame in frame_names of get_motion_control_status, take the entry of
    # frame_poses at the same index and convert it with geometry.from_pose.
    status = robot.get_motion_control_status()
    frame_names = list(status.frame_names)
    index = frame_names.index(FRAMES[side])
    pose = status.frame_poses[index]
    return geometry.from_pose(pose)


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
    request = agibot_gdk.EndEffectorPose()
    request.life_time = LIFE_TIME
    if side == "left":
        request.group = int(agibot_gdk.EndEffectorControlGroup.kLeftArm)
        left_pose = target
        right_pose = idle
    else:
        request.group = int(agibot_gdk.EndEffectorControlGroup.kRightArm)
        left_pose = idle
        right_pose = target
    geometry.fill_pose(request.left_end_effector_pose, left_pose)
    geometry.fill_pose(request.right_end_effector_pose, right_pose)
    return robot.end_effector_pose_control(request)


def gripper_positions(robot: agibot_gdk.Robot) -> list[float]:
    """Returns the position of the left and the right gripper in rad."""
    end_state = robot.get_end_state()
    positions = []
    for side in ("left", "right"):
        joint = end_state[f"{side}_end_state"]["end_states"][0]
        positions.append(joint["position"])
    return positions


def command_grippers(robot: agibot_gdk.Robot, position: float) -> int:
    """Commands both grippers to the same position in rad, like command in exercise 5."""
    left_state = agibot_gdk.JointState()
    left_state.position = position
    right_state = agibot_gdk.JointState()
    right_state.position = position

    request = agibot_gdk.JointStates()
    request.group = "dual_tool"
    request.target_type = "omnipicker"
    request.states = [left_state, right_state]
    request.nums = 2
    return robot.move_ee_pos(request)


def time_gripper(robot: agibot_gdk.Robot, position: float) -> None:
    """Commands both grippers every GRIPPER_RETRY_S and prints the time until they start to move."""
    before = gripper_positions(robot)
    started = time.monotonic()
    while time.monotonic() - started < GRIPPER_TIMEOUT_S:
        result = command_grippers(robot, position)
        time.sleep(GRIPPER_RETRY_S)

        now = gripper_positions(robot)
        left_moved = abs(now[0] - before[0])
        right_moved = abs(now[1] - before[1])
        moved = max(left_moved, right_moved)
        elapsed = time.monotonic() - started
        print(f"t {elapsed:4.1f} s  move_ee_pos returned {result}  moved {moved:.3f} rad")
        if moved > GRIPPER_MOVED:
            return


def arguments() -> tuple[str, np.ndarray]:
    """Returns the side and the offset from the command line and ends the script on invalid input."""
    words = [word for word in sys.argv[1:] if word != "--grip"]
    try:
        side = words[0]
        offset_values = [float(word) for word in words[1:]]
    except (IndexError, ValueError):
        sys.exit(USAGE)
    offset = np.array(offset_values)
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
    if side == "left":
        other = "right"
    else:
        other = "left"
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    robot = agibot_gdk.Robot()
    time.sleep(2.0)
    try:
        start = end_effector(robot, side)
        idle = end_effector(robot, other)  # the other arm keeps this pose
        goal = start.copy()
        goal[:3, 3] += offset  # the column 3 of the matrix is the position

        rate = show.Rate(RATE_HZ)
        for waypoint in geometry.line(start, goal, STEP_M):
            stream(robot, side, waypoint, idle)
            rate.sleep()
        # Keep sending the goal for a moment, so that the arm settles there.
        for _ in range(int(HOLD_S * RATE_HZ)):
            stream(robot, side, goal, idle)
            rate.sleep()

        reached = end_effector(robot, side)
        miss_m = np.linalg.norm(reached[:3, 3] - goal[:3, 3])
        result = {
            "start": geometry.position(start),
            "goal": geometry.position(goal),
            "reached": geometry.position(reached),
            "miss in mm": float(miss_m * 1000),
        }
        print(show.fields(result))

        if "--grip" in sys.argv:
            left_gripper = gripper_positions(robot)[0]
            if left_gripper < GRIPPER_HALF_OPEN:
                time_gripper(robot, GRIPPER_CLOSED)
            else:
                time_gripper(robot, GRIPPER_OPEN)
    finally:
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
