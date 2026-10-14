"""Exercise 4, joint moves. A prompt sends targets through the blocking joint call.

    python3 exercises/04_joint_moves/move.py

The prompt accepts the following lines.

    > idx13_head_joint3 0.2                 moves one joint to 0.2 rad
    > idx27_arm_l_joint7 0.3 idx67_arm_r_joint7 -0.3
    > head 0.3 0.1 0                        moves the head to yaw, pitch, roll
    > save a                                stores the measured joints as pose "a"
    > go a                                  moves to pose "a"
    > speed 0.1                             sets the speed in rad/s for the following moves
    > q                                     ends the script
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import agibot_gdk

from gdk_training import show
from gdk_training.joints import ALL

POSES = Path(__file__).with_name("poses.json")
SPEED = 0.2
MAX_SPEED = 0.5
USAGE = """Enter one of these lines.
  <joint> <rad> [<joint> <rad> ...]   move joints to absolute angles, e.g. idx13_head_joint3 0.2
  head <yaw> <pitch> <roll>           move the head
  save <name>                         store the measured joints as a pose
  go <name>                           move to a stored pose
  speed <rad/s>                       set the speed for the following moves
  q                                   quit"""
LIFE_TIME = 5.0
JOINT_POSITION_MODE = 1  # control_mode of the motion control status; the joint calls need it


def ensure_position_mode(robot: agibot_gdk.Robot) -> int:
    """Makes sure the robot is in joint position control, which the joint calls require.

    Returns:
        The control mode the robot was in before the call (1, 2 or 3).
    """
    # TODO: Read control_mode from get_motion_control_status. Call
    # set_control_mode(JOINT_POSITION_MODE) when it has another value.
    raise NotImplementedError


def measured(robot: agibot_gdk.Robot) -> dict[str, float]:
    """Returns the measured position of every body joint in rad, by joint name."""
    joint_states = robot.get_joint_states()
    positions = {}
    for state in joint_states["states"]:
        # get_joint_states also reports joints that are not part of the body, e.g. the grippers.
        if state["name"] in ALL:
            positions[state["name"]] = state["motor_position"]
    return positions


def move(robot: agibot_gdk.Robot, targets: dict[str, float], speed: float) -> int:
    """Moves a set of joints to target angles and waits until the robot has arrived.

    Args:
        targets: Joint name to target angle in rad.
        speed: Speed of every joint in rad/s.

    Returns:
        The result of joint_control_request, 0 on success.

    Raises:
        RuntimeError: The robot rejected the request, for example for a target outside the limits.
    """
    # TODO: Fill one JointControlReq with the names, the positions, one velocity per joint
    # and LIFE_TIME, then send it with joint_control_request.
    raise NotImplementedError


def move_head(robot: agibot_gdk.Robot, yaw: float, pitch: float, roll: float, speed: float) -> int:
    """Moves the three head joints to the given angles in rad and waits until they have arrived.

    Returns:
        The result of move_head_joint, 0 on success.
    """
    # TODO: Call move_head_joint with the three positions and three velocities.
    raise NotImplementedError


def read_poses() -> dict[str, dict[str, float]]:
    """Returns the stored poses by name, an empty dict when none are stored yet."""
    if not POSES.exists():
        return {}
    return json.loads(POSES.read_text())


def parse_targets(words: list[str]) -> dict[str, float]:
    """Turns the words "joint angle joint angle ..." into joint name to angle in rad."""
    if len(words) % 2 != 0:
        raise ValueError("The line needs pairs of a joint name and an angle in rad.")
    targets = {}
    for index in range(0, len(words), 2):
        name = words[index]
        angle = float(words[index + 1])
        if name not in ALL:
            raise ValueError("The line needs pairs of a joint name and an angle in rad.")
        targets[name] = angle
    return targets


def run(robot: agibot_gdk.Robot, line: str, speed: float) -> float:
    """Executes one prompt line and returns the speed for the next one."""
    words = line.split()
    command = words[0]

    if command == "speed":
        new_speed = float(words[1])
        if not 0 < new_speed <= MAX_SPEED:
            raise ValueError(f"The speed has to be above 0 and at most {MAX_SPEED} rad/s.")
        return new_speed

    if command == "save":
        pose_name = words[1]
        poses = read_poses()
        poses[pose_name] = measured(robot)
        POSES.write_text(json.dumps(poses, indent=2))
        print(f"saved {pose_name}")
        return speed

    started = time.monotonic()
    if command == "head":
        yaw = float(words[1])
        pitch = float(words[2])
        roll = float(words[3])
        result = move_head(robot, yaw, pitch, roll, speed)
    elif command == "go":
        pose_name = words[1]
        poses = read_poses()
        result = move(robot, poses[pose_name], speed)
    else:
        targets = parse_targets(words)
        result = move(robot, targets, speed)
    duration = time.monotonic() - started
    print(f"result {result} after {duration:.1f} s")
    return speed


def print_joints(robot: agibot_gdk.Robot) -> None:
    rows = []
    for name, position in measured(robot).items():
        rows.append({"joint": name, "position": position})
    print(show.table(rows))


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    robot = agibot_gdk.Robot()
    time.sleep(2.0)
    speed = SPEED
    try:
        previous_mode = ensure_position_mode(robot)
        print(f"control mode was {previous_mode}")
        print(USAGE)
        while True:
            print_joints(robot)
            try:
                line = input(f"\n{speed} rad/s > ").strip()
            except EOFError:  # the input ended, e.g. a file piped into the script
                break
            if line == "q":
                break
            if not line:
                continue
            try:
                speed = run(robot, line, speed)
            except (RuntimeError, KeyError, ValueError, IndexError) as error:
                # A typo or a rejected target should not end the script.
                print(f"{type(error).__name__}: {error}\n{USAGE}")
    finally:
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
