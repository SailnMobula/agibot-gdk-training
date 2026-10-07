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


def ensure_position_mode(robot: agibot_gdk.Robot) -> int:
    """Makes sure the robot is in joint position control, which the joint calls require.

    Returns:
        The control mode the robot was in before the call (1, 2 or 3).
    """
    # TODO: Read control_mode from get_motion_control_status. Call set_control_mode(1) when
    # it has another value.
    raise NotImplementedError


def measured(robot: agibot_gdk.Robot) -> dict[str, float]:
    """Returns the measured position of every body joint in rad, by joint name."""
    return {state["name"]: state["motor_position"] for state in robot.get_joint_states()["states"]
            if state["name"] in ALL}


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


def run(robot: agibot_gdk.Robot, line: str, speed: float) -> float:
    """Executes one prompt line and returns the speed for the next one."""
    words = line.split()
    poses = json.loads(POSES.read_text()) if POSES.exists() else {}
    if words[0] == "speed":
        if not 0 < float(words[1]) <= MAX_SPEED:
            raise ValueError(f"The speed has to be above 0 and at most {MAX_SPEED} rad/s.")
        return float(words[1])
    if words[0] == "save":
        poses[words[1]] = measured(robot)
        POSES.write_text(json.dumps(poses, indent=2))
        print(f"saved {words[1]}")
        return speed
    started = time.monotonic()
    if words[0] == "head":
        result = move_head(robot, *(float(word) for word in words[1:4]), speed)
    elif words[0] == "go":
        result = move(robot, poses[words[1]], speed)
    else:
        if len(words) % 2 or any(name not in ALL for name in words[::2]):
            raise ValueError("The line needs pairs of a joint name and an angle in rad.")
        result = move(robot, {name: float(value) for name, value in zip(words[::2], words[1::2])}, speed)
    print(f"result {result} after {time.monotonic() - started:.1f} s")
    return speed


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    robot = agibot_gdk.Robot()
    time.sleep(2.0)
    speed = SPEED
    try:
        print(f"control mode was {ensure_position_mode(robot)}")
        print(USAGE)
        while True:
            print(show.table([{"joint": name, "position": position} for name, position in measured(robot).items()]))
            try:
                line = input(f"\n{speed} rad/s > ").strip()
            except EOFError:
                break
            if line == "q":
                break
            if not line:
                continue
            try:
                speed = run(robot, line, speed)
            except (RuntimeError, KeyError, ValueError, IndexError) as error:
                print(f"{type(error).__name__}: {error}\n{USAGE}")
    finally:
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
