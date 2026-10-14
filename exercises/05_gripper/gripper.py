"""Exercise 5, gripper. Shows the OmniPicker state and opens and closes it from the keyboard.

    python3 exercises/05_gripper/gripper.py
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass

import agibot_gdk

from gdk_training import show

OPEN = -0.785  # rad
CLOSED = 0.0  # rad
HALF_OPEN = OPEN / 2


@dataclass(frozen=True, kw_only=True)
class Gripper:
    side: str  # "left" or "right"
    type: str  # end effector model, "omnipicker" on this robot
    position: float  # rad
    velocity: float
    current: float
    temperature: float  # °C


def read_gripper(robot: agibot_gdk.Robot) -> list[Gripper]:
    """Reads the state of both grippers.

    Returns:
        Two entries, the left gripper first.
    """
    # TODO: Call get_end_state. Build one Gripper from left_end_state and one from
    # right_end_state. The joint values are in the first entry of end_states.
    raise NotImplementedError


def command(robot: agibot_gdk.Robot, left: float, right: float) -> int:
    """Commands both grippers to a position in rad. OPEN is -0.785 and CLOSED is 0.

    Returns:
        The result of move_ee_pos. It is 0 even when the gripper does not move.
    """
    # TODO: Fill one JointStates with group "dual_tool", target type "omnipicker" and one
    # JointState per side, the left side first. Set nums and send it with move_ee_pos.
    raise NotImplementedError


def close_one(robot: agibot_gdk.Robot, side: str) -> int:
    """Closes one gripper and commands the other one to its measured position."""
    positions: dict[str, float] = {}
    for gripper in read_gripper(robot):
        positions[gripper.side] = gripper.position
    positions[side] = CLOSED
    return command(robot, positions["left"], positions["right"])


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    robot = agibot_gdk.Robot()
    time.sleep(2.0)

    def read_table() -> list[Gripper]:
        return read_gripper(robot)

    def open_both() -> int:
        return command(robot, OPEN, OPEN)

    def close_both() -> int:
        return command(robot, CLOSED, CLOSED)

    def half_open_both() -> int:
        return command(robot, HALF_OPEN, HALF_OPEN)

    def close_left() -> int:
        return close_one(robot, "left")

    def close_right() -> int:
        return close_one(robot, "right")

    keys = {
        "o": open_both,
        "c": close_both,
        "h": half_open_both,
        "l": close_left,
        "r": close_right,
    }
    try:
        show.live(read_table, keys=keys, help="o open   c close   h half   l close left   r close right", hz=10.0)
    finally:
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
