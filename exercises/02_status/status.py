"""Exercise 2, status. Shows control mode, errors, emergency stops and batteries in a live table.

    python3 exercises/02_status/status.py
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass

import agibot_gdk

from gdk_training import show


@dataclass(frozen=True, kw_only=True)
class ControlState:
    mode: int  # 0 stop, 1 servo, 2 planning
    control_mode: int  # 1 joint position, 2 Cartesian impedance, 3 joint impedance
    error_code: int  # 0 means no error
    error_msg: str


@dataclass(frozen=True, kw_only=True)
class Battery:
    index: int  # position in the list of batteries, starting at 0
    soc: float  # state of charge in %
    voltage: float  # V
    current: float  # A
    temperature: float  # °C


def control_state(robot: agibot_gdk.Robot) -> ControlState:
    """Reads the state of the motion control."""
    # TODO: Call get_motion_control_status and copy the four attributes into a ControlState.
    raise NotImplementedError


def body_state(robot: agibot_gdk.Robot) -> dict[str, object]:
    """Reads the error, control and emergency stop flags of all body parts.

    Returns:
        The dict of get_whole_body_status without its "timestamp" entry.
    """
    # TODO: Call get_whole_body_status and remove the timestamp.
    raise NotImplementedError


def batteries(robot: agibot_gdk.Robot) -> list[Battery]:
    """Reads the state of every battery in the chassis."""
    # TODO: Call get_chassis_power_state and build one Battery per entry of battery_states.
    raise NotImplementedError


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    robot = agibot_gdk.Robot()
    time.sleep(2.0)
    try:
        show.live(lambda: {"motion control": control_state(robot), "body": body_state(robot),
                           "batteries": batteries(robot)}, hz=2.0)
    finally:
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
