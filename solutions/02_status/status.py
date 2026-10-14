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
    status = robot.get_motion_control_status()
    state = ControlState(
        mode=status.mode,
        control_mode=status.control_mode,
        error_code=status.error_code,
        error_msg=status.error_msg,
    )
    return state


def body_state(robot: agibot_gdk.Robot) -> dict[str, object]:
    """Reads the error, control and emergency stop flags of all body parts.

    Returns:
        The dict of get_whole_body_status without its "timestamp" entry.
    """
    # TODO: Call get_whole_body_status and remove the timestamp.
    status = dict(robot.get_whole_body_status())  # a copy, so that pop does not change the GDK result
    status.pop("timestamp", None)
    return status


def batteries(robot: agibot_gdk.Robot) -> list[Battery]:
    """Reads the state of every battery in the chassis."""
    # TODO: Call get_chassis_power_state and build one Battery per entry of battery_states.
    power = robot.get_chassis_power_state()
    result = []
    for index, battery_state in enumerate(power.battery_states):
        battery = Battery(
            index=index,
            soc=battery_state.battery_soc,
            voltage=battery_state.battery_output_voltage,
            current=battery_state.battery_output_current,
            temperature=battery_state.battery_temperature,
        )
        result.append(battery)
    return result


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    robot = agibot_gdk.Robot()
    time.sleep(2.0)

    def read_table() -> dict[str, object]:
        table = {
            "motion control": control_state(robot),
            "body": body_state(robot),
            "batteries": batteries(robot),
        }
        return table

    try:
        show.live(read_table, hz=2.0)
    finally:
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
