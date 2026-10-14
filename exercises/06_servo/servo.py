"""Exercise 6, servo. One joint follows a sine. The script sends one setpoint per tick.

    python3 exercises/06_servo/servo.py idx27_arm_l_joint7
"""

from __future__ import annotations

import math
import sys
import time

import agibot_gdk

from gdk_training import show
from gdk_training.joints import LIMITS

RATE_HZ = 30.0
CONTROL_PERIOD = 0.035  # slightly longer than one tick, as the GDK reference asks
AMPLITUDE = 0.15  # rad
FREQUENCY = 0.25  # Hz
PERIODS = 2
HOLD_S = 0.5
PRINT_EVERY = 6  # ticks between two printed lines, 5 lines per second
DEFAULT_JOINT = "idx27_arm_l_joint7"


def measure(robot: agibot_gdk.Robot, joint: str) -> float:
    """Returns the measured position of one joint in rad."""
    for state in robot.get_joint_states()["states"]:
        if state["name"] == joint:
            return state["motor_position"]
    raise KeyError(f"the robot reports no joint {joint}")


def setpoint(start: float, t: float) -> float:
    """Computes the position the joint should have at time t.

    Args:
        start: Position of the joint when the motion began, in rad.
        t: Time since the motion began, in s.

    Returns:
        A sine around start with AMPLITUDE and FREQUENCY, in rad. At t = 0 it equals start.
    """
    # TODO: Return start plus the sine.
    raise NotImplementedError


def send(robot: agibot_gdk.Robot, joint: str, position: float) -> int:
    """Sends one setpoint in rad for one joint. The call does not wait for the joint to arrive.

    Returns:
        The result of joint_servo_control, 0 on success.
    """
    # TODO: Fill one JointServoControlReq with CONTROL_PERIOD, the joint name and the
    # position, then send it with joint_servo_control. Leave enable_low_latency off.
    raise NotImplementedError


def main() -> None:
    if len(sys.argv) > 1:
        joint = sys.argv[1]
    else:
        joint = DEFAULT_JOINT
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    robot = agibot_gdk.Robot()
    time.sleep(2.0)
    try:
        start = measure(robot, joint)

        # The sine must stay inside the joint limits, also at its lowest and highest point.
        low, high = LIMITS[joint]
        lowest_target = start - AMPLITUDE
        highest_target = start + AMPLITUDE
        if lowest_target <= low or highest_target >= high:
            sys.exit(f"The sine around {start:+.3f} rad would leave the limits {low} to {high} of {joint}.")

        duration_s = PERIODS / FREQUENCY
        ticks = int(duration_s * RATE_HZ)
        rate = show.Rate(RATE_HZ)
        largest_error = 0.0
        for tick in range(ticks + 1):
            t = tick / RATE_HZ
            target = setpoint(start, t)
            if send(robot, joint, target) != 0:
                raise RuntimeError("joint_servo_control did not return 0")
            error = target - measure(robot, joint)
            largest_error = max(largest_error, abs(error))
            if tick % PRINT_EVERY == 0:
                print(f"t {t:5.2f}  target {target:+.3f}  error {error:+.4f}")
            rate.sleep()

        # Keep sending the start position for a moment, so that the joint settles there.
        hold_ticks = int(HOLD_S * RATE_HZ)
        for _ in range(hold_ticks):
            send(robot, joint, start)
            rate.sleep()
        end = measure(robot, joint)
        print(f"largest error {largest_error:.4f} rad, back at {end:+.3f} from {start:+.3f}")
    finally:
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
