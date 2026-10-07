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


def measure(robot: agibot_gdk.Robot, joint: str) -> float:
    """Returns the measured position of one joint in rad."""
    return next(state["motor_position"] for state in robot.get_joint_states()["states"] if state["name"] == joint)


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
    joint = sys.argv[1] if len(sys.argv) > 1 else "idx27_arm_l_joint7"
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    robot = agibot_gdk.Robot()
    time.sleep(2.0)
    try:
        start = measure(robot, joint)
        low, high = LIMITS[joint]
        if not (low < start - AMPLITUDE and start + AMPLITUDE < high):
            sys.exit(f"The sine around {start:+.3f} rad would leave the limits {low} to {high} of {joint}.")
        ticks = int(PERIODS / FREQUENCY * RATE_HZ)
        rate = show.Rate(RATE_HZ)
        worst = 0.0
        for tick in range(ticks + 1):
            target = setpoint(start, tick / RATE_HZ)
            if send(robot, joint, target) != 0:
                raise RuntimeError("joint_servo_control did not return 0")
            error = target - measure(robot, joint)
            worst = max(worst, abs(error))
            if tick % 6 == 0:
                print(f"t {tick / RATE_HZ:5.2f}  target {target:+.3f}  error {error:+.4f}")
            rate.sleep()
        for _ in range(int(HOLD_S * RATE_HZ)):
            send(robot, joint, start)
            rate.sleep()
        print(f"largest error {worst:.4f} rad, back at {measure(robot, joint):+.3f} from {start:+.3f}")
    finally:
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
