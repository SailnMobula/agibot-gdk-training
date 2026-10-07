"""Exercise 3, frames and forces. Shows link poses, wrenches and external joint torques.

    python3 exercises/03_frames_and_forces/forces.py
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass

import agibot_gdk

from gdk_training import show
from gdk_training.jog import Jog
from gdk_training.joints import LEFT_ARM, RIGHT_ARM

Vector = tuple[float, float, float]  # x, y, z


@dataclass(frozen=True, kw_only=True)
class Frame:
    name: str
    position: Vector  # m, in base_link
    force: Vector  # N
    torque: Vector  # Nm


@dataclass(frozen=True, kw_only=True)
class JointTorque:
    joint: str
    torque: float  # Nm, estimated external torque


def frames(robot: agibot_gdk.Robot) -> list[Frame]:
    """Reads the position of every frame of the motion control status and the wrench acting on it.

    Returns:
        One Frame per entry of frame_names. frame_poses and wrenches have the same order.
    """
    # TODO: Call get_motion_control_status and combine frame_names, frame_poses and wrenches.
    raise NotImplementedError


def disturbance(robot: agibot_gdk.Robot) -> list[JointTorque]:
    """Reads the estimated external torque on every arm joint.

    Returns:
        14 entries. The seven joints of the left arm come first, then the right arm.
    """
    # TODO: Pair arm_disturbance_force of the motion control status with LEFT_ARM + RIGHT_ARM.
    raise NotImplementedError


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    robot = agibot_gdk.Robot()
    time.sleep(2.0)
    try:
        jog = Jog(robot, "idx27_arm_l_joint7")
        show.live(lambda: {"frames in base_link": frames(robot), "external joint torque": disturbance(robot)},
                  keys=jog.keys, help=jog.help)
    finally:
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
