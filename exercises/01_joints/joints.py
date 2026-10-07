"""Exercise 1, joints. Shows position, velocity and effort of every joint in a live table.

    python3 exercises/01_joints/joints.py
"""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass

import agibot_gdk

from gdk_training import show
from gdk_training.jog import Jog
from gdk_training.joints import LIMITS


@dataclass(frozen=True, kw_only=True)
class Joint:
    name: str
    position: float  # rad
    velocity: float  # rad/s
    effort: float  # Nm


def connect() -> agibot_gdk.Robot:
    """Starts the GDK and returns a Robot that is ready to use.

    The GDK needs about 2 s after the Robot is created until the DDS connection delivers data.

    Raises:
        RuntimeError: gdk_init did not return kSuccess.
    """
    # TODO: Initialise the GDK, create the Robot, wait 2 s and return the Robot.
    raise NotImplementedError


def read_joints(robot: agibot_gdk.Robot) -> list[Joint]:
    """Reads the measured state of every joint.

    Returns:
        One Joint per joint, in the order the GDK reports them.
    """
    # TODO: Call get_joint_states and build one Joint per entry.
    raise NotImplementedError


def with_limits(joints: list[Joint]) -> list[dict]:
    """Adds the lower and upper limit of each joint as table columns."""
    unknown = (float("nan"), float("nan"))
    return [{**asdict(joint), "min": LIMITS.get(joint.name, unknown)[0], "max": LIMITS.get(joint.name, unknown)[1]}
            for joint in joints]


def main() -> None:
    robot = connect()
    try:
        jog = Jog(robot)
        show.live(lambda: with_limits(read_joints(robot)), keys=jog.keys, help=jog.help)
    finally:
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
