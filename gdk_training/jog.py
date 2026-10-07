"""Jog keys for the live displays. They move one joint while the table stays on screen.

    jog = Jog(robot)
    show.live(read, keys=jog.keys, help=jog.help)

The keys a and d select the joint. The keys s and w move it by STEP rad at SPEED rad/s with
the blocking joint call from exercise 4.
"""

from __future__ import annotations

from collections.abc import Callable

import agibot_gdk

from gdk_training.joints import ALL, LIMITS

STEP = 0.1
SPEED = 0.2


class Jog:
    def __init__(self, robot: agibot_gdk.Robot, joint: str = "idx11_head_joint1") -> None:
        self.robot = robot
        self.index = ALL.index(joint)
        self.keys: dict[str, Callable[[], object]] = {
            "a": lambda: self.select(-1), "d": lambda: self.select(1),
            "s": lambda: self.step(-STEP), "w": lambda: self.step(STEP),
        }

    @property
    def joint(self) -> str:
        return ALL[self.index]

    def help(self) -> str:
        return f"jog {self.joint}   a d select joint   s w move by -/+ {STEP} rad"

    def select(self, direction: int) -> str:
        self.index = (self.index + direction) % len(ALL)
        return self.joint

    def step(self, delta: float) -> str:
        measured = next(state["motor_position"] for state in self.robot.get_joint_states()["states"]
                        if state["name"] == self.joint)
        low, high = LIMITS[self.joint]
        request = agibot_gdk.JointControlReq()
        request.life_time = 5.0
        request.joint_names = [self.joint]
        request.joint_positions = [max(low, min(high, measured + delta))]
        request.joint_velocities = [SPEED]
        if self.robot.joint_control_request(request) != 0:
            raise RuntimeError("joint_control_request did not return 0")
        return f"{self.joint} moved to {request.joint_positions[0]:+.3f}"
