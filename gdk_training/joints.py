"""Joint names of the G2 and the limits from the GDK 2.6.3 reference, in rad.

The GDK 3.3.8 reference no longer lists limits and refers to the URDF. The limits the robot
enforces are tighter for some joints, so treat this table as an approximation.
"""

from __future__ import annotations

WAIST = [f"idx0{i}_body_joint{i}" for i in range(1, 6)]
HEAD = [f"idx1{i}_head_joint{i}" for i in range(1, 4)]
LEFT_ARM = [f"idx2{i}_arm_l_joint{i}" for i in range(1, 8)]
RIGHT_ARM = [f"idx6{i}_arm_r_joint{i}" for i in range(1, 8)]
ALL = WAIST + HEAD + LEFT_ARM + RIGHT_ARM

ARM_LIMITS = [(-3.072, 3.072), (-2.060, 2.060), (-3.072, 3.072), (-2.496, 1.012), (-3.072, 3.072), (-1.012, 1.012),
              (-1.536, 1.536)]

LIMITS: dict[str, tuple[float, float]] = {
    "idx01_body_joint1": (-1.082, 0.000),
    "idx02_body_joint2": (0.000, 2.653),
    "idx03_body_joint3": (-1.920, 1.571),
    "idx04_body_joint4": (-0.436, 0.436),
    "idx05_body_joint5": (-3.046, 3.046),
    "idx11_head_joint1": (-1.571, 1.571),
    "idx12_head_joint2": (-0.349, 0.349),
    "idx13_head_joint3": (-0.535, 0.535),
    **dict(zip(LEFT_ARM, ARM_LIMITS)),
    **dict(zip(RIGHT_ARM, ARM_LIMITS)),
}
