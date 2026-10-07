"""Exercise 8, TF. Shows frame positions from several sources to find the direction of a lookup.

    python3 exercises/08_tf/frames.py
    python3 exercises/08_tf/frames.py --all          prints every frame name
"""

from __future__ import annotations

import sys
import time

import agibot_gdk

from gdk_training import geometry, show
from gdk_training.jog import Jog

FRAMES = ["head_link3", "arm_l_end_link", "arm_r_end_link"]
TOOL = ("arm_l_end_link", "gripper_l_center_link")


def frame_names(tf: agibot_gdk.TF) -> list[str]:
    """Returns the name of every frame in the TF tree."""
    # TODO: Call get_all_frame_names.
    raise NotImplementedError


def lookup(tf: agibot_gdk.TF, target: str, source: str) -> geometry.Matrix:
    """Looks up the latest transform between two frames.

    Returns:
        The transform as a 4x4 matrix, exactly as lookup_transform_latest(target, source) reports
        it. The exercise is to find out in which direction this matrix maps a point.
    """
    # TODO: Call lookup_transform_latest. It returns a tuple of the Transform and a
    # timestamp. Convert the Transform with geometry.from_transform.
    raise NotImplementedError


def from_base(tf: agibot_gdk.TF, frame: str) -> geometry.Matrix:
    """Reads the transform of one frame through get_tf_from_base_link.

    Returns:
        The transform as a 4x4 matrix.
    """
    # TODO: Call get_tf_from_base_link and convert the result with geometry.from_transform.
    raise NotImplementedError


def status_poses(robot: agibot_gdk.Robot) -> dict[str, geometry.Matrix]:
    """Returns the pose of every frame of the motion control status in base_link, by frame name."""
    status = robot.get_motion_control_status()
    return {name: geometry.from_pose(pose) for name, pose in zip(status.frame_names, status.frame_poses)}


def compare(robot: agibot_gdk.Robot, tf: agibot_gdk.TF, frames: list[str]) -> list[dict[str, object]]:
    """Returns the position of each frame from four sources. The motion control status is the reference."""
    reference = status_poses(robot)
    rows = []
    for frame in frames:
        found = lookup(tf, "base_link", frame)
        rows.append({
            "frame": frame,
            "status": geometry.position(reference[frame]) if frame in reference else "-",
            "lookup(base_link, frame)": geometry.position(found),
            "inverse of it": geometry.position(geometry.inverse(found)),
            "get_tf_from_base_link": geometry.position(from_base(tf, frame)),
        })
    return rows


def tool_offset(tf: agibot_gdk.TF) -> dict[str, tuple[float, float, float]]:
    """Returns position and orientation of the lookup between the two TOOL frames."""
    offset = lookup(tf, *TOOL)
    return {"position": geometry.position(offset), "roll pitch yaw": geometry.rpy(offset)}


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    robot, tf = agibot_gdk.Robot(), agibot_gdk.TF()
    time.sleep(2.0)
    try:
        names = frame_names(tf)
        if "--all" in sys.argv:
            print("\n".join(names))
            return
        frames = [frame for frame in FRAMES if frame in names]
        sections = {"frame positions in base_link": lambda: compare(robot, tf, frames)}
        if all(frame in names for frame in TOOL):
            sections[f"lookup{TOOL}"] = lambda: tool_offset(tf)
        jog = Jog(robot)
        show.live(lambda: {title: read() for title, read in sections.items()}, keys=jog.keys, help=jog.help)
    finally:
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
