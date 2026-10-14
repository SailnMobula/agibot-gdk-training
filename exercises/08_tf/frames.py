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
TOOL = ("arm_l_end_link", "gripper_l_center_link")  # target and source of the tool lookup


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
    poses: dict[str, geometry.Matrix] = {}
    # frame_names and frame_poses have the same order.
    for index, name in enumerate(status.frame_names):
        poses[name] = geometry.from_pose(status.frame_poses[index])
    return poses


def compare(robot: agibot_gdk.Robot, tf: agibot_gdk.TF, frames: list[str]) -> list[dict[str, object]]:
    """Returns the position of each frame from four sources. The motion control status is the reference."""
    reference = status_poses(robot)
    rows = []
    for frame in frames:
        if frame in reference:
            status_position = geometry.position(reference[frame])
        else:
            status_position = "-"  # the motion control status does not know every frame
        looked_up = lookup(tf, "base_link", frame)
        row = {
            "frame": frame,
            "status": status_position,
            "lookup(base_link, frame)": geometry.position(looked_up),
            "inverse of it": geometry.position(geometry.inverse(looked_up)),
            "get_tf_from_base_link": geometry.position(from_base(tf, frame)),
        }
        rows.append(row)
    return rows


def tool_offset(tf: agibot_gdk.TF) -> dict[str, tuple[float, float, float]]:
    """Returns position and orientation of the lookup between the two TOOL frames."""
    target, source = TOOL
    offset = lookup(tf, target, source)
    return {"position": geometry.position(offset), "roll pitch yaw": geometry.rpy(offset)}


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    robot = agibot_gdk.Robot()
    tf = agibot_gdk.TF()
    time.sleep(2.0)
    try:
        names = frame_names(tf)
        if "--all" in sys.argv:
            print("\n".join(names))
            return

        # Not every robot has every frame. Show only those the TF tree knows.
        frames = [frame for frame in FRAMES if frame in names]
        has_tool_frames = TOOL[0] in names and TOOL[1] in names

        def read_table() -> dict[str, object]:
            table: dict[str, object] = {"frame positions in base_link": compare(robot, tf, frames)}
            if has_tool_frames:
                table[f"lookup{TOOL}"] = tool_offset(tf)
            return table

        jog = Jog(robot)
        show.live(read_table, keys=jog.keys, help=jog.help)
    finally:
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
