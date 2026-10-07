"""Connection check for the real GDK. It only reads, so the robot does not move.

    source real.sh [ROBOT_IP]
    python -m gdk_training.doctor

Each printed line is one step. A failed step ends the check and prints what to look at.
"""

from __future__ import annotations

import importlib.util
import json
import os
import shutil
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

ENV = ("ROBOT_IP", "LOCATOR_IP", "AORTA_DISCOVERY_URI", "GDK_APP")


def report(ok: bool, text: str, hint: str = "") -> None:
    print(f"[{'ok' if ok else 'FAIL'}] {text}")
    if not ok:
        if hint:
            print(f"       {hint}")
        sys.exit(1)


def warn(text: str, hint: str) -> None:
    print(f"[warn] {text}\n       {hint}")


def firewall_blocks_robot(robot: str) -> bool:
    """Returns True when firewalld runs and does not trust the address of the robot.

    Reading data works in that case. Commands hang, because the robot has to open TCP connections
    back to this laptop on random ports and firewalld rejects them in its default zone.
    """
    if not shutil.which("firewall-cmd"):
        return False

    def run(*args: str) -> int:
        return subprocess.run(["firewall-cmd", *args], capture_output=True).returncode
    return run("--state") == 0 and run("--zone=trusted", f"--query-source={robot}") != 0


def port_open(host: str, port: int) -> bool:
    try:
        with socket.create_connection((host, port), timeout=3):
            return True
    except OSError:
        return False


def main() -> None:
    tag = f"cpython-{sys.version_info.major}{sys.version_info.minor}"
    spec = importlib.util.find_spec("agibot_gdk")
    folder = Path(spec.submodule_search_locations[0]) if spec and spec.submodule_search_locations else None
    report(folder is not None and any(folder.glob(f"*{tag}*.so")),
           f"Python {sys.version.split()[0]} with agibot_gdk from {folder}",
           "No agibot_gdk binding for this Python version was found. Build the wheel and install it, see SETUP.md.")
    missing = [name for name in ENV if not os.environ.get(name)]
    report(not missing, "environment " + ", ".join(f"{n}={os.environ[n]}" for n in ENV if n in os.environ),
           f"missing {', '.join(missing)}. Run `source real.sh [ROBOT_IP]` in this shell first.")

    robot = os.environ["ROBOT_IP"]
    report(port_open(robot, 2379), f"AORTA discovery {robot}:2379",
           "There is no route or no service. On the cable the laptop needs a 10.42.1.x address. On Wi-Fi check the robot address with ping.")
    report(port_open(robot, 8849), f"GDK package server {robot}:8849", "The gdk_http_server on the robot does not answer.")
    try:
        with urllib.request.urlopen(f"http://{robot}:2379/version", timeout=3) as reply:
            version = json.load(reply)
        report(True, f"AORTA service etcd {version.get('etcdserver')}")
    except (OSError, ValueError) as error:
        report(False, "AORTA /version", str(error))

    if firewall_blocks_robot(robot):
        warn("firewalld does not trust the robot",
             f"Reading works. Commands will hang, because the robot cannot connect back to this laptop. "
             f"Run: sudo firewall-cmd --zone=trusted --add-source={robot}   (see SETUP.md)")

    import agibot_gdk  # noqa: E402  The import needs the environment from real.sh.

    report(agibot_gdk.gdk_init() == agibot_gdk.GDKRes.kSuccess, "gdk_init",
           "The GDK did not reach AORTA. Check LOCATOR_IP is your address on the route to the robot.")
    try:
        bot = agibot_gdk.Robot()
        time.sleep(2.0)
        joints = bot.get_joint_states()["states"]
        report(len(joints) == 22, f"robot state: {len(joints)} joints")
        status = bot.get_motion_control_status()
        report(status is not None, "motion control status")

        tf = agibot_gdk.TF()
        time.sleep(1.0)
        frames = list(tf.get_all_frame_names())
        report(len(frames) > 0, f"TF: {len(frames)} frames")

        camera = agibot_gdk.Camera([agibot_gdk.CameraType.kHeadColor])
        time.sleep(3.0)  # The camera delivers its first frames about 3 s after it was created.
        try:
            image = None
            for _ in range(3):
                try:
                    image = camera.get_latest_image(agibot_gdk.CameraType.kHeadColor, 3000)
                except RuntimeError:
                    image = None
                if image is not None and len(image.data) > 0:
                    break
            report(image is not None and len(image.data) > 0,
                   "head camera image" + (f" {image.width}x{image.height}" if image else ""),
                   "No image after 3 tries. The camera may be off, or the image data path is blocked.")
        finally:
            camera.close_camera()
    finally:
        agibot_gdk.gdk_release()
    print("All checks passed.")


if __name__ == "__main__":
    main()
