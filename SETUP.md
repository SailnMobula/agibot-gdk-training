# Laptop setup for the real GDK

This page explains how the GDK on a laptop talks to the robot, what the setup consists of and
what was tested. The observations come from one G2 with GDK 3.3.8 and an Ubuntu 24.04 laptop
on x86_64, made on 2026-10-07. Other units can differ.

## Quick start

```bash
scripts/install_gdk.sh <ROBOT_IP>     # once
uv sync                               # once
uv pip install <agibot_gdk wheel for Python 3.12>     # once, see "Python binding"
source real.sh <ROBOT_IP>             # in every new shell
python -m gdk_training.doctor
```

Over the Debug cable the address can be left out in both commands. `uv sync` reads
`.python-version` and creates the environment `.venv` with Python 3.12.

The doctor only reads from the robot. It ends with `All checks passed.` when the GDK can read
the joints, TF and the head camera. After that every exercise runs the same way as with the
fake, for example `python exercises/01_joints/joints.py`.

## Architecture

```
 Laptop, x86_64                                          G2 robot, aarch64
 --------------------------------                        ---------------------------------
 Python 3.12 in .venv                                    system Python 3.12
   agibot_gdk wheel built for CPython 3.12                 agibot_gdk for CPython 3.12
   libgdk_core, libgdk_dds, libgdk_adapter                 gdk_service and the robot nodes
          |                                                        |
          |  1. discovery, HTTP  ---------------------------->  AORTA service (etcd)    port 2379
          |  2. data, TCP and UDP  <------------------------->  robot nodes
          |  3. install only, HTTP  ------------------------->  gdk_http_server         port 8849
```

- The laptop and the robot exchange data over the network. Each side needs the binding for its
  own CPU and Python version.
- The package the robot serves for laptops contains a prebuilt binding for CPython 3.10 on
  x86_64. This repository uses Python 3.12, so the laptop needs a binding built for 3.12. The
  section [Python binding](#python-binding) describes the build. A Python version without a
  matching binding fails with `No module named 'agibot_gdk.agibot_gdk'`.
- The GDK finds the robot through the discovery service. It reads the address of that service
  from `AORTA_DISCOVERY_URI`. When the variable is missing, the GDK uses `http://127.0.0.1:2379`
  and logs `aorta domain init failed`.
- The robot has several networks. Its address on the Debug port is always `10.42.1.101`. Its
  Wi-Fi address comes from the router and can change, so check it before a session.

## Python binding

The GDK package contains the sources of the Python binding in
`~/.cache/agibot/app/gdk/build_dep/python/pybind/`. Build a wheel for Python 3.12 from them and
install it into the environment of this repository.

```bash
sudo apt update && sudo apt install -y libprotobuf-dev protobuf-compiler
source .venv/bin/activate
uv pip install pybind11 setuptools wheel
cd ~/.cache/agibot/app/gdk/build_dep/python/pybind/
python setup.py bdist_wheel --dist-dir dist
uv pip install dist/agibot_gdk-*.whl
```

`uv sync` removes packages that are not in `uv.lock`, and the wheel is one of them. Install the
wheel again after a `uv sync`, or run `uv sync --inexact`, which keeps it.

`real.sh` prints the folder of the binding it found. It stops with a message when no binding
fits the active Python version.

## What is installed where

| Item | Location | Created by |
| --- | --- | --- |
| GDK package with libraries, binding and examples | `~/.cache/agibot/app` | `scripts/install_gdk.sh` |
| Python 3.12 interpreter | system, or `~/.local/share/uv/python/` | `uv sync` |
| `agibot_gdk` wheel for Python 3.12 | `.venv/` in this repository | `uv pip install` |
| Python environment | `.venv/` in this repository | `uv sync` |

`real.sh` sets the environment for the current shell and writes nothing to your dotfiles.
`rm -rf ~/.cache/agibot .venv` removes the whole setup.

## Environment set by `real.sh`

| Variable | Value | Needed |
| --- | --- | --- |
| `AORTA_DISCOVERY_URI` | `http://<ROBOT_IP>:2379` | Yes. Without it the GDK looks on `127.0.0.1` and fails. |
| `LD_LIBRARY_PATH` | every folder with libraries below `~/.cache/agibot/app/lib` | Yes. The binding links the GDK libraries. |
| `PYTHONPATH` | this repository. The folder `~/.cache/agibot/app/gdk/lib` is added only when its prebuilt binding fits the active Python. | Yes. |
| `LOCATOR_IP` | address of the laptop on the route to the robot | Recommended. The vendor script sets it. Reading also worked without it. |
| `DEV_IP` | same as `LOCATOR_IP` | Recommended. The DDS profile of the package uses it. |
| `APP_CONF_PATH` | `~/.cache/agibot/app/gdk/config/app_conf.json` | Optional. |
| `AORTA_DISPATCHER_THREAD_NUM` | `6` | Optional. It is the vendor value. |

The column "Needed" was checked by removing one variable at a time and reading the 22 joints.
`LOCATOR_IP` probably matters when the laptop is on the cable and on Wi-Fi at the same time.
This was not tested.

## Cable or Wi-Fi

| | Debug cable | Wi-Fi or LAN |
| --- | --- | --- |
| Robot address | `10.42.1.101`, the laptop gets a static `10.42.1.x` address | the address the router gives the robot |
| Environment | `source real.sh` | `source real.sh <ROBOT_IP>` |
| Install | `scripts/install_gdk.sh` | `scripts/install_gdk.sh <ROBOT_IP>` |
| Supported by the vendor | yes | no |
| Tested here | not yet | exercises 1, 2, 3, 8, 9 and 10 |
| Measured | not yet | 100 pings with 0 % loss, 17 ms on average and 124 ms in the worst case |

Use the cable for exercises 6 and 7. They stream setpoints at a fixed rate, and a delay of
this size can disturb the stream. This is an expectation and was not tested. Exercises that
read data and blocking joint moves do not depend on timing.

The vendor tools that use FastDDS, such as the ROS bridge, are configured for the cable
networks of the robot. They are not expected to work over Wi-Fi. This follows from the
configuration files and was not tested.

## Firewall

Reading data works as soon as the laptop can reach the robot. Commands need a second
condition. The laptop publishes the command topics, and the robot opens a TCP connection back
to the laptop on a random high port to receive them. A firewall on the laptop that rejects
this connection has a typical symptom. A call such as `joint_control_request` or
`move_head_joint` never returns, and the joint does not move.

On Fedora, RHEL and similar systems firewalld is active by default. Trust the address of the
robot.

```bash
sudo firewall-cmd --zone=trusted --add-source=<ROBOT_IP>               # until the next reload
sudo firewall-cmd --permanent --zone=trusted --add-source=<ROBOT_IP>   # permanent
```

With the cable the address is `10.42.1.101`. Another firewall such as `ufw` needs a rule that
allows all incoming connections from the robot. The doctor prints a warning when firewalld is
running and does not trust the robot.

The connection can be tested without the GDK. Start a server on the laptop.

```bash
python3 -m http.server 34999 --bind <LAPTOP_IP>
```

Then connect to it from a shell on the robot.

```bash
timeout 5 bash -c 'echo > /dev/tcp/<LAPTOP_IP>/34999'
```

The answer `No route to host` means that the firewall rejects the robot.

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| `aorta domain init failed: http://127.0.0.1:2379` | `AORTA_DISCOVERY_URI` is not set. | Run `source real.sh <ROBOT_IP>`. |
| `No module named 'agibot_gdk.agibot_gdk'` | Python found a binding for another Python version. | Install the wheel for Python 3.12 into `.venv`, see [Python binding](#python-binding). |
| `real.sh` reports that no binding was found. | The wheel is not installed in `.venv`. `uv sync` removes it again. | Install the wheel again with `uv pip install`. |
| `libgdk_core.so.3: cannot open shared object file` | `LD_LIBRARY_PATH` is not set. | Run `source real.sh`. |
| The installer ends with `curl: (7)` or a timeout. | The robot is not reachable. | Check the address with `ping <ROBOT_IP>`. The Wi-Fi address can have changed. |
| The doctor stops at `AORTA discovery`. | There is no route to the robot, or the service on the robot is down. | Check the address and the network. |
| A joint move never returns and the joint does not move, while reading works. | The firewall of the laptop rejects the connection from the robot. | Trust the address of the robot, see [Firewall](#firewall). |
| A script prints nothing when its output goes through a pipe or `timeout`. | Python buffers the output. | Start it with `python -u`. |
| A wrist camera shows no image. | On the tested robot both wrist cameras report size 0 and 0 fps. | This is an open point. The head cameras work. |

## Differences from the vendor instructions

| Vendor | This repository | Reason |
| --- | --- | --- |
| `curl http://10.42.1.101:8849/install.sh \| bash` | `scripts/install_gdk.sh [ROBOT_IP]` | The robot address is a parameter, an interrupted download resumes, and the old install stays until the new one is ready. |
| `source ~/.cache/agibot/app/env.sh` | `source real.sh [ROBOT_IP]` | It works over Wi-Fi and LAN, activates the environment and reports a missing setup. |
| prebuilt binding for Python 3.10 | wheel built for Python 3.12 | The exercises and the robot use Python 3.12. |
