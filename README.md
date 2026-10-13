# GDK training

Exercises for the AGIBOT G2 on GDK 3.3.8. They cover reading the robot state, moving the robot,
coordinate frames and cameras, and in a second part lidar, mapping, localization and navigation. Each exercise is one script with a few functions left open. These
functions contain the GDK calls. Output, keyboard handling and the image viewer are provided.

## Exercises

| # | Exercise | Robot moves | Content |
| --- | --- | --- | --- |
| 1 | Joints | no | position, velocity and effort of all 22 joints |
| 2 | Status | no | control mode, errors, e-stops, batteries |
| 3 | Frames and forces | no | link poses, wrenches, external joint torques |
| 4 | Joint moves | yes | the blocking joint call from a prompt, saved poses |
| 5 | Gripper | yes | OmniPicker state, open and close on keys |
| 6 | Servo | yes | streaming setpoints, one joint on a sine |
| 7 | End-effector pose | yes | end effector on a straight line in `base_link` |
| 8 | TF | no | frame list, lookups, direction of a transform |
| 9 | Camera | no | head and wrist images in the browser |
| 10 | Camera and TF | no | end effectors projected into the head image |
| 11 | Lidar | no | front and back point clouds decoded from bytes, moved into `base_link`, a height band, 3D view |
| 12 | Drive | yes | chassis on keys, dead-man switch, taking and releasing control |
| 13 | Mapping | yes | start, stop and cancel mapping, SLAM states, the new map, switching between maps |
| 14 | Map in 2D | no | occupancy grid, coordinates, guide points, the robot pose and the live lidar scan |
| 15 | Map in 3D | no | the point cloud of the map, transformed from the front lidar frame into the map frame, an own occupancy grid compared with the robot's |
| 16 | Navigate | yes | `normal_navi` and `relative_move`, task states, goal checks, goals by click |

Each folder in `exercises/` has a `README.md` with the task. Open functions are marked
`TODO` and raise `NotImplementedError` until you write them. The docstring of each
function states what it has to return, and the small classes at the top of a script name the
fields and their units. The Returns section of each
README lists the fields of the GDK calls you need. The reference implementation is in
`solutions/`.

## On a laptop with the robot

The scripts run on your laptop and talk to the robot over the network. This works over the
Debug cable and over Wi-Fi. The laptop needs Linux on x86_64, because the GDK package for
laptops is built for that platform.

```bash
scripts/install_gdk.sh <ROBOT_IP>     # once
uv sync                               # once
uv pip install <agibot_gdk wheel for Python 3.12>     # once
source real.sh <ROBOT_IP>             # in every new shell
python -m gdk_training.doctor
python exercises/01_joints/joints.py
```

`scripts/install_gdk.sh` downloads the GDK from the robot into `~/.cache/agibot/app`.
`uv sync` creates a Python 3.12 environment in `.venv`. The GDK package only contains a binding
for Python 3.10, so the laptop needs an `agibot_gdk` wheel built for Python 3.12.
[SETUP.md](SETUP.md#python-binding) describes the build. `real.sh` sets the environment for the
GDK and activates `.venv`. The doctor checks the connection and only reads from the robot.
Over the Debug cable the robot has the address `10.42.1.101`, and `<ROBOT_IP>` can be left out.

[SETUP.md](SETUP.md) explains the architecture, the difference between cable and Wi-Fi, the
firewall rule that commands need and the troubleshooting.

The robot serves the GDK 3.3.8 reference as a website at `http://<ROBOT_IP>:8849`. The Python
pages `robot`, `tf` and `camera` describe every call the exercises use.

`q` ends a script with a live table. `Ctrl+C` ends every script. Run one script at a time. The
live displays have jog keys, so you can move a joint while you watch the values. The camera
exercises serve their images at `http://localhost:8000`. Exercise 11 shows its point clouds in 3D
there, exercises 14 and 16 the map. Exercise 15 shows the map cloud in 3D at
`http://localhost:8001` and the grids at `8000`. Exercises 12 and 13 use only the terminal. Every
script prints its link when it starts.

## On the robot

The exercises also run on the robot itself. The GDK is already installed there.

```bash
ssh agi@<ROBOT_IP>
cd <folder of this repository>
source robot.sh
python3 exercises/01_joints/joints.py
```

The camera images are then at `http://<ROBOT_IP>:8000`.

## Without a robot

```bash
uv sync
source fake.sh
python exercises/01_joints/joints.py
```

`fake.sh` points `agibot_gdk` at `fake/agibot_gdk`. It offers the same calls and return shapes
without a robot. Joints move in a straight ramp to their target, link poses come from a toy model
and the head camera shows a grid with a cross on each end effector. Lidar, SLAM, maps and
navigation work on a map recorded on a G2: the lidars ray-cast against its grid, the chassis and
the planner move the robot on it. Virtual walls and no-go zones exist only on the robot. Use
the fake to get a function working before the robot is free.

`uv run pytest` runs every solution against the fake. It takes about a minute.

## Layout

| Path | Content |
| --- | --- |
| `exercises/` | scripts with open functions and the task READMEs |
| `solutions/` | the same scripts, complete |
| `gdk_training/show.py` | live tables, key bindings, fixed-rate loop |
| `gdk_training/viewer.py` | MJPEG server on port 8000, clicks on the image |
| `gdk_training/geometry.py` | poses as 4x4 matrices, straight-line interpolation |
| `gdk_training/jog.py` | jog keys for the live displays |
| `gdk_training/joints.py` | joint names and limits |
| `gdk_training/lidar.py` | point clouds as arrays, the finished code of exercise 11 |
| `gdk_training/serial.py` | one call at a time to a GDK object, for the threads of the live displays |
| `gdk_training/drive.py` | drive keys for the live displays, the finished code of exercise 12 |
| `gdk_training/maps.py` | maps as images and arrays, the finished code of exercises 14 and 15 |
| `gdk_training/cloud3d.py` | 3D point cloud view in the browser, three.js in `gdk_training/static` |
| `fake/agibot_gdk/` | stand-in for the GDK, the navigation part in `_nav.py` and `data/` |
| `scripts/install_gdk.sh` | installs the GDK package for laptops from the robot |
| `real.sh`, `robot.sh`, `fake.sh` | environment for the laptop with the robot, for the robot itself and for the fake |
| `gdk_training/doctor.py` | connection check for the laptop setup |
| `SETUP.md` | architecture and setup of the laptop |

## License

[Apache License 2.0](LICENSE). The AGIBOT GDK is not part of this repository. `fake/agibot_gdk` is an
independent stand-in that contains no vendor code. `fake/agibot_gdk/data` holds a map and sensor
values recorded on a G2. `gdk_training/static` contains three.js r160 under the MIT license
(`LICENSE-three.txt`).
