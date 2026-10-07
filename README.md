# GDK training

Exercises for the AGIBOT G2 on GDK 3.3.8. They cover reading the robot state, moving the robot,
coordinate frames and cameras. Each exercise is one script with a few functions left open. These
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

Each folder in `exercises/` has a `README.md` with the task. Open functions are marked
`TODO` and raise `NotImplementedError` until you write them. The docstring of each
function states what it has to return, and the small classes at the top of a script name the
fields and their units. The Returns section of each
README lists the fields of the GDK calls you need. The reference implementation is in
`solutions/`.

## On a laptop with the robot

The scripts run on your laptop and talk to the robot over the network. This works over the
Debug cable and over Wi-Fi. The laptop needs Linux on x86_64, because the GDK package for
laptops is built for that platform and for Python 3.10.

```bash
scripts/install_gdk.sh <ROBOT_IP>     # once
uv sync                               # once
source real.sh <ROBOT_IP>             # in every new shell
python -m gdk_training.doctor
python exercises/01_joints/joints.py
```

`scripts/install_gdk.sh` downloads the GDK from the robot into `~/.cache/agibot/app`.
`uv sync` creates a Python 3.10 environment in `.venv`. `real.sh` sets the environment for the
GDK and activates `.venv`. The doctor checks the connection and only reads from the robot.
Over the Debug cable the robot has the address `10.42.1.101`, and `<ROBOT_IP>` can be left out.

[SETUP.md](SETUP.md) explains the architecture, the difference between cable and Wi-Fi, the
firewall rule that commands need and the troubleshooting.

The robot serves the GDK 3.3.8 reference as a website at `http://<ROBOT_IP>:8849`. The Python
pages `robot`, `tf` and `camera` describe every call the exercises use.

`q` ends a script with a live table. `Ctrl+C` ends every script. Run one script at a time. The
live displays have jog keys, so you can move a joint while you watch the values. The camera
exercises serve their images at `http://localhost:8000`.

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
and the head camera shows a grid with a cross on each end effector. Use it to get a function
working before the robot is free.

`uv run pytest` runs every solution against the fake. It takes about a minute.

## Layout

| Path | Content |
| --- | --- |
| `exercises/` | scripts with open functions and the task READMEs |
| `solutions/` | the same scripts, complete |
| `gdk_training/show.py` | live tables, key bindings, fixed-rate loop |
| `gdk_training/viewer.py` | MJPEG server on port 8000 |
| `gdk_training/geometry.py` | poses as 4x4 matrices, straight-line interpolation |
| `gdk_training/jog.py` | jog keys for the live displays |
| `gdk_training/joints.py` | joint names and limits |
| `fake/agibot_gdk/` | stand-in for the GDK |
| `scripts/install_gdk.sh` | installs the GDK package for laptops from the robot |
| `real.sh`, `robot.sh`, `fake.sh` | environment for the laptop with the robot, for the robot itself and for the fake |
| `gdk_training/doctor.py` | connection check for the laptop setup |
| `SETUP.md` | architecture and setup of the laptop |

## License

[Apache License 2.0](LICENSE). The AGIBOT GDK is not part of this repository. `fake/agibot_gdk` is an
independent stand-in that contains no vendor code.
