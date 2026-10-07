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
README lists the fields of the GDK calls you need. The Python pages `robot`, `tf` and `camera`
of the GDK 3.3.8 reference describe every call in full. The reference implementation is in
`solutions/`.

## With the robot

The scripts run on your PC and talk to the robot over the network cable. The GDK calls this
hybrid deployment. It needs Ubuntu 22.04 on x86_64.

1. Connect the PC to the Debug port of the robot with an Ethernet cable. Give the network
   interface of the PC the static address `10.42.1.102` with netmask `255.255.255.0`. On this
   network the robot has the fixed address `10.42.1.101`. A second PC on the same network needs
   another free address, for example `10.42.1.103`.

   ```bash
   ping 10.42.1.101
   ```

2. Install the GDK on the PC. The installer puts it into `~/.cache/agibot/app`.

   ```bash
   curl -sSL http://10.42.1.101:8849/install.sh | bash
   ```

3. Install the two Python packages the exercises need.

   ```bash
   python3 -m pip install numpy opencv-python-headless
   ```

4. Clone this repository and run the first exercise.

   ```bash
   git clone <this repository> && cd gdk-training
   source robot.sh
   python3 exercises/01_joints/joints.py
   ```

The prebuilt GDK is made for Python 3.10, which is the `python3` of Ubuntu 22.04. The exercises
run on Python 3.10 and newer, so use the system `python3`. There is no need to build the GDK for
another Python version. On a system with a newer `python3`, create a Python 3.10 environment and
work in it.

```bash
uv venv --python 3.10 && source .venv/bin/activate
uv pip install numpy opencv-python-headless
```

The GDK reaches the robot only through the `10.42.1.x` network of the Debug port. The address
the robot has in the Wi-Fi network is useful for `ssh`, but the GDK on a PC cannot use it.
`robot.sh` prints a warning when the PC has no `10.42.1.x` address or when `python3` does not
match the Python version of the GDK.

`robot.sh` sources the `env.sh` of the GDK, which puts `agibot_gdk` on the Python path, and adds
this folder. Source it once per shell, from the folder of this repository. `q` ends a script with
a live table. `Ctrl+C` ends every script. Run one script at a time. The live displays have jog
keys, so you can move a joint while you watch the values.

The camera exercises serve the images on port 8000 of the PC. Open `http://localhost:8000` in
the browser of the same PC.

The same steps work directly on the robot. There the GDK is already installed in
`/home/agi/app`, and `robot.sh` finds it.

## Without the robot

```bash
uv sync
source fake.sh
python3 exercises/01_joints/joints.py
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

## License

[Apache License 2.0](LICENSE). The AGIBOT GDK is not part of this repository. `fake/agibot_gdk` is an
independent stand-in that contains no vendor code.
