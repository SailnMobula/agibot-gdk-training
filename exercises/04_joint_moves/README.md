# 4. Joint moves

`joint_control_request` moves any set of joints to target angles. The call returns when the
robot has arrived. The script provides a prompt that takes joint names and angles, so you can
try a configuration with one line of input.

## Write

- `ensure_position_mode` reads the control mode and switches to mode 1 if needed.
- `move` sends a `JointControlReq` through `joint_control_request`.
- `move_head` uses the dedicated call `move_head_joint`.

## Returns

`get_motion_control_status().control_mode` is an int. `set_control_mode(mode)` takes 1, 2 or 3.

`agibot_gdk.JointControlReq()` creates an empty request with these attributes.

| Attribute | Content |
| --- | --- |
| `joint_names` | list of joint names |
| `joint_positions` | list of targets in rad, same order |
| `joint_velocities` | list of speeds in rad/s, same order |
| `life_time` | float in s |

`joint_control_request(request)` returns 0 on success and raises `RuntimeError` when the robot
rejects the request. `move_head_joint(positions, velocities)` takes two lists of three values in
the order yaw, pitch, roll.

## Try

- Enter `idx13_head_joint3 0.2`, then `head 0 0 0`.
- Move one wrist joint of each arm with a single line.
- Enter `save a`, move to another configuration and return with `go a`.
- Enter `speed 0.05` and repeat a move.
- Send a target outside the limit of a joint.
- Send two joints with very different distances in one request.

## Look for

- The call blocks for the whole duration of the move.
- The joints of one request start at the same time. Observe whether they also arrive at the same time.
- Read the error the robot returns for a target outside the limits.
- Waist joints shift the centre of gravity. The robot rejects a target that would bring it out of balance.
- `move_waist_joint` and `move_arm_joint` work like `move_head_joint` for their joint group.

Keep the speed at 0.2 rad/s unless the trainer tells you otherwise. The prompt accepts at most 0.5 rad/s.
