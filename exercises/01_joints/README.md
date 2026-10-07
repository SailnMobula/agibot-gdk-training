# 1. Joints

The first exercise connects to the robot and shows position, velocity and effort of every
joint. The table refreshes five times per second.

## Write

- `connect` initialises the GDK, creates the `Robot` and waits 2 s for the DDS connection.
- `read_joints` calls `get_joint_states` and returns one `Joint` per joint.

## Returns

`get_joint_states()` returns a dict.

| Key | Content |
| --- | --- |
| `timestamp` | time of the sample in ns |
| `nums` | number of joints |
| `states` | list with one dict per joint |

Each entry of `states` has the keys `name`, `motor_position`, `motor_velocity`, `effort`,
`motor_current`, `error_code`, `mode`, `position` and `velocity`.

`gdk_init()` and `gdk_release()` return a `GDKRes`. Compare it with `agibot_gdk.GDKRes.kSuccess`.

## Try

- Select a joint with `a` and `d`. Step it by -0.1 or +0.1 rad with `s` and `w`. Watch the position and velocity columns during the move.
- Ask the trainer to press down on a wrist and watch the effort column.

## Look for

- The robot has 22 joints. The waist uses `idx01` to `idx05`, the head `idx11` to `idx13`, the left arm `idx21` to `idx27` and the right arm `idx61` to `idx67`.
- The measured values are in `motor_position` and `motor_velocity`. The fields `position` and `velocity` are reserved and carry no usable data.
- Positions are in rad and velocities in rad/s.
- A resting arm shows an effort different from zero, because its joints hold the arm against gravity.
- Check whether the gripper joints appear in this list. Exercise 5 reads them through a different call.
