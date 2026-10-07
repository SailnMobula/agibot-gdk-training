# 7. End-effector pose

`end_effector_pose_control` takes a pose of the end effector in `base_link`. The robot solves
the joint angles itself. Like the servo call it expects a stream, here at 50 Hz, and each pose
has to be close to the previous one. The script divides a straight line into steps of 2 mm.

## Write

- `end_effector` returns the pose of `arm_l_end_link` or `arm_r_end_link` from `get_motion_control_status`.
- `send` sends an `EndEffectorPose` through `end_effector_pose_control`.

## Returns

The frame poses come from `get_motion_control_status()`, as in exercise 3.

`agibot_gdk.EndEffectorPose()` creates an empty request with these attributes.

| Attribute | Content |
| --- | --- |
| `life_time` | float in s |
| `group` | int, taken from `agibot_gdk.EndEffectorControlGroup`, for example `kLeftArm` or `kRightArm` |
| `left_end_effector_pose` | `Pose` in `base_link` |
| `right_end_effector_pose` | `Pose` in `base_link` |

`end_effector_pose_control(request)` returns 0 on success.

## Try

- Run the script with `left 0 0 0.05`, then with `left 0 0 -0.05`.
- Run it with `right 0.05 0 0`.
- Run it with `left 0 0 0.05 --grip`, which commands the gripper directly after the move.
- `Ctrl+C` ends the script early.

## Look for

- The commanded pose refers to the end-effector frame `arm_l_end_link` or `arm_r_end_link`. The gripper centre is a separate frame further along the tool axis.
- Every request contains a pose for both arms. Fill the arm that should stay in place with its current pose.
- The script prints the remaining distance to the goal in mm.
- With `--grip` the script prints how long the gripper takes to react after the stream ends. `move_ee_pos` returns 0 during that time, so the return value does not tell you whether the command was executed.
- This interface does no planning and no collision checking. Keep the moves short and stay away from a fully stretched arm.
- Read the group from `EndEffectorControlGroup` by name. The enum has further groups in which the robot also moves the waist to reach the pose.
