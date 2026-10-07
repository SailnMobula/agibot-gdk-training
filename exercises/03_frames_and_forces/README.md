# 3. Frames and forces

The motion control status also reports the pose of selected links and the force and torque
acting on them. It reports an estimated external torque for each arm joint as well.

## Write

- `frames` combines `frame_names`, `frame_poses` and `wrenches` from `get_motion_control_status` into one `Frame` per frame.
- `disturbance` returns one `JointTorque` per value of `arm_disturbance_force`. It has 14 values and the left arm comes first.

## Returns

`get_motion_control_status()` returns an object with these attributes.

| Attribute | Content |
| --- | --- |
| `frame_names` | list of frame names |
| `frame_poses` | list of `Pose`, one per frame, with `position.x/y/z` and `orientation.x/y/z/w` |
| `wrenches` | list of `Wrench`, one per frame, with `force.x/y/z` and `torque.x/y/z` |
| `twists` | list of `Twist`, one per frame, with `linear.x/y/z` and `angular.x/y/z` |
| `arm_disturbance_force` | list of 14 floats, one per arm joint |

The lists `frame_names`, `frame_poses` and `wrenches` have the same order.

## Try

- Ask the trainer to pull the left gripper down, then sideways.
- Hang an object of known weight on a gripper.
- Jog an arm joint with `s` and `w` and follow the position of the end effector.

## Look for

- All poses are given in `base_link`, in metres.
- `arm_l_end_link` and `arm_r_end_link` are the end-effector frames of the two arms.
- Find the axis of the wrench that responds to a pull straight down.
- Compare the joint torques during the same pull. Some joints react strongly and others hardly at all.
