# 2. Status

This exercise reads the state of the robot as a system. It covers the control mode, error
fields, emergency stops and the batteries.

## Write

- `control_state` reads `get_motion_control_status` and returns a `ControlState`.
- `body_state` reads `get_whole_body_status`.
- `batteries` reads `get_chassis_power_state` and returns one `Battery` per battery.

## Returns

`get_motion_control_status()` returns an object with the attributes `mode`, `control_mode`,
`error_code` and `error_msg`. Exercise 3 uses its other attributes.

`get_whole_body_status()` returns a dict. Its keys are `timestamp`, one `<part>_error` per body
part, `left_arm_control`, `right_arm_control`, `left_arm_estop`, `right_arm_estop`,
`left_end_model` and `right_end_model`.

`get_chassis_power_state()` returns an object. Its attribute `battery_states` is a list with one
object per battery. Each battery has `battery_soc`, `battery_output_voltage`,
`battery_output_current` and `battery_temperature`.

## Try

- Ask the trainer to press and release the soft emergency stop. Note which fields change.
- Read `control_mode` before anyone moves the robot today.

## Look for

- `control_mode` is 1 for joint position control, 2 for Cartesian impedance and 3 for joint impedance.
- The control mode is stored on the robot. A new process finds the mode that the previous client set.
- `mode` is 0 for stop, 1 for servo and 2 for planning.
- The whole-body status has one error field per body part. A value of zero means no error.
