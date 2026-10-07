# 6. Servo

`joint_servo_control` takes one setpoint and returns immediately. The robot does not plan a
path. It follows the setpoints in the order and at the rate they arrive. In this exercise one
joint follows a sine for eight seconds.

## Write

- `setpoint` computes the sine.
- `send` sends a `JointServoControlReq` through `joint_servo_control`.

## Returns

`agibot_gdk.JointServoControlReq()` creates an empty request with these attributes.

| Attribute | Content |
| --- | --- |
| `control_period` | float in s, slightly longer than the time between two requests |
| `joint_names` | list of joint names |
| `joint_positions` | list of setpoints in rad, same order |

`joint_servo_control(request, enable_low_latency=False)` returns 0 on success.

## Try

- Run the default joint first, then a head joint. `Ctrl+C` ends the script early.
- Set `RATE_HZ` to 10 and `CONTROL_PERIOD` to 0.1.
- Set `AMPLITUDE` to 0.05.

## Look for

- The error column shows how far the joint lags behind the target.
- Compare the motion at 10 Hz with the motion at 30 Hz, both visually and by sound.
- The first setpoint equals the measured position. A first setpoint far from the measured position would command a jump.
- `enable_low_latency=True` disables the collision protection. Leave it off.
- Exercise 4 sent a goal and waited for the result. With the servo call your code defines every point of the path and its timing.
