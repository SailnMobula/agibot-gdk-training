# 12. Drive

`move_chassis` drives the base with a velocity command. The robot only accepts it after
`request_chassis_control`. The script sends the command of the pressed key ten times per second
and stops the robot when no key is pressed.

## Write

- `take_control` cancels a task that is still in flight, waits until it has ended and calls `request_chassis_control(0)`.
- `twist` builds a `Twist` from a `Command`.
- `send` passes the twist to `move_chassis`.
- `deadman` returns the command while the key is fresh and `STOP` after `hold_s`.
- `release` stops the robot and cancels the task that holds the chassis.

## Returns

`agibot_gdk.Pnc()` creates the navigation client. It needs 2 s after construction.

`agibot_gdk.Twist()` has `linear` and `angular`, each a `Vector3` with `x`, `y` and `z`.
`linear.x` is forward in m/s and `angular.z` is the turn rate in rad/s, counter-clockwise
positive. `Vector3` takes no constructor arguments: set the fields of `twist.linear` and
`twist.angular`.

| Call | Returns |
| --- | --- |
| `request_chassis_control(mode)` | nothing, raises `RuntimeError` on failure |
| `move_chassis(twist)` | nothing, raises `RuntimeError` on failure |
| `get_task_state()` | object with `id`, `state`, `type` and `message` |
| `cancel_task(task_id)` | nothing |

Mode 0 is Ackermann. `type` 3 means remote control. `state` is 0 idle, 1 starting, 2 running,
3 pausing, 4 paused, 5 resuming, 6 canceling, 7 canceled, 8 fail and 9 success.

`cancel_task` ends a task in state 1, 2, 4 or 5. Do not look at the type before you cancel: the
planner takes nothing new while any task is in flight, also one of the HMI or of navigation.

Only one `Pnc` call may be under way at a time. A second one from another thread breaks the first
("Broken promise"). The provided code holds a lock around the sender's `move_chassis` and the
table's `get_task_state`.

## Try

- Drive with `w`, `s`, `a` and `d`. Release the key and watch the robot stop.
- Watch the task in the table while you drive and after you quit.
- Stop the script with Ctrl+C and start it again.

Someone stands at the emergency stop. The provided code limits the speed to 0.3 m/s and
0.5 rad/s.

### A leftover remote control task, with the fake

On the robot a remote control task often keeps running after the HMI drove the robot or after a
script crashed. The fake starts fresh with every run, so it never has one left over. Start the
script like this to get one:

```bash
FAKE_REMOTE_CONTROL=1 python3 exercises/12_drive/drive.py
```

The fake then starts with a running remote control task. Your code does not change. When
`take_control` only calls `request_chassis_control(0)`, the script stops with
`RuntimeError: RequestChassisControl failed` and the log says the task is not idle. When it
cancels the old task first, the table shows a new task id and you can drive.

## Look for

- `request_chassis_control` fails while any task is in flight, also your own from the last run.
- The robot stops on its own about 0.6 s after the last command. A zero command stops it at once.
- While you hold the chassis, a remote control task runs. Navigation cannot run at the same time.
- Holding a key repeats it only after a short delay. `hold_s` has to bridge that delay.
