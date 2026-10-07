# 5. Gripper

The OmniPicker has one joint. Its position is -0.785 rad when open and 0 rad when closed. The
script shows the state of both grippers and binds commands to keys.

## Write

- `read_gripper` reads `get_end_state` and returns one `Gripper` per side.
- `command` sends a `JointStates` with group `dual_tool` through `move_ee_pos`.

## Returns

`get_end_state()` returns a dict with the keys `left_end_state` and `right_end_state`. Each of
them is a dict with `controlled`, `type`, `names` and `end_states`. `end_states` is a list with
one dict per gripper joint, and the OmniPicker has one joint. That dict has the keys `position`,
`velocity`, `current`, `temperature`, `effort`, `voltage`, `enable`, `status` and `err_code`.

`agibot_gdk.JointStates()` creates an empty command with these attributes.

| Attribute | Content |
| --- | --- |
| `group` | `"dual_tool"`, `"left_tool"` or `"right_tool"` |
| `target_type` | `"omnipicker"` |
| `states` | list of `agibot_gdk.JointState()`, each with its `position` set |
| `nums` | number of entries in `states` |

`move_ee_pos(command)` returns 0.

## Try

- `o` opens, `c` closes and `h` moves to the middle.
- `l` closes the left gripper and `r` closes the right gripper. The other gripper keeps its position.
- Close on a soft object and watch position and current.

## Look for

- The gripper state is part of `get_end_state`.
- `move_ee_pos` returns 0 even when the gripper does not move. Confirm every command by reading the position.
- Note the position at which the gripper stops when an object is between the fingers.
- The groups `left_tool` and `right_tool` address one gripper. A new command to a group cancels the previous command to that group.
