# 8. TF

The TF tree contains every frame of the robot. The table in this exercise shows the position
of a few frames from four sources next to each other. This makes the direction of a lookup
visible.

## Write

- `frame_names` calls `get_all_frame_names`.
- `lookup` calls `lookup_transform_latest`.
- `from_base` calls `get_tf_from_base_link`.

## Returns

`agibot_gdk.TF()` creates the TF client. It needs 2 s after construction, like `Robot`.

| Call | Returns |
| --- | --- |
| `get_all_frame_names()` | list of str |
| `lookup_transform_latest(target, source)` | tuple of a `Transform` and a timestamp, which is `None` unless requested |
| `get_tf_from_base_link(frame)` | `Transform` |

A `Transform` has `translation.x/y/z` in m and `rotation.x/y/z/w`.

## Try

- Run with `--all` to print the list of frames.
- Jog the head with `s` and `w`. Then select an arm joint with `d` and jog it.
- Add frames from the list to `FRAMES`.

## Look for

- The status column is the pose of the frame in `base_link`. Find the TF column that agrees with it.
- Conclude from this which way `lookup_transform_latest(target, source)` maps a point.
- The transform from the end effector to the gripper centre stays constant while the arm moves.
- Note the frames near the cameras. Exercise 10 needs one of them.
