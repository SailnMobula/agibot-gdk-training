# 10. Camera and TF

This exercise draws the position of both end effectors, as the robot computes them, into the
image of the head camera.

## Write

- `camera_from_base` combines `lookup_transform_latest` for `head_link3` with `get_tf_from_sensor` for the camera mounted on it.
- `project` applies the pinhole model with the camera intrinsics.

## Returns

`lookup_transform_latest` returns a `Transform`, as in exercise 8.

`get_tf_from_sensor(agibot_gdk.SensorExtrinsicType.kHeadRGBDToHeadLink3)` returns a `Transform`.
It maps a point given in the optical frame of the head camera into `head_link3`.

`geometry.from_transform` converts a `Transform` to a 4x4 matrix and `geometry.inverse` inverts it.

## Try

- Open `http://localhost:8000` in the browser of the PC that runs the script, with both arms in view of the head camera.
- Jog the head with `s` and `w`. Then select an arm joint with `d` and jog it.

## Look for

- The circles stay on the end effectors while head and arms move.
- If a circle moves in the wrong direction when the head turns, one of the two transforms is inverted.
- Measure the remaining offset in pixels and convert it to millimetres at that distance.
- In the optical frame z points forward, x to the right and y down.
