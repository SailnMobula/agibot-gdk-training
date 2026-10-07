# 9. Camera

This exercise shows head colour, head depth and both wrist cameras in the browser.

## Write

- `info` calls `get_image_shape`, `get_image_fps` and `get_camera_intrinsic` and returns a `CameraInfo`.
- `grab` calls `get_latest_image` and decodes the data according to `encoding` and `color_format`.

## Returns

| Call | Returns |
| --- | --- |
| `get_image_shape(camera_type)` | tuple `(width, height)` |
| `get_image_fps(camera_type)` | float |
| `get_camera_intrinsic(camera_type)` | object with `intrinsic`, a list `[fx, fy, cx, cy]`, and `distortion` |
| `get_latest_image(camera_type, timeout_ms)` | image object, see below |

The image object has `width`, `height`, `timestamp_ns`, `encoding`, `color_format` and `data`.
`encoding` is a value of `agibot_gdk.Encoding`, which has `UNCOMPRESSED`, `JPEG` and `PNG`.
`color_format` is a value of `agibot_gdk.ColorFormat`, which includes `RGB`, `BGR` and
`RS2_FORMAT_Z16`. `data` holds the bytes of the image. `np.frombuffer(image.data, dtype)` turns
them into an array.

## Try

- Open `http://localhost:8000` in the browser of the machine that runs the script. `Ctrl+C` ends the script.
- Hold a hand in front of the head camera and watch the depth stream.
- Hold a sticky note in front of the left wrist camera and check that it appears in `hand_left`.

## Look for

- Note which cameras deliver JPEG and which deliver raw data.
- Depth has 16 bit per pixel and is given in millimetres.
- `Camera([...])` starts only the cameras in the list.
- The first frames arrive about 3 s after the constructor. Before that `get_latest_image` returns no image.
