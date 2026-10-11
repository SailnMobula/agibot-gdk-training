# 11. Lidar

The chassis has a 3D lidar at the front and one at the back. This exercise reads both point
clouds, decodes them from raw bytes, moves them into `base_link` and shows them in 3D in the
browser.

## Write

- `info` calls `get_lidar_fps`, `get_lidar_latency` and `get_latest_pointcloud` and returns a `LidarInfo`.
- `decode` turns the bytes of a cloud into an array with one row x, y, z per point.
- `sensor_to_base` reads the mounting of a lidar with `get_tf_from_sensor`.
- `to_base` applies a 4x4 matrix to the points.
- `height_band` keeps the points between two heights.

## Returns

`agibot_gdk.Lidar()` starts both lidars. `Lidar([agibot_gdk.LidarType.kLidarFront])` starts only
the ones in the list. `LidarType` has `kLidarFront` and `kLidarBack`.

| Call | Returns |
| --- | --- |
| `get_latest_pointcloud(lidar_type, timeout_ms)` | point cloud, see below |
| `get_lidar_fps(lidar_type)` | float |
| `get_lidar_latency(lidar_type, window_s)` | object with `avg_latency_ms`, `max_latency_ms` and `p99_latency_ms` |
| `close_lidar()` | `GDKRes` |

The point cloud has `width`, `height`, `point_step`, `row_step`, `is_bigendian`, `is_dense`,
`timestamp_ns`, `fields`, `data` and `data_size`. `data` is a numpy array of bytes, one point
every `point_step` bytes. Each entry of `fields` has `name`, `offset` in bytes within a point,
`datatype` and `count`. The datatype numbers are 1 int8, 2 uint8, 3 int16, 4 uint16, 5 int32,
6 uint32, 7 float32 and 8 float64.

`get_tf_from_sensor(agibot_gdk.SensorExtrinsicType.kChassisFrontLidarToBaseLink)` and
`kChassisBackLidarToBaseLink` return a `Transform`. `geometry.from_transform` converts it.

`np.dtype({"names": [...], "formats": [...], "offsets": [...], "itemsize": point_step})` describes
a point with fields at given offsets. `np.frombuffer(data, dtype)` reads all points at once.

## Try

- Open `http://localhost:8000`. Turn the view with the mouse. Walk around the robot and find yourself in both clouds.
- Colour by height. Find the floor, the ceiling and the tables.
- Switch on the height band of the page and change it. Use the view from above.
- Press `b` for your own `height_band` with `BAND`.
- Hold a box in front of the robot at different heights.

## Look for

- Note which fields a point has and how many bytes it takes. Why does `reshape` not work?
- Find out in which direction the transform maps a point. Transform the origin of the sensor and check where it lands. The front lidar has to end up at positive x.
- Both lidars see the same wall at the same place once both transforms are right.
- The lidar also sees the robot itself. Where do these points lie?
- A height band through the cloud seen from above gives a 2D scan. Exercise 14 lays it on the map.
- Compare the rate and the latency of both lidars.

## Stretch

- Write `reflectivity` and colour the points by value. Find shiny and dark surfaces.
