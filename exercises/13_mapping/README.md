# 13. Mapping

The robot builds a map while you drive it through the room. This exercise starts and stops the
mapping, finds the new map in the map list and switches between maps.

## Write

- `get_maps` reads `get_all_map` and returns one `MapEntry` per map.
- `SLAM_STATES` names the codes of `get_slam_state`. Fill it in while you try the keys.
- `slam_state` reads `get_slam_state` and returns the code and its name from `SLAM_STATES`.
- `start` calls `start_mapping`.
- `stop` calls `stop_mapping` and returns the new map.
- `cancel` calls `cancel_mapping`.
- `switch` calls `switch_map`.

## Returns

`agibot_gdk.Slam()` and `agibot_gdk.Map()` need 2 s after construction.

| Call | Returns |
| --- | --- |
| `start_mapping()` | nothing, raises `RuntimeError` on failure |
| `stop_mapping()` | nothing. It saves the map |
| `cancel_mapping()` | nothing. It discards the map |
| `get_slam_state()` | int |
| `get_all_map()` | list of objects with `id`, `name` and `is_curr_map` |
| `get_curr_map()` | one such object |
| `switch_map(map_id)` | nothing, raises `RuntimeError` on failure |

Map ids are ints from 0 to 255.

## Try

- Note the id of the current map. Press `1` and drive through the room with the drive keys. Press `2`.
- Start a mapping and press `3`.
- Press `4` and type the id of your new map, then press `4` and Enter to switch back to the
  workshop map. Relocalize the robot on the HMI after each switch.
- Open `map_<id>.png` next to the script.

Agree on the mapping with the other pair. Only one map can be built at a time, and everyone uses
the same robot.

## Look for

- Note the SLAM state before, during and after the mapping. Fill in `SLAM_STATES`.
- `stop_mapping` returns nothing. Compare the map list before and after.
- The new map becomes the current map about 1 s later. It has no virtual walls, no-go zones or guide points.
- After mapping and after switching, the robot has to be relocalized before it can navigate.
- Map names are empty. Maps are known only by their id.
- Glass, mirrors and long empty corridors are hard for the lidar. Note where the map is wrong.
