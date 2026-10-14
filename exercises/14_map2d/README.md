# 14. Map in 2D

A map is an occupancy grid plus guide points from the HMI. This exercise draws both together
with the robot pose.

## Write

- `geometry_of` reads size, resolution and origin of the grid and returns a `MapGeometry`.
- `world_to_pixel` converts a point on the map in metres to a pixel of the image.
- `pixel_to_world` converts back.
- `grid_image` colours the cells: unknown grey, free white, occupied black.
- `guide_points` returns the guide points of the map.
- `robot_pose` reads `get_curr_pose` and returns x, y and yaw.

## Returns

`get_map(map_id)` returns the map. Its attributes include `id`, `grid_map` and `guide_pts`.

`grid_map` has `width` and `height` in cells, `resolution` in m per cell, `origin` as a `Pose`
and `data`, a list with one value per cell, row by row, starting at the origin. The values are
-1 unknown, 0 free and 100 occupied.

`guide_pts` is a list of guide points. Each has `id`, `type` and `pt`, a `Pose` on the map.

`get_curr_pose()` returns a `Pose`. `geometry.from_pose` and `geometry.rpy` give the yaw.

## Try

- Open `http://localhost:8000`.
- Drive with the keys from exercise 12 and watch the arrow. Does it point the way the robot drives?
- Add a guide point on the HMI. Press `r` and watch it appear.
- Click on a guide point. The table shows the point in the map frame, computed with your
  `pixel_to_world`, and a red cross marks it, drawn with your `world_to_pixel`. Compare it with
  the guide point in the table.
- Press `l` for the live lidar scan, drawn at the pose of the robot. Drive and watch it.
- Run the script with `--map ID` for the map from exercise 13.

## Look for

- Row 0 of the grid is at the origin, the bottom of the map. Image row 0 is at the top.
- Note where the origin of the map frame is (the magenta cross) and where the mapping started.
- Guide points belong to the map and every client sees them. Waypoints in your own program do not.
- Map names, `counter` and `timestamp_ns` are empty or 0. A changed map can only be detected by comparing its content.
- `info.walls` and `info.infeasible_areas` raise an error. The HMI has them and the planner uses them, but the GDK cannot hand them to Python.
- The GDK reports a pose but no measure of how good it is. When the robot is well localized, the
  scan lies on the walls. What lies off the walls, and why?
- Without a correct localization the robot cannot navigate. Relocalize on the HMI. The GDK has
  `Slam.global_local`, but AgiBot supports it only at the pose where the mapping started.

### A badly localized robot, with the fake

The fake can play a robot that is badly localized. Start the script like this:

```bash
FAKE_POSE_ERROR=0.3,0,10 python3 exercises/14_map2d/map2d.py
```

The robot stays where it is, but `get_curr_pose` reports a pose 0.3 m in x and 10° off
(`dx,dy,dyaw` in m, m and degrees). Your code does not change: the arrow is drawn at the reported
pose, and so is the scan. Press `l`. The scan comes from the true position, so it no longer lies
on the walls but next to them, shifted and turned. Compare it with a run without the variable.
On the robot this is the picture that tells you to relocalize on the HMI.

## Stretch

- `changed` compares two maps by grid and guide points. With it the script reloads by itself every 2 s.
