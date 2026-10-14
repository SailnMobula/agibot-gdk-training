# 15. Map in 3D

While mapping, the robot records a 3D point cloud of the room. The 2D grid is made from it. This
exercise shows the cloud in 3D, transforms it from the front lidar frame into the map frame and
builds an own grid from it to compare with the robot's.

## Write

- `map_cloud` decodes the point cloud of the map with your `decode` from exercise 11.
- `to_map` moves the points into the map frame.
- `to_grid` marks every cell of the grid that a point in a height band falls into.

The provided `agreement` compares your grid with the robot's. The table shows its numbers:
"in occupied", "in free" and "in unknown" are the shares of your cells that the robot marks
occupied, free and unknown, "robot found" is the share of the robot's occupied cells that are
also yours.

## Returns

`get_map(map_id)` returns the map. `nonground_cloud` is a point cloud like the ones of the lidar
in exercise 11, without the floor. The point cloud carries no frame. The robot stores it in the
frame of the front lidar at the moment the mapping started, and the map frame is `base_link` at
that moment. So the extrinsic of the front lidar from exercise 11 transforms the cloud into the
map frame.

`decode`, `sensor_to_base` and `to_base` from exercise 11 are in `gdk_training.lidar`,
`geometry_of` from exercise 14 in `gdk_training.maps`. `maps.cells(info)` gives the robot's grid
as an array, row 0 at the origin.

## Try

- Open `http://localhost:8001`. Turn the cloud and look at it from the side.
- Press `l` to switch between the cloud as stored and in the map frame. Watch the lowest z in the table.
- Open `http://localhost:8000` and compare your grid with the robot's. The legend explains the
  colours. Move the band with `+` and `-`.

## Look for

- The point cloud of the map has a different layout than the lidar clouds. Does your `decode` still work?
- Where is the floor in the stored cloud, and where in the map frame? Compare with the height of the front lidar.
- Compare the agreement of both clouds. How much of the robot's grid do you find?
- Which band gives the grid most similar to the robot's?
- Which obstacles are in your grid but not in the robot's? The planner cannot see them.
- `MapInfo` also has `ground_cloud`. What is in it?
- The table shows the tilt of `gravity`, the direction of gravity measured while mapping. Which
  part of the robot is tilted by about as much?
