# 16. Navigate

`normal_navi` drives the robot to a pose on the map. The robot plans the path itself and avoids
obstacles. `relative_move` moves the robot by a short distance relative to itself. Both start a
task that runs in the background. The script shows the robot, the goal and the task in a live
table and the map at `http://localhost:8000`, where a click sets a goal.

## Write

- `navi_req` builds a `NaviReq` from x, y and yaw.
- `task` reads `get_task_state` and returns a `Task` with the names of state and type, the last
  line of the message and the number of attempts.
- `go` calls `normal_navi`, your `relative_move` calls the GDK's `relative_move`.
- `pause`, `resume` and `cancel` act on the current task.
- `check_goal` returns the problems of a goal before it is sent. It combines three yes/no checks
  that you write first:
  - `on_map`: does the goal lie inside the grid?
  - `is_free`: is its cell free?
  - `has_clearance`: is no occupied cell within the clearance? `maps.cells_around` gives the
    square of cells around the goal.
- `navigate_to` sends a goal and waits until the task has ended.

## Returns

`agibot_gdk.NaviReq()` has `target`, a `Pose`. For `normal_navi` it is a pose on the map, for
`relative_move` a pose in `base_link`.

| Call | Returns |
| --- | --- |
| `normal_navi(request)` | nothing, raises `RuntimeError` on failure |
| `relative_move(request)` | nothing, raises `RuntimeError` on failure |
| `get_task_state()` | object with `id`, `state`, `type` and `message` |
| `pause_task(task_id)`, `resume_task(task_id)`, `cancel_task(task_id)` | nothing |

`state` is 0 idle, 1 starting, 2 running, 3 pausing, 4 paused, 5 resuming, 6 canceling,
7 canceled, 8 fail and 9 success. `type` is 0 idle, 1 navigation, 3 remote control and
4 relative move.

The map comes from `gdk_training.maps`. `maps.cell_of(info, x, y)` gives row and column of the
cell of a point, or `None` outside the map. `maps.cells(info)` is the grid as an array, row 0 at
the origin. `maps.cells_around(info, x, y, radius)` is the square of grid values around a point.
`CLEARANCE` is 0.4 m.

## Try

Someone stands at the emergency stop. Open `http://localhost:8000`. A goal is set first and
checked at once, the table shows the answers of your checks. A separate key sends it.

| Key | Action |
| --- | --- |
| click on the map | goal there, facing away from the robot |
| `1` to `9` | goal at that guide point |
| `x` | type a goal: x y yaw (m, m, deg) |
| `g` | go: your `go`, returns at once |
| `w` | go and wait: your `navigate_to`, the result appears in the table |
| `p`, `r`, `c` | pause, resume, cancel |
| `m` | relative move with your `relative_move`: dx dy dyaw (m, m, deg), at most 1.5 m |

With the fake or the robot:

- Set a goal and press `g`. Watch the task in the table. Press `p`, `r` and `c`.
- Press `1` for guide point 1, then `w`. Cancel it while `navigate_to` waits.
- Click on a wall and next to a wall. What do your checks say? Send the goal anyway.
- Send a goal while exercise 12 holds the chassis.

On the robot only, after you drew a no-go zone across a path on the HMI:

- Send a goal behind the zone.
- Click inside the zone. What do your checks say? Send it.
- Stand right next to the zone and send a free goal.
- `m` with `1 0 0` towards the zone.

## Look for

- `normal_navi` returns at once. The task tells you when the robot has arrived.
- Note the sequence of states and the messages for success and failure.
- A failed goal takes several seconds. What does the robot do meanwhile? Look at the message.
- `check_goal` accepts the goal inside the zone, the robot rejects it. Why? What does that mean
  for the app?
- "goal occupied" next to the zone: which goal is meant?
- Which of the two calls respects the zone?
- `navigate_to` is the function the web app uses for every waypoint.
