"""Checks the solutions of exercises 11 to 16 against the navigation part of the fake GDK."""

from __future__ import annotations

import math
import time

import numpy as np
import pytest
from test_solutions import ENV, ROOT, load, run  # noqa: F401  (also puts the fake on sys.path)

import agibot_gdk
from agibot_gdk import _nav
from gdk_training import geometry, lidar, maps, show

FREE_GOAL = (0.10, 0.76)  # guide point 2


def wall_goal() -> tuple[float, float]:
    """The centre of an occupied cell of the recorded map."""
    info = maps.current_map(agibot_gdk.Map())
    row, column = np.argwhere(maps.cells(info) == 100)[100]
    geo = maps.geometry_of(info)
    return geo.origin_x + (column + 0.5) * geo.resolution, geo.origin_y + (row + 0.5) * geo.resolution


@pytest.fixture(autouse=True)
def fresh_world(monkeypatch: pytest.MonkeyPatch) -> _nav.NavWorld:
    """Every test starts with a new fake world at the recorded start pose."""
    monkeypatch.delenv("FAKE_INSTANT", raising=False)
    monkeypatch.delenv("FAKE_POSE_ERROR", raising=False)
    monkeypatch.setattr(_nav, "world", _nav.NavWorld())
    return _nav.world


@pytest.fixture
def instant(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("FAKE_INSTANT", "1")


def on_map(points_base: np.ndarray, world: _nav.NavWorld) -> np.ndarray:
    return lidar.to_base(points_base, world.map_from_base())


def occupied_share(points_map: np.ndarray, world: _nav.NavWorld) -> float:
    """The share of points within one cell of an occupied cell of the fake world. Points off the map count as misses."""
    data = world.world_map
    near = np.zeros_like(data.grid, bool)
    occupied = data.grid == 100
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            near |= np.roll(occupied, (dr, dc), axis=(0, 1))
    rows = np.floor((points_map[:, 1] - data.origin[1]) / data.resolution).astype(int)
    columns = np.floor((points_map[:, 0] - data.origin[0]) / data.resolution).astype(int)
    inside = (rows >= 0) & (rows < near.shape[0]) & (columns >= 0) & (columns < near.shape[1])
    return float(near[rows[inside], columns[inside]].sum() / len(points_map))


# --- 11 lidar -------------------------------------------------------------------------------


def test_decode_reads_the_points_of_the_cloud() -> None:
    exercise = load("11_lidar/lidar.py")
    cloud = agibot_gdk.Lidar().get_latest_pointcloud(agibot_gdk.LidarType.kLidarFront, 100)
    points = exercise.decode(cloud)
    assert points.shape == (cloud.width, 3)
    assert np.allclose(points[:, 2], lidar.field_array(cloud, "z"))
    assert np.allclose(exercise.reflectivity(cloud), lidar.field_array(cloud, "reflectivity"))


def test_front_lidar_is_in_front_and_both_lidars_see_the_walls(fresh_world: _nav.NavWorld) -> None:
    exercise = load("11_lidar/lidar.py")
    sensors, tf = agibot_gdk.Lidar(), agibot_gdk.TF()
    assert exercise.sensor_to_base(tf, "front")[0, 3] > 0
    assert exercise.sensor_to_base(tf, "back")[0, 3] < 0
    for name, lidar_type in exercise.LIDARS.items():
        points = exercise.to_base(exercise.decode(sensors.get_latest_pointcloud(lidar_type, 100)),
                                  exercise.sensor_to_base(tf, name))
        band = lidar.without_robot(exercise.height_band(points, *exercise.BAND), lidar.MIN_RANGE)
        assert occupied_share(on_map(band, fresh_world), fresh_world) > 0.95


def test_height_band_keeps_the_heights() -> None:
    exercise = load("11_lidar/lidar.py")
    points = lidar.scan_in_base(agibot_gdk.Lidar(), agibot_gdk.TF())
    band = exercise.height_band(points, 0.1, 1.8)
    assert 0 < len(band) < len(points)
    assert band[:, 2].min() >= 0.1 and band[:, 2].max() <= 1.8


# --- 12 drive -------------------------------------------------------------------------------


def test_deadman_stops_after_the_hold_time() -> None:
    drive = load("12_drive/drive.py")
    command = drive.Command(vx=0.2, wz=0.0)
    assert drive.deadman(command, 0.1, 0.6) == command
    assert drive.deadman(command, 0.7, 0.6) == drive.STOP


def test_twist_puts_the_command_into_linear_x_and_angular_z() -> None:
    twist = load("12_drive/drive.py").twist(load("12_drive/drive.py").Command(vx=0.2, wz=-0.3))
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.2, 0.0, -0.3)


def test_take_control_cancels_a_leftover_task_and_release_ends_it(fresh_world: _nav.NavWorld) -> None:
    drive = load("12_drive/drive.py")
    pnc = agibot_gdk.Pnc()
    fresh_world.start_task(_nav.TYPE_REMOTE_CONTROL, state=2)
    with pytest.raises(RuntimeError):
        pnc.request_chassis_control(0)
    drive.take_control(pnc)
    task = pnc.get_task_state()
    assert (task.type, task.state) == (3, 2)
    start = (fresh_world.x, fresh_world.y)
    drive.send(pnc, drive.Command(vx=0.3, wz=0.0))
    time.sleep(0.3)
    pnc.get_task_state()  # the fake moves the robot when it is called
    assert math.dist(start, (fresh_world.x, fresh_world.y)) > 0.05
    drive.release(pnc)
    time.sleep(0.4)
    assert pnc.get_task_state().state == 7


def test_the_chassis_stops_without_commands(fresh_world: _nav.NavWorld) -> None:
    drive = load("12_drive/drive.py")
    pnc = agibot_gdk.Pnc()
    drive.take_control(pnc)
    drive.send(pnc, drive.Command(vx=0.3, wz=0.0))
    time.sleep(0.8)
    pnc.get_task_state()
    stopped = (fresh_world.x, fresh_world.y)
    time.sleep(0.3)
    pnc.get_task_state()
    assert (fresh_world.x, fresh_world.y) == stopped


# --- 13 mapping -----------------------------------------------------------------------------


def test_stop_returns_the_new_current_map(instant: None, fresh_world: _nav.NavWorld) -> None:
    mapping = load("13_mapping/mapping.py")
    slam, map_api = agibot_gdk.Slam(), agibot_gdk.Map()
    before = mapping.get_maps(map_api)
    mapping.start(slam)
    assert mapping.slam_state(slam).startswith("1")
    new = mapping.stop(slam, map_api, before)
    assert new.id not in {entry.id for entry in before}
    assert new.current and map_api.get_curr_map().id == new.id
    mapping.switch(map_api, fresh_world.world_map.id)
    assert map_api.get_curr_map().id == fresh_world.world_map.id


def test_cancel_keeps_the_map_list() -> None:
    mapping = load("13_mapping/mapping.py")
    slam, map_api = agibot_gdk.Slam(), agibot_gdk.Map()
    before = mapping.get_maps(map_api)
    mapping.start(slam)
    mapping.cancel(slam)
    assert mapping.get_maps(map_api) == before


# --- 14 map in 2D ---------------------------------------------------------------------------


def test_pixels_and_metres_convert_both_ways() -> None:
    map2d = load("14_map2d/map2d.py")
    info = maps.current_map(agibot_gdk.Map())
    geo = map2d.geometry_of(info)
    assert (geo.width, geo.height, geo.origin_x) == (666, 652, -13.25)
    for x, y in [(0.0, 0.0), (2.03, 2.76), (-13.25, -10.3)]:
        assert np.allclose(map2d.pixel_to_world(geo, *map2d.world_to_pixel(geo, x, y)), (x, y))
    u, v = map2d.world_to_pixel(geo, geo.origin_x + 0.01, geo.origin_y + 0.01)
    assert (math.floor(u), math.floor(v)) == (0, geo.height - 1)


def test_grid_image_puts_row_zero_at_the_bottom() -> None:
    map2d = load("14_map2d/map2d.py")
    info = maps.current_map(agibot_gdk.Map())
    image, grid = map2d.grid_image(info), maps.cells(info)
    row, column = np.argwhere(grid == 100)[0]
    assert tuple(image[info.grid_map.height - 1 - row, column]) == (0, 0, 0)
    row, column = np.argwhere(grid == 0)[0]
    assert tuple(image[info.grid_map.height - 1 - row, column]) == (255, 255, 255)


def test_guide_points_and_changed(fresh_world: _nav.NavWorld) -> None:
    map2d = load("14_map2d/map2d.py")
    map_api = agibot_gdk.Map()
    workshop = fresh_world.world_map
    old = map_api.get_map(workshop.id)
    assert [p.id for p in map2d.guide_points(old)] == [1, 2, 3]
    assert not map2d.changed(old, map_api.get_map(workshop.id))
    workshop.guide_pts = [*workshop.guide_pts, {
        "id": 4, "type": 0, "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}]
    assert map2d.changed(old, map_api.get_map(workshop.id))


def test_walls_and_zones_are_unreadable() -> None:
    info = maps.current_map(agibot_gdk.Map())
    for name in ("walls", "infeasible_areas"):
        with pytest.raises(TypeError, match="Unable to convert"):
            getattr(info, name)


# --- 15 map in 3D ---------------------------------------------------------------------------


def test_map_cloud_and_grid() -> None:
    map3d = load("15_map3d/map3d.py")
    info = maps.current_map(agibot_gdk.Map())
    cloud = map3d.map_cloud(info)
    assert cloud.shape == (133182, 3)
    geo = maps.geometry_of(info)
    grid = map3d.to_grid(np.array([[0.02, 0.02, 0.5], [0.02, 0.02, 3.0]]), geo, 0.1, 1.8)
    assert grid.sum() == 1 and grid[206, 265]  # floor((0.02 + 10.3) / 0.05), floor((0.02 + 13.25) / 0.05)


def test_agreement_of_the_robot_grid_with_itself_is_one() -> None:
    map3d = load("15_map3d/map3d.py")
    info = maps.current_map(agibot_gdk.Map())
    numbers = map3d.agreement(maps.cells(info) == 100, info)
    assert numbers["in occupied"] == 1.0 and numbers["robot found"] == 1.0


def test_the_cloud_fits_the_grid_only_on_the_map() -> None:
    map3d = load("15_map3d/map3d.py")
    info = maps.current_map(agibot_gdk.Map())
    geo, cloud = maps.geometry_of(info), map3d.map_cloud(info)
    found = {name: map3d.agreement(map3d.to_grid(points, geo, 0.1, 1.8), info)["robot found"]
             for name, points in (("stored", cloud), ("on map", map3d.to_map(cloud, agibot_gdk.TF())))}
    assert found["on map"] > 0.5 > found["stored"]


# --- 16 navigate ----------------------------------------------------------------------------


def test_navi_req_turns_the_yaw_into_a_quaternion() -> None:
    request = load("16_navigate/navigate.py").navi_req(1.0, 2.0, math.pi / 2)
    assert (request.target.position.x, request.target.position.y) == (1.0, 2.0)
    assert math.isclose(geometry.rpy(geometry.from_pose(request.target))[2], math.pi / 2)


def test_check_goal_sees_the_grid() -> None:
    navigate = load("16_navigate/navigate.py")
    info = maps.current_map(agibot_gdk.Map())
    assert navigate.check_goal(info, 100.0, 0.0, 0.4) == ["outside the map"]
    assert navigate.check_goal(info, *FREE_GOAL, 0.4) == []
    row, column = np.argwhere(maps.cells(info) == 100)[100]
    wall = (-13.25 + (column + 0.5) * 0.05, -10.3 + (row + 0.5) * 0.05)
    assert "not a free cell" in navigate.check_goal(info, *wall, 0.4)
    assert any("closer" in problem for problem in navigate.check_goal(info, wall[0] + 0.15, wall[1], 0.4))
    assert maps.cell_of(info, *wall) == (row, column)
    assert navigate.on_map(info, *wall) and not navigate.on_map(info, 100.0, 0.0)
    assert not navigate.is_free(info, *wall) and navigate.is_free(info, *FREE_GOAL)
    assert not navigate.has_clearance(info, *wall, 0.4) and navigate.has_clearance(info, *FREE_GOAL, 0.4)


def test_navigate_to_succeeds_and_fails_like_the_robot(instant: None, fresh_world: _nav.NavWorld) -> None:
    navigate = load("16_navigate/navigate.py")
    pnc = agibot_gdk.Pnc()
    done = navigate.navigate_to(pnc, *FREE_GOAL, 0.0, 10.0)
    assert (done.state, done.type) == ("success", "navigation")
    assert math.dist((fresh_world.x, fresh_world.y), FREE_GOAL) < 1e-6
    failed = navigate.navigate_to(pnc, *wall_goal(), 0.0, 10.0)
    assert failed.state == "fail"
    assert failed.message == _nav.GOAL_OCCUPIED and failed.attempts == _nav.RETRIES


def test_relative_move_drives_straight(instant: None, fresh_world: _nav.NavWorld) -> None:
    navigate = load("16_navigate/navigate.py")
    pnc = agibot_gdk.Pnc()
    fresh_world.x, fresh_world.y, fresh_world.yaw = 1.6, 2.8, math.pi / 2
    navigate.relative_move(pnc, 1.0, 0.0, 0.0)
    assert navigate.task(pnc).state == "success"
    assert math.isclose(fresh_world.x, 1.6, abs_tol=1e-6)
    assert math.isclose(fresh_world.y, 2.8 + _nav.RELATIVE_OVERSHOOT, abs_tol=1e-6)  # 1 m forward, with overshoot


def test_pause_resume_and_cancel_a_running_task() -> None:
    navigate = load("16_navigate/navigate.py")
    pnc = agibot_gdk.Pnc()
    navigate.go(pnc, *FREE_GOAL, 0.0)
    time.sleep(0.5)
    assert navigate.task(pnc).state == "running"
    navigate.pause(pnc)
    time.sleep(0.4)
    assert navigate.task(pnc).state == "paused"
    navigate.resume(pnc)
    time.sleep(0.4)
    assert navigate.task(pnc).state == "running"
    navigate.cancel(pnc)
    time.sleep(0.4)
    assert navigate.task(pnc).state == "canceled"


def test_navigation_is_refused_while_the_chassis_is_held() -> None:
    navigate, drive = load("16_navigate/navigate.py"), load("12_drive/drive.py")
    pnc = agibot_gdk.Pnc()
    drive.take_control(pnc)
    with pytest.raises(RuntimeError, match="NormalNavi failed"):
        navigate.go(pnc, *FREE_GOAL, 0.0)


def test_keys_send_a_guide_point_and_a_wall_goal_fails(instant: None) -> None:
    navigate = load("16_navigate/navigate.py")
    ui = navigate.Controller(agibot_gdk.Pnc(), agibot_gdk.Slam(), agibot_gdk.Map())
    keys = ui.keys()
    assert "click on the map" in show.render(ui.table())
    keys["1"]()
    assert ui.checks()[0]["check_goal"] == "ok"
    keys["w"]()
    ui.waiter.join(10)
    assert "navigation success" in ui.waited
    ui.typed_goal("{} {} 0".format(*wall_goal()))
    assert not ui.checks()[1]
    keys["w"]()
    ui.waiter.join(30)
    assert "fail (5 attempts) No valid path could be found: goal occupied" in ui.waited