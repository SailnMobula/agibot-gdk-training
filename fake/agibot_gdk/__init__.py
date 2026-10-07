"""Stand-in for agibot_gdk 3.3.8 that runs the exercises on a laptop.

It has the same names and return shapes as the real module for the calls the exercises use.
The behaviour is simplified. A joint moves in a straight ramp to its target. The frame poses
come from a toy model and do not follow the real kinematics. The head camera shows a grid with
a cross at the projection of each end effector.

FAKE_INSTANT=1 skips the time a blocking move takes.
"""

from __future__ import annotations

import math
import os
import random
import time
from enum import Enum, IntEnum
from types import SimpleNamespace

import cv2
import numpy as np

from gdk_training import geometry
from gdk_training.joints import ALL, HEAD, LEFT_ARM, LIMITS, RIGHT_ARM, WAIST

__version__ = "3.3.8-fake"


class GDKRes(IntEnum):
    kSuccess = 0
    kInvalidInput = 1
    kInvalidOutput = 2
    kRuntimeError = 3
    kUnknown = 4


class EndEffectorControlGroup(IntEnum):
    kUnknown = 0
    kLeftArm = 1
    kRightArm = 2
    kBothArms = 3


class CameraType(Enum):
    kHeadStereoLeft = 4
    kHeadStereoRight = 5
    kHandLeftColor = 6
    kHandRightColor = 7
    kHeadColor = 8
    kHeadDepth = 9


class Encoding(Enum):
    UNCOMPRESSED = 0
    JPEG = 1
    PNG = 2


class ColorFormat(Enum):
    RGB = 0
    BGR = 1
    GRAY8 = 2
    GRAY16 = 3
    RS2_FORMAT_Z16 = 4


class SensorExtrinsicType(Enum):
    kHeadDepthToHeadColor = 1
    kHeadRGBDToHeadLink3 = 2
    kLeftHandRGBDToArmLEndLink = 3
    kRightHandRGBDToArmREndLink = 4


class Vector3:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        self.x, self.y, self.z = x, y, z


class Quaternion:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0) -> None:
        self.x, self.y, self.z, self.w = x, y, z, w


class Pose:
    def __init__(self) -> None:
        self.position = Vector3()
        self.orientation = Quaternion()


class Transform:
    def __init__(self) -> None:
        self.translation = Vector3()
        self.rotation = Quaternion()


class Twist:
    def __init__(self) -> None:
        self.linear = Vector3()
        self.angular = Vector3()


class Wrench:
    def __init__(self) -> None:
        self.force = Vector3()
        self.torque = Vector3()


class JointControlReq:
    def __init__(self) -> None:
        self.life_time = 0.0
        self.joint_names: list[str] = []
        self.joint_positions: list[float] = []
        self.joint_velocities: list[float] = []
        self.detail = ""


class JointServoControlReq:
    def __init__(self) -> None:
        self.control_period = 0.0
        self.joint_names: list[str] = []
        self.joint_positions: list[float] = []
        self.joint_velocities: list[float] = []


class JointState:
    def __init__(self) -> None:
        self.position = 0.0


class JointStates:
    def __init__(self) -> None:
        self.group = ""
        self.target_type = ""
        self.states: list[JointState] = []
        self.nums = 0


class EndEffectorPose:
    def __init__(self) -> None:
        self.life_time = 0.0
        self.group = 0
        self.left_end_effector_pose = Pose()
        self.right_end_effector_pose = Pose()


def _pose(transform: np.ndarray) -> Pose:
    pose = Pose()
    pose.position = Vector3(*geometry.position(transform))
    pose.orientation = Quaternion(*geometry.quaternion(transform))
    return pose


def _transform(matrix: np.ndarray) -> Transform:
    transform = Transform()
    transform.translation = Vector3(*geometry.position(matrix))
    transform.rotation = Quaternion(*geometry.quaternion(matrix))
    return transform


def _translation(x: float, y: float, z: float) -> np.ndarray:
    return geometry.matrix((x, y, z), (0, 0, 0, 1))


GRIPPER_OPEN, GRIPPER_CLOSED = -0.785, 0.0
GRIPPER_SPEED = 2.0
TOOL_BLOCK_S = 5.0
END_EFFECTOR_HOME = {"l": (0.65, 0.30, 1.30), "r": (0.65, -0.30, 1.30)}
END_EFFECTOR_TO_GRIPPER = _translation(0.0, 0.0, 0.14)
# Pose of the camera optical frame in head_link3. The optical z axis points forward, x right, y down.
HEAD_TO_CAMERA = np.array([[0, 0, 1, 0.10], [-1, 0, 0, 0.0], [0, -1, 0, 0.07], [0, 0, 0, 1]], dtype=float)
HEAD_SHAPE = (640, 400)
HEAD_INTRINSIC = [305.0, 305.0, 319.0, 200.0]
FRAMES = ["base_link", "head_link3", "arm_l_end_link", "arm_r_end_link", "gripper_l_center_link",
          "gripper_r_center_link"]


class World:
    def __init__(self) -> None:
        self.position = dict.fromkeys(ALL, 0.0)
        self.velocity = dict.fromkeys(ALL, 0.0)
        self.servo_time: dict[str, float] = {}
        self.control_mode = 1
        self.streamed: dict[str, np.ndarray | None] = {"l": None, "r": None}
        self.gripper = {"left": GRIPPER_OPEN, "right": GRIPPER_OPEN}
        self.gripper_target = dict(self.gripper)
        self.gripper_time = time.monotonic()
        self.tool_blocked_until = 0.0
        self.frame = 0

    def end_effector_pose(self, side: str) -> np.ndarray:
        streamed = self.streamed[side]
        if streamed is not None:
            return streamed
        arm = LEFT_ARM if side == "l" else RIGHT_ARM
        q = [self.position[name] for name in arm]
        x, y, z = END_EFFECTOR_HOME[side]
        return geometry.matrix((x + 0.15 * math.sin(q[3]), y + 0.15 * math.sin(q[0]), z + 0.15 * math.sin(q[1])),
                               (0.0, math.sin(q[6] / 2), 0.0, math.cos(q[6] / 2)))

    def head_pose(self) -> np.ndarray:
        yaw, pitch = self.position[HEAD[0]], self.position[HEAD[1]]
        turn = geometry.matrix((0.05, 0.0, 1.45 + 0.3 * self.position[WAIST[1]]), (0, 0, math.sin(yaw / 2), math.cos(yaw / 2)))
        nod = geometry.matrix((0, 0, 0), (0, math.sin(pitch / 2), 0, math.cos(pitch / 2)))
        return turn @ nod

    def frame_pose(self, name: str) -> np.ndarray:
        """Returns the pose of the frame in base_link."""
        if name == "base_link":
            return np.eye(4)
        if name == "head_link3":
            return self.head_pose()
        if name in ("arm_l_end_link", "arm_r_end_link"):
            return self.end_effector_pose(name[4])
        if name in ("gripper_l_center_link", "gripper_r_center_link"):
            return self.end_effector_pose(name[8]) @ END_EFFECTOR_TO_GRIPPER
        raise RuntimeError(f"unknown frame {name}")

    def step_gripper(self) -> None:
        now = time.monotonic()
        reach = GRIPPER_SPEED * (now - self.gripper_time)
        self.gripper_time = now
        for side, target in self.gripper_target.items():
            delta = target - self.gripper[side]
            self.gripper[side] += max(-reach, min(reach, delta))


_world = World()


def gdk_init() -> GDKRes:
    return GDKRes.kSuccess


def gdk_release() -> GDKRes:
    return GDKRes.kSuccess


def get_app_version() -> str:
    return "fake"


class Robot:
    def get_joint_states(self) -> dict:
        states = []
        for name in ALL:
            position = _world.position[name]
            effort = 4.0 * math.sin(position) + random.gauss(0, 0.05)
            states.append({"name": name, "mode": 8, "position": 0.0, "velocity": 0.0, "effort": effort,
                           "motor_position": position, "motor_velocity": _world.velocity[name],
                           "motor_current": effort / 2, "error_code": 0})
        return {"timestamp": time.time_ns(), "nums": len(states), "states": states}

    def get_whole_body_status(self) -> dict:
        status = {"timestamp": time.time_ns(), "left_end_model": "omnipicker", "right_end_model": "omnipicker"}
        for part in ("right_arm", "left_arm", "right_end", "left_end", "waist", "lift", "neck", "chassis"):
            status[f"{part}_error"] = 0
        for side in ("right", "left"):
            status[f"{side}_arm_control"] = True
            status[f"{side}_arm_estop"] = False
        return status

    def get_motion_control_status(self) -> SimpleNamespace:
        wrenches = []
        for name in FRAMES:
            wrench = Wrench()
            if "arm" in name or "gripper" in name:
                wrench.force = Vector3(random.gauss(0, 0.2), random.gauss(0, 0.2), -6.0 + random.gauss(0, 0.2))
                wrench.torque = Vector3(*(random.gauss(0, 0.02) for _ in range(3)))
            wrenches.append(wrench)
        return SimpleNamespace(
            mode=1, control_mode=_world.control_mode, error_code=0, error_msg="",
            frame_names=list(FRAMES), frame_poses=[_pose(_world.frame_pose(name)) for name in FRAMES],
            twists=[Twist() for _ in FRAMES], wrenches=wrenches,
            arm_disturbance_force=[random.gauss(0, 0.1) for _ in range(14)],
            collision_pairs_1=[], collision_pairs_2=[],
        )

    def get_end_state(self) -> dict:
        _world.step_gripper()
        result = {}
        for side, joint in (("left", "idx31_gripper_l_inner_joint1"), ("right", "idx71_gripper_r_inner_joint1")):
            moving = abs(_world.gripper_target[side] - _world.gripper[side]) > 1e-6
            result[f"{side}_end_state"] = {
                "controlled": True, "type": "omnipicker", "names": [joint],
                "end_states": [{"id": 1, "enable": True, "position": _world.gripper[side],
                                "velocity": GRIPPER_SPEED if moving else 0.0, "effort": 0.0, "current": 0.1,
                                "voltage": 24.0, "temperature": 31.0, "status": 0, "err_code": 0}],
            }
        return result

    def get_chassis_power_state(self) -> SimpleNamespace:
        battery = SimpleNamespace(battery_soc=78, battery_output_voltage=52.1, battery_output_current=6.4,
                                  battery_temperature=29.0, battery_charging_status=0)
        return SimpleNamespace(timestamp=time.time_ns(), battery_states=[battery, battery],
                               charge_plug_insert_state=0, emergency_stop_pedal_state=0)

    def set_control_mode(self, control_mode: int) -> int:
        if control_mode not in (1, 2, 3):
            raise RuntimeError("control_mode must be 1, 2 or 3")
        _world.control_mode = control_mode
        return 0

    def clear_motion_control_error(self) -> int:
        return 0

    def _check(self, names: list[str], positions: list[float]) -> None:
        if _world.control_mode == 2:
            raise RuntimeError("joint interfaces need control mode 1 or 3, robot is in 2")
        if not names or len(names) != len(positions):
            raise RuntimeError("kInvalidInput: joint_names and joint_positions differ in length")
        for name, position in zip(names, positions):
            if name not in LIMITS:
                raise RuntimeError(f"kInvalidInput: unknown joint {name}")
            low, high = LIMITS[name]
            if not low - 1e-3 <= position <= high + 1e-3:
                raise RuntimeError(f"kInvalidInput: {name} {position:.3f} outside {low} .. {high}")

    def _release_stream(self, names: list[str]) -> None:
        for side, arm in (("l", LEFT_ARM), ("r", RIGHT_ARM)):
            if set(names) & set(arm):
                _world.streamed[side] = None

    def joint_control_request(self, request: JointControlReq) -> int:
        names, positions = list(request.joint_names), list(request.joint_positions)
        self._check(names, positions)
        speeds = list(request.joint_velocities) or [0.3]
        if len(speeds) == 1:
            speeds = speeds * len(names)
        duration = max(abs(p - _world.position[n]) / max(v, 1e-3) for n, p, v in zip(names, positions, speeds))
        self._release_stream(names)
        start = [_world.position[name] for name in names]
        steps = 0 if os.environ.get("FAKE_INSTANT") else int(min(duration, 10.0) / 0.02)
        for step in range(1, steps + 1):
            for name, begin, position in zip(names, start, positions):
                _world.position[name] = begin + (position - begin) * step / steps
                _world.velocity[name] = (position - begin) / (steps * 0.02)
            time.sleep(0.02)
        for name, position in zip(names, positions):
            _world.position[name] = position
            _world.velocity[name] = 0.0
        return 0

    def move_head_joint(self, positions: list[float], velocities: list[float]) -> int:
        return self._move(HEAD, positions, velocities)

    def move_waist_joint(self, positions: list[float], velocities: list[float]) -> int:
        return self._move(WAIST, positions, velocities)

    def move_arm_joint(self, positions: list[float], velocities: list[float], control_group: int) -> int:
        names = {0: LEFT_ARM, 1: RIGHT_ARM, 2: LEFT_ARM + RIGHT_ARM}[control_group]
        return self._move(names, positions, velocities)

    def _move(self, names: list[str], positions: list[float], velocities: list[float]) -> int:
        request = JointControlReq()
        request.joint_names, request.joint_positions, request.joint_velocities = names, positions, velocities
        return self.joint_control_request(request)

    def joint_servo_control(self, request: JointServoControlReq, enable_low_latency: bool = False) -> int:
        names, positions = list(request.joint_names), list(request.joint_positions)
        self._check(names, positions)
        self._release_stream(names)
        now = time.monotonic()
        for name, position in zip(names, positions):
            elapsed = now - _world.servo_time.get(name, now - request.control_period)
            _world.velocity[name] = (position - _world.position[name]) / max(elapsed, 1e-3)
            _world.position[name] = position
            _world.servo_time[name] = now
        return 0

    def move_ee_pos(self, command: JointStates) -> int:
        _world.step_gripper()
        sides = {"left_tool": ["left"], "right_tool": ["right"], "dual_tool": ["left", "right"]}.get(command.group)
        if sides is None or command.nums != len(command.states) or len(command.states) != len(sides):
            raise RuntimeError("kInvalidInput: group and states do not match")
        if command.target_type != "omnipicker":
            raise RuntimeError(f"kInvalidInput: target_type {command.target_type}")
        if time.monotonic() < _world.tool_blocked_until:
            return 0  # The command is dropped, as on the robot after an end-effector pose stream.
        for side, state in zip(sides, command.states):
            _world.gripper_target[side] = max(GRIPPER_OPEN, min(GRIPPER_CLOSED, float(state.position)))
        return 0

    def end_effector_pose_control(self, pose: EndEffectorPose) -> int:
        group = int(pose.group)
        if group not in (1, 2, 3):
            raise RuntimeError("kInvalidInput: group")
        targets = {"l": pose.left_end_effector_pose, "r": pose.right_end_effector_pose}
        for side in {1: "l", 2: "r", 3: "lr"}[group]:
            current, target = _world.end_effector_pose(side), geometry.from_pose(targets[side])
            if np.linalg.norm(target[:3, 3] - current[:3, 3]) > 0.05:
                raise RuntimeError("target jumps more than 5 cm from the current pose")
            lagged = current[:3, 3] + 0.6 * (target[:3, 3] - current[:3, 3])
            _world.streamed[side] = geometry.matrix(
                tuple(lagged), geometry.slerp(geometry.quaternion(current), geometry.quaternion(target), 0.6))
        _world.tool_blocked_until = time.monotonic() + TOOL_BLOCK_S
        return 0


class TF:
    def get_all_frame_names(self) -> list[str]:
        return list(FRAMES)

    def can_transform(self, target_frame: str, source_frame: str) -> bool:
        return target_frame in FRAMES and source_frame in FRAMES

    def get_tf_from_base_link(self, child_frame_id: str) -> Transform:
        return _transform(_world.frame_pose(child_frame_id))

    def lookup_transform_latest(self, target_frame: str, source_frame: str,
                                return_timestamp: bool = False) -> tuple[Transform, int | None]:
        source_from_target = geometry.inverse(_world.frame_pose(source_frame)) @ _world.frame_pose(target_frame)
        return _transform(source_from_target), time.time_ns() if return_timestamp else None

    def get_tf_from_sensor(self, sensor_extrinsic_type: SensorExtrinsicType) -> Transform:
        if sensor_extrinsic_type == SensorExtrinsicType.kHeadRGBDToHeadLink3:
            return _transform(HEAD_TO_CAMERA)
        if sensor_extrinsic_type == SensorExtrinsicType.kHeadDepthToHeadColor:
            return _transform(_translation(0.015, 0.0, 0.0))
        return _transform(_translation(0.0, 0.06, 0.05))


class CameraIntrinsic:
    def __init__(self, intrinsic: list[float]) -> None:
        self.intrinsic = intrinsic
        self.distortion = [0.0] * 5


def _head_image() -> np.ndarray:
    width, height = HEAD_SHAPE
    image = np.full((height, width, 3), 40, np.uint8)
    image[::40, :] = 70
    image[:, ::40] = 70
    fx, fy, cx, cy = HEAD_INTRINSIC
    camera_from_base = geometry.inverse(_world.head_pose() @ HEAD_TO_CAMERA)
    for side, color in (("l", (60, 60, 255)), ("r", (255, 120, 60))):
        x, y, z, _ = camera_from_base @ _world.end_effector_pose(side)[:, 3]
        if z > 0.05:
            cv2.drawMarker(image, (int(cx + fx * x / z), int(cy + fy * y / z)), color, cv2.MARKER_CROSS, 24, 2)
    cv2.putText(image, f"fake head  frame {_world.frame}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    return image


class Camera:
    def __init__(self, camera_types: list[CameraType] | None = None) -> None:
        self.types = set(camera_types or CameraType)

    def get_image_shape(self, camera_type: CameraType) -> tuple[int, int]:
        return HEAD_SHAPE if camera_type in (CameraType.kHeadColor, CameraType.kHeadDepth) else (1280, 1056)

    def get_image_fps(self, camera_type: CameraType) -> float:
        return 30.0

    def get_camera_intrinsic(self, camera_type: CameraType) -> CameraIntrinsic:
        if camera_type in (CameraType.kHeadColor, CameraType.kHeadDepth):
            return CameraIntrinsic(list(HEAD_INTRINSIC))
        return CameraIntrinsic([900.0, 900.0, 640.0, 528.0])

    def get_latest_image(self, camera_type: CameraType, timeout_ms: float) -> SimpleNamespace:
        if camera_type not in self.types:
            raise RuntimeError(f"{camera_type} was not requested from this Camera")
        _world.frame += 1
        width, height = self.get_image_shape(camera_type)
        if camera_type == CameraType.kHeadDepth:
            depth = np.tile(np.linspace(400, 2500, width, dtype=np.uint16), (height, 1))
            return SimpleNamespace(timestamp_ns=time.time_ns(), width=width, height=height, bit_depth=16,
                                   encoding=Encoding.UNCOMPRESSED, color_format=ColorFormat.RS2_FORMAT_Z16,
                                   data=depth.view(np.uint8).reshape(-1))
        if camera_type == CameraType.kHeadColor:
            image = _head_image()
        else:
            image = np.full((height, width, 3), 60, np.uint8)
            cv2.putText(image, f"fake {camera_type.name}", (40, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (200, 200, 200), 2)
        return SimpleNamespace(timestamp_ns=time.time_ns(), width=width, height=height, bit_depth=8,
                               encoding=Encoding.JPEG, color_format=ColorFormat.BGR,
                               data=cv2.imencode(".jpg", image)[1].reshape(-1))

    def close_camera(self) -> GDKRes:
        return GDKRes.kSuccess
