#!/usr/bin/env python3

from __future__ import annotations

import importlib
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import rclpy
from quadrotor_msgs.msg import GripperCommandPair, GripperFeedback
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64, String


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
REF_CODE_DIR = PACKAGE_DIR.parent.parent.parent

SDK_LOADED = False
COMM_SUCCESS = None
PortHandler = None
hls = None
HLS_PRESENT_POSITION_L = None
HLS_PRESENT_CURRENT_H = None
HLS_PRESENT_SPEED_L = None
HLS_PRESENT_LOAD_L = None
HLS_PRESENT_VOLTAGE = None
HLS_PRESENT_TEMPERATURE = None
HLS_MOVING = None
HLS_PRESENT_CURRENT_L = None
HLS_MODE = None


STATE_OPEN = "OPEN"
STATE_SEARCH_OBJECT = "SEARCH_OBJECT"
STATE_LEFT_CONTACT = "LEFT_CONTACT"
STATE_RIGHT_CONTACT = "RIGHT_CONTACT"
STATE_BOTH_CONTACT = "BOTH_CONTACT"
STATE_CENTERING = "CENTERING"
STATE_CENTERED = "CENTERED"
STATE_FINAL_GRIP = "FINAL_GRIP"
STATE_LIFT_READY = "LIFT_READY"
STATE_FAULT = "FAULT"

SIDE_LEFT = "left"
SIDE_RIGHT = "right"


@dataclass(frozen=True)
class ServoCalibration:
    servo_id: int
    open_pos: int
    clear_pos: int
    close_pos: int
    inward_sign: int


@dataclass(frozen=True)
class MotionProfile:
    profile_index: int
    profile_name: str
    speed: int
    acc: int
    torque_limit: int


@dataclass
class ServoFeedback:
    pos: float
    close_ratio: float
    speed: float
    load: float
    voltage: float
    temp: float
    moving: float
    current: float
    baseline_current: float = 0.0
    residual_current: float = 0.0


@dataclass(frozen=True)
class CommandIntent:
    mode: Literal["open", "grasp", "hold"]
    value: float


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def quaternion_to_euler_deg(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def default_sdk_candidates() -> list[Path]:
    return [
        REF_CODE_DIR / "FT-servo" / "FTServo_Python-main" / "FTServo_Python-main",
        Path.cwd().parent / "FT-servo" / "FTServo_Python-main" / "FTServo_Python-main",
        Path.cwd() / "lerobot" / "ref_code" / "FT-servo" / "FTServo_Python-main" / "FTServo_Python-main",
        Path.cwd() / "FT-servo" / "FTServo_Python-main" / "FTServo_Python-main",
    ]


def load_sdk(sdk_root: str = "") -> None:
    global SDK_LOADED
    global COMM_SUCCESS
    global PortHandler
    global hls
    global HLS_PRESENT_POSITION_L
    global HLS_PRESENT_CURRENT_H
    global HLS_PRESENT_SPEED_L
    global HLS_PRESENT_LOAD_L
    global HLS_PRESENT_VOLTAGE
    global HLS_PRESENT_TEMPERATURE
    global HLS_MOVING
    global HLS_PRESENT_CURRENT_L
    global HLS_MODE

    if SDK_LOADED:
        return

    candidates: list[Path] = []
    if sdk_root:
        candidates.append(Path(sdk_root).expanduser())
    candidates.extend(default_sdk_candidates())

    for sdk_path in candidates:
        if (sdk_path / "scservo_sdk").is_dir():
            sys.path.insert(0, str(sdk_path))
            break

    try:
        sdk = importlib.import_module("scservo_sdk")
    except ImportError as exc:
        raise RuntimeError(
            "HLS gripper node requires the FTServo Python SDK. Pass sdk_root if the SDK is not "
            "under lerobot/ref_code/FT-servo/FTServo_Python-main/FTServo_Python-main. "
            f"Import error: {exc}"
        ) from exc
    if not hasattr(sdk, "hls"):
        searched = ", ".join(str(path) for path in candidates)
        raise RuntimeError(
            "Imported scservo_sdk does not provide HLS support. "
            "Pass sdk_root:=/home/user/vla_drone/lerobot/ref_code/FT-servo/"
            "FTServo_Python-main/FTServo_Python-main or run from the vla_px4ctrl_ros2 workspace. "
            f"Imported module={getattr(sdk, '__file__', '<unknown>')} searched={searched}"
        )

    COMM_SUCCESS = sdk.COMM_SUCCESS
    PortHandler = sdk.PortHandler
    hls = sdk.hls
    HLS_PRESENT_POSITION_L = sdk.HLS_PRESENT_POSITION_L
    HLS_PRESENT_CURRENT_H = sdk.HLS_PRESENT_CURRENT_H
    HLS_PRESENT_SPEED_L = sdk.HLS_PRESENT_SPEED_L
    HLS_PRESENT_LOAD_L = sdk.HLS_PRESENT_LOAD_L
    HLS_PRESENT_VOLTAGE = sdk.HLS_PRESENT_VOLTAGE
    HLS_PRESENT_TEMPERATURE = sdk.HLS_PRESENT_TEMPERATURE
    HLS_MOVING = sdk.HLS_MOVING
    HLS_PRESENT_CURRENT_L = sdk.HLS_PRESENT_CURRENT_L
    HLS_MODE = sdk.HLS_MODE
    SDK_LOADED = True


def default_compensation_candidates() -> list[Path]:
    candidates = [
        PACKAGE_DIR / "config" / "gravity_compensation.json",
        Path.cwd() / "src" / "hls_gripper" / "config" / "gravity_compensation.json",
        Path.cwd() / "lerobot" / "ref_code" / "vla_px4ctrl_ros2" / "src" / "hls_gripper" / "config" / "gravity_compensation.json",
    ]
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(Path(get_package_share_directory("hls_gripper")) / "config" / "gravity_compensation.json")
    except Exception:
        pass
    return candidates


def resolve_compensation_path(configured_path: str) -> Path | None:
    if configured_path:
        path = Path(configured_path).expanduser()
        return path if path.is_file() else None
    for path in default_compensation_candidates():
        if path.is_file():
            return path
    return None


def ratio_to_pos(open_pos: int, close_pos: int, ratio: float) -> int:
    ratio = clamp(ratio, 0.0, 1.0)
    return int(round(open_pos + (close_pos - open_pos) * ratio))


def pos_to_ratio(open_pos: int, close_pos: int, pos: float) -> float:
    span = close_pos - open_pos
    if span == 0:
        return 0.0
    return clamp((float(pos) - open_pos) / float(span), 0.0, 1.0)


def int_from_mapping(mapping: dict, key: str, default: int) -> int:
    value = mapping.get(key, default)
    if value is None or value == "":
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def float_from_mapping(mapping: dict, key: str, default: float) -> float:
    value = mapping.get(key, default)
    if value is None or value == "":
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


class GravityCompensationTable:
    def __init__(self, path: Path | None) -> None:
        self.path = path
        self.table = None
        self.points: dict[str, list[dict]] = {SIDE_LEFT: [], SIDE_RIGHT: []}
        self.calibration: dict[str, ServoCalibration] = {}
        self.contact_detection: dict[str, dict] = {SIDE_LEFT: {}, SIDE_RIGHT: {}}
        self.contact_detection_by_profile: dict[str, dict] = {}
        self.motion_profiles: list[MotionProfile] = []
        if path is not None:
            with open(path) as json_file:
                self.table = json.load(json_file)
            self._load_table(self.table)

    @property
    def loaded(self) -> bool:
        return self.table is not None

    def _load_table(self, table: dict) -> None:
        servos = table.get("servos", {})
        for side in (SIDE_LEFT, SIDE_RIGHT):
            servo = servos.get(side, {})
            servo_id = int_from_mapping(servo, "id", 1 if side == SIDE_LEFT else 2)
            open_pos = int_from_mapping(servo, "open_pos", -1)
            close_pos = int_from_mapping(servo, "close_pos", -1)
            clear_pos = int_from_mapping(servo, "clear_pos", close_pos)
            inward_sign = int_from_mapping(servo, "inward_sign", 1 if side == SIDE_LEFT else -1)
            if open_pos >= 0 and close_pos >= 0 and open_pos != close_pos:
                self.calibration[side] = ServoCalibration(
                    servo_id=servo_id,
                    open_pos=open_pos,
                    clear_pos=clear_pos if clear_pos >= 0 else close_pos,
                    close_pos=close_pos,
                    inward_sign=1 if inward_sign >= 0 else -1,
                )
            self.points[side] = list(servo.get("points", []))

        detection = table.get("contact_detection", {})
        for side in (SIDE_LEFT, SIDE_RIGHT):
            side_detection = detection.get(side, {})
            if isinstance(side_detection, dict):
                self.contact_detection[side] = dict(side_detection)
        profiles_detection = detection.get("profiles", {})
        if isinstance(profiles_detection, dict):
            self.contact_detection_by_profile = {
                str(key): value for key, value in profiles_detection.items() if isinstance(value, dict)
            }

        for profile in table.get("collection", {}).get("profiles", []):
            try:
                profile_index = int(profile.get("profile_index", len(self.motion_profiles)))
                profile_name = str(profile.get("profile_name") or f"p{profile_index}")
                speed = int(profile.get("speed", profile.get("speed_cmd", 0)))
                acc = int(profile.get("acc", profile.get("acc_cmd", 0)))
                torque_limit = int(profile.get("torque_limit", 0))
            except (TypeError, ValueError):
                continue
            if speed > 0 and acc > 0 and torque_limit > 0:
                self.motion_profiles.append(
                    MotionProfile(
                        profile_index=profile_index,
                        profile_name=profile_name,
                        speed=speed,
                        acc=acc,
                        torque_limit=torque_limit,
                    )
                )

    def baseline_current(
        self,
        side: str,
        close_ratio: float,
        roll_deg: float,
        pitch_deg: float,
        profile_index: int | None = None,
        profile_name: str = "",
        direction: str = "",
    ) -> float:
        points = self.points.get(side, [])
        if not points:
            return 0.0
        profile_points = []
        for point in points:
            try:
                point_index = int(point.get("profile_index"))
            except (TypeError, ValueError):
                point_index = None
            point_name = str(point.get("profile_name") or "")
            point_direction = str(point.get("direction") or "")
            if direction and point_direction and point_direction != direction:
                continue
            if profile_index is not None and point_index != profile_index:
                continue
            if profile_name and point_name and point_name != profile_name:
                continue
            profile_points.append(point)
        if profile_points:
            points = profile_points
        elif direction:
            direction_points = [
                point
                for point in points
                if str(point.get("direction") or "") in ("", direction)
            ]
            if direction_points:
                points = direction_points

        close_ratio = clamp(close_ratio, 0.0, 1.0)
        if not math.isfinite(roll_deg):
            roll_deg = 0.0
        if not math.isfinite(pitch_deg):
            pitch_deg = 0.0

        best_point = points[0]
        best_score = float("inf")
        for point in points:
            try:
                ratio = float(point.get("close_ratio", 0.0))
                roll = float(point.get("roll_bin_deg", 0.0) or 0.0)
                pitch = float(point.get("pitch_bin_deg", 0.0) or 0.0)
            except (TypeError, ValueError):
                continue

            score = abs(ratio - close_ratio) / 0.05 + abs(roll - roll_deg) / 5.0 + abs(pitch - pitch_deg) / 5.0
            if score < best_score:
                best_score = score
                best_point = point

        try:
            return float(best_point.get("current_median", 0.0))
        except (TypeError, ValueError):
            return 0.0


class HlsBus:
    def __init__(self, port_name: str, baudrate: int, sdk_root: str = "") -> None:
        load_sdk(sdk_root)
        self.port_name = port_name
        self.baudrate = baudrate
        self.port = PortHandler(port_name)
        self.packet = hls(self.port)
        self.mode: dict[int, str] = {}

    def connect(self) -> None:
        if not self.port.openPort():
            raise RuntimeError(f"failed to open HLS port {self.port_name}")
        if not self.port.setBaudRate(self.baudrate):
            raise RuntimeError(f"failed to set HLS baudrate {self.baudrate}")

    def close(self) -> None:
        self.port.closePort()

    def _check(self, result: int, error: int, label: str) -> None:
        if result != COMM_SUCCESS:
            raise RuntimeError(f"{label}: {self.packet.getTxRxResult(result)}")
        if error:
            raise RuntimeError(f"{label}: {self.packet.getRxPacketError(error)}")

    def read_mode(self, servo_id: int) -> int:
        mode, result, error = self.packet.read1ByteTxRx(servo_id, HLS_MODE)
        self._check(result, error, f"read mode id={servo_id}")
        return int(mode)

    def write_mode_checked(self, servo_id: int, mode_name: str, mode_fn, expected_mode: int) -> None:
        result, error = self.packet.EnableTorque(servo_id, 0)
        self._check(result, error, f"disable torque id={servo_id}")

        result, error = mode_fn(servo_id)
        self._check(result, error, f"{mode_name} id={servo_id}")
        time.sleep(0.05)
        mode = self.read_mode(servo_id)

        if mode != expected_mode:
            print(
                f"id={servo_id} mode readback is {mode} after {mode_name}; retrying with EEPROM unlock"
            )
            result, error = self.packet.unLockEprom(servo_id)
            self._check(result, error, f"unlock EPROM id={servo_id}")
            time.sleep(0.02)
            result, error = mode_fn(servo_id)
            self._check(result, error, f"{mode_name} after unlock id={servo_id}")
            time.sleep(0.05)
            result, error = self.packet.LockEprom(servo_id)
            self._check(result, error, f"lock EPROM id={servo_id}")
            mode = self.read_mode(servo_id)

        if mode != expected_mode:
            raise RuntimeError(
                f"failed to switch id={servo_id} to {mode_name}: mode readback={mode} expected={expected_mode}"
            )

        result, error = self.packet.EnableTorque(servo_id, 1)
        self._check(result, error, f"enable torque id={servo_id}")

    def ping(self, servo_id: int) -> None:
        _model, result, error = self.packet.ping(servo_id)
        self._check(result, error, f"ping id={servo_id}")

    def set_position_mode(self, servo_id: int) -> None:
        if self.mode.get(servo_id) == "position":
            return
        self.write_mode_checked(servo_id, "ServoMode", self.packet.ServoMode, 0)
        self.mode[servo_id] = "position"

    def set_ele_mode(self, servo_id: int) -> None:
        if self.mode.get(servo_id) == "ele":
            return
        self.write_mode_checked(servo_id, "EleMode", self.packet.EleMode, 2)
        self.mode[servo_id] = "ele"

    def write_position(self, servo_id: int, position: int, speed: int, acc: int, torque_limit: int) -> None:
        self.set_position_mode(servo_id)
        result, error = self.packet.WritePosEx(servo_id, int(position), int(speed), int(acc), int(torque_limit))
        self._check(result, error, f"WritePosEx id={servo_id}")

    def write_current(self, servo_id: int, current: int) -> None:
        self.set_ele_mode(servo_id)
        result, error = self.packet.WriteEle(servo_id, int(current))
        self._check(result, error, f"WriteEle id={servo_id}")

    def read_feedback(self, servo_id: int, cal: ServoCalibration) -> ServoFeedback:
        length = HLS_PRESENT_CURRENT_H - HLS_PRESENT_POSITION_L + 1
        data, result, error = self.packet.readTxRx(servo_id, HLS_PRESENT_POSITION_L, length)
        self._check(result, error, f"read feedback id={servo_id}")

        def word(addr: int) -> int:
            offset = addr - HLS_PRESENT_POSITION_L
            return self.packet.scs_makeword(data[offset], data[offset + 1])

        pos = float(self.packet.scs_tohost(word(HLS_PRESENT_POSITION_L), 15))
        return ServoFeedback(
            pos=pos,
            close_ratio=pos_to_ratio(cal.open_pos, cal.close_pos, pos),
            speed=float(self.packet.scs_tohost(word(HLS_PRESENT_SPEED_L), 15)),
            load=float(self.packet.scs_tohost(word(HLS_PRESENT_LOAD_L), 10)),
            voltage=float(data[HLS_PRESENT_VOLTAGE - HLS_PRESENT_POSITION_L]),
            temp=float(data[HLS_PRESENT_TEMPERATURE - HLS_PRESENT_POSITION_L]),
            moving=float(data[HLS_MOVING - HLS_PRESENT_POSITION_L]),
            current=float(self.packet.scs_tohost(word(HLS_PRESENT_CURRENT_L), 15)),
        )


class HlsGripperNode(Node):
    def __init__(self) -> None:
        super().__init__("hls_gripper_node")

        self.command_topic = str(self.declare_parameter("command_topic", "/gripper/command").value)
        self.command_pair_topic = str(self.declare_parameter("command_pair_topic", "/gripper/command_pair").value)
        self.feedback_topic = str(self.declare_parameter("feedback_topic", "/gripper/feedback").value)
        legacy_status_topic = str(self.declare_parameter("status_topic", "/hls_gripper/status").value)
        status_prefix_param = str(self.declare_parameter("status_prefix", "").value)
        if status_prefix_param:
            self.status_prefix = status_prefix_param.rstrip("/")
        elif legacy_status_topic.endswith("/status"):
            self.status_prefix = legacy_status_topic[: -len("/status")]
        else:
            self.status_prefix = legacy_status_topic.rstrip("/")
        self.attitude_topic = str(self.declare_parameter("attitude_topic", "/mavros/imu/data").value)
        self.port = str(self.declare_parameter("port", "/dev/ttyACM1").value)
        self.baud = int(self.declare_parameter("baud", 1000000).value)
        self.sdk_root = str(self.declare_parameter("sdk_root", "").value)
        self.dry_run = bool(self.declare_parameter("dry_run", False).value)

        self.left_id = int(self.declare_parameter("left_id", -1).value)
        self.right_id = int(self.declare_parameter("right_id", -1).value)
        self.left_open = int(self.declare_parameter("left_open", -1).value)
        self.left_clear = int(self.declare_parameter("left_clear", -1).value)
        self.left_close = int(self.declare_parameter("left_close", -1).value)
        self.right_open = int(self.declare_parameter("right_open", -1).value)
        self.right_clear = int(self.declare_parameter("right_clear", -1).value)
        self.right_close = int(self.declare_parameter("right_close", -1).value)
        self.left_inward_sign = int(self.declare_parameter("left_inward_sign", 0).value)
        self.right_inward_sign = int(self.declare_parameter("right_inward_sign", 0).value)
        self.left_current_inward_sign_param = int(self.declare_parameter("left_current_inward_sign", 0).value)
        self.right_current_inward_sign_param = int(self.declare_parameter("right_current_inward_sign", 0).value)

        self.gravity_comp_path_param = str(self.declare_parameter("gravity_comp_path", "").value)
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 30.0).value)
        self.feedback_rate_hz = float(self.declare_parameter("feedback_rate_hz", 20.0).value)
        self.open_enter_threshold = float(self.declare_parameter("open_enter_threshold", 80.0).value)
        self.close_enter_threshold = float(self.declare_parameter("close_enter_threshold", 20.0).value)
        self.motion_profile_name_param = str(self.declare_parameter("motion_profile", "").value)
        self.motion_profile_index_param = int(self.declare_parameter("motion_profile_index", -1).value)
        self.search_speed_param = int(self.declare_parameter("search_speed", -1).value)
        self.search_acc_param = int(self.declare_parameter("search_acc", -1).value)
        self.search_torque_limit_param = int(self.declare_parameter("search_torque_limit", -1).value)
        self.open_speed = int(self.declare_parameter("open_speed", 40).value)
        self.open_acc = int(self.declare_parameter("open_acc", 10).value)
        self.open_torque_limit = int(self.declare_parameter("open_torque_limit", 300).value)
        self.low_current = int(self.declare_parameter("low_current", 40).value)
        self.lift_current = int(self.declare_parameter("lift_current", 120).value)
        self.center_hold_current_param = int(self.declare_parameter("center_hold_current", -1).value)
        self.center_push_current_param = int(self.declare_parameter("center_push_current", -1).value)
        self.grip_chase_min_current_param = int(self.declare_parameter("grip_chase_min_current", -1).value)
        self.center_timeout_action = str(self.declare_parameter("center_timeout_action", "final_grip").value)
        self.final_grip_ramp_s = float(self.declare_parameter("final_grip_ramp_s", 1.2).value)
        self.contact_current_threshold = float(self.declare_parameter("contact_current_threshold", -1.0).value)
        self.contact_exit_threshold = float(self.declare_parameter("contact_exit_threshold", -1.0).value)
        self.contact_strong_threshold = float(self.declare_parameter("contact_strong_threshold", -1.0).value)
        self.left_contact_metric_sign_param = int(self.declare_parameter("left_contact_metric_sign", 0).value)
        self.right_contact_metric_sign_param = int(self.declare_parameter("right_contact_metric_sign", 0).value)
        self.contact_confirm_cycles = int(self.declare_parameter("contact_confirm_cycles", 3).value)
        self.max_current = float(self.declare_parameter("max_current", 600.0).value)
        self.max_temp = float(self.declare_parameter("max_temp", 70.0).value)
        self.feedback_timeout_s = float(self.declare_parameter("feedback_timeout_s", 0.5).value)
        self.attitude_timeout_s = float(self.declare_parameter("attitude_timeout_s", 0.5).value)
        self.command_timeout_s = float(self.declare_parameter("command_timeout_s", 0.0).value)
        self.search_timeout_s = float(self.declare_parameter("search_timeout_s", 5.0).value)
        self.single_contact_timeout_s = float(self.declare_parameter("single_contact_timeout_s", 20.0).value)
        self.single_contact_limit_ratio = float(self.declare_parameter("single_contact_limit_ratio", 0.97).value)
        self.center_timeout_s = float(self.declare_parameter("center_timeout_s", 8.0).value)
        self.final_grip_timeout_s = float(self.declare_parameter("final_grip_timeout_s", 3.0).value)
        self.center_deadband_m = float(self.declare_parameter("center_deadband_m", 0.005).value)
        self.center_stable_time_s = float(self.declare_parameter("center_stable_time_s", 0.5).value)
        self.finger_length_m = float(self.declare_parameter("finger_length_m", 0.19).value)
        self.servo_ticks_per_rev = float(self.declare_parameter("servo_ticks_per_rev", 4096.0).value)
        self.center_bias = float(self.declare_parameter("center_bias", 0.0).value)
        self.center_sign = float(self.declare_parameter("center_sign", 1.0).value)
        self.center_gain_override = float(self.declare_parameter("center_gain_m_per_ratio", 0.0).value)
        self.dry_run_contact_pattern = str(self.declare_parameter("dry_run_contact_pattern", "both").value)

        comp_path = resolve_compensation_path(self.gravity_comp_path_param)
        self.compensation = GravityCompensationTable(comp_path)
        self.calibration = self._build_calibration()
        self.motion_profile = self._select_motion_profile()
        self.search_speed = self.search_speed_param if self.search_speed_param > 0 else self.motion_profile.speed
        self.search_acc = self.search_acc_param if self.search_acc_param > 0 else self.motion_profile.acc
        self.search_torque_limit = (
            self.search_torque_limit_param if self.search_torque_limit_param > 0 else self.motion_profile.torque_limit
        )
        self.center_hold_current = (
            self.center_hold_current_param if self.center_hold_current_param > 0 else self.low_current
        )
        self.center_push_current = (
            self.center_push_current_param
            if self.center_push_current_param > 0
            else max(self.center_hold_current, min(self.lift_current, int(round(0.75 * self.lift_current))))
        )
        self.grip_chase_min_current = (
            self.grip_chase_min_current_param
            if self.grip_chase_min_current_param > 0
            else self.center_push_current
        )
        self.contact_detection = self._build_contact_detection()
        self.current_inward_sign = self._build_current_inward_sign()
        self.center_gain_m_per_ratio = self._compute_center_gain()

        self.bus: HlsBus | None = None
        self.state = STATE_OPEN
        self.fault_reason = ""
        self.state_started_s = time.monotonic()
        self.centered_since_s: float | None = None
        self.left_contact_cycles = 0
        self.right_contact_cycles = 0
        self.left_contact = False
        self.right_contact = False
        self.fault_requires_open_reset = False
        self.last_fault_grasp_warn_s = 0.0
        self.last_feedback_s: float | None = None
        self.last_command_s: float | None = None
        self.last_open_write_s = 0.0
        self.last_search_write_s = 0.0
        self.last_ele_write_s = 0.0
        self.latest_roll_deg = 0.0
        self.latest_pitch_deg = 0.0
        self.latest_yaw_deg = 0.0
        self.latest_attitude_s: float | None = None
        self.feedback: dict[str, ServoFeedback] = self._initial_feedback()
        self.goal_left_pos = 100.0
        self.goal_right_pos = 100.0

        if not self.dry_run:
            self.bus = HlsBus(self.port, self.baud, self.sdk_root)
            self.bus.connect()
            for cal in self.calibration.values():
                self.bus.ping(cal.servo_id)

        self.create_subscription(Float64, self.command_topic, self._command_cb, 10)
        self.create_subscription(GripperCommandPair, self.command_pair_topic, self._command_pair_cb, 10)
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Imu, self.attitude_topic, self._imu_cb, imu_qos)
        self.feedback_pub = self.create_publisher(GripperFeedback, self.feedback_topic, 10)
        self.state_pub = self.create_publisher(String, f"{self.status_prefix}/state", 10)
        self.fault_reason_pub = self.create_publisher(String, f"{self.status_prefix}/fault_reason", 10)
        self.motion_profile_pub = self.create_publisher(String, f"{self.status_prefix}/motion_profile", 10)
        self.float_status_pubs = {
            name: self.create_publisher(Float64, f"{self.status_prefix}/{name}", 10)
            for name in (
                "left_close_ratio",
                "right_close_ratio",
                "center_error_ratio",
                "center_error_m",
                "left_current",
                "right_current",
                "left_current_baseline",
                "right_current_baseline",
                "left_current_residual",
                "right_current_residual",
                "left_contact_metric",
                "right_contact_metric",
                "single_contact_direction",
                "motion_profile_index",
                "motion_profile_speed",
                "motion_profile_acc",
                "motion_profile_torque_limit",
                "roll_deg",
                "pitch_deg",
            )
        }
        self.bool_status_pubs = {
            name: self.create_publisher(Bool, f"{self.status_prefix}/{name}", 10)
            for name in (
                "left_contact",
                "right_contact",
                "both_contact",
                "centered",
                "safe_to_lift",
                "fault",
                "single_contact_need_motion",
                "left_at_close_limit",
                "right_at_close_limit",
            )
        }
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._control_timer_cb)

        comp_label = str(comp_path) if comp_path is not None else "<none>"
        self.get_logger().info(
            "HLS gripper node started: "
            f"port={self.port} dry_run={self.dry_run} compensation={comp_label} "
            f"center_gain={self.center_gain_m_per_ratio:.5f}m/ratio "
            f"profile={self.motion_profile.profile_name}"
            f"({self.search_speed},{self.search_acc},{self.search_torque_limit}) "
            f"current_sign=({self.current_inward_sign[SIDE_LEFT]:+d},{self.current_inward_sign[SIDE_RIGHT]:+d}) "
            f"center_current=({self.center_hold_current},{self.center_push_current}) "
            f"grip_chase_min_current={self.grip_chase_min_current} "
            f"contact_enter=({self.contact_detection[SIDE_LEFT]['enter_threshold']:.1f},"
            f"{self.contact_detection[SIDE_RIGHT]['enter_threshold']:.1f}) "
            f"status_prefix={self.status_prefix}"
        )
        self._enter_state(STATE_OPEN)

    def _build_calibration(self) -> dict[str, ServoCalibration]:
        from_table = dict(self.compensation.calibration)

        left_table = from_table.get(SIDE_LEFT)
        right_table = from_table.get(SIDE_RIGHT)

        left_id = self.left_id if self.left_id >= 0 else (left_table.servo_id if left_table else 1)
        right_id = self.right_id if self.right_id >= 0 else (right_table.servo_id if right_table else 2)

        left_open = self.left_open if self.left_open >= 0 else (left_table.open_pos if left_table else -1)
        left_clear = self.left_clear if self.left_clear >= 0 else (left_table.clear_pos if left_table else -1)
        left_close = self.left_close if self.left_close >= 0 else (left_table.close_pos if left_table else -1)
        right_open = self.right_open if self.right_open >= 0 else (right_table.open_pos if right_table else -1)
        right_clear = self.right_clear if self.right_clear >= 0 else (right_table.clear_pos if right_table else -1)
        right_close = self.right_close if self.right_close >= 0 else (right_table.close_pos if right_table else -1)

        if left_clear < 0:
            left_clear = left_close
        if right_clear < 0:
            right_clear = right_close

        left_inward = self.left_inward_sign if self.left_inward_sign != 0 else (left_table.inward_sign if left_table else 1)
        right_inward = self.right_inward_sign if self.right_inward_sign != 0 else (right_table.inward_sign if right_table else -1)

        if min(left_open, left_clear, left_close, right_open, right_clear, right_close) < 0 or left_open == left_close or right_open == right_close:
            if not self.dry_run:
                raise RuntimeError(
                    "HLS open/clear/close limits are required. Provide gravity_comp_path generated by "
                    "gripper_gravity_calibration.py collect-full, or set open/clear/close parameters."
                )
            left_open, left_clear, left_close = 0, 700, 1000
            right_open, right_clear, right_close = 0, 700, 1000

        return {
            SIDE_LEFT: ServoCalibration(
                servo_id=left_id,
                open_pos=left_open,
                clear_pos=left_clear,
                close_pos=left_close,
                inward_sign=1 if left_inward >= 0 else -1,
            ),
            SIDE_RIGHT: ServoCalibration(
                servo_id=right_id,
                open_pos=right_open,
                clear_pos=right_clear,
                close_pos=right_close,
                inward_sign=1 if right_inward >= 0 else -1,
            ),
        }

    def _select_motion_profile(self) -> MotionProfile:
        profiles = list(self.compensation.motion_profiles)
        if not profiles:
            return MotionProfile(profile_index=0, profile_name="default", speed=10, acc=5, torque_limit=120)
        if self.motion_profile_name_param:
            for profile in profiles:
                if profile.profile_name == self.motion_profile_name_param:
                    return profile
            self.get_logger().warn(
                f"motion_profile={self.motion_profile_name_param} not found in compensation table; using first profile"
            )
        if self.motion_profile_index_param >= 0:
            for profile in profiles:
                if profile.profile_index == self.motion_profile_index_param:
                    return profile
            self.get_logger().warn(
                f"motion_profile_index={self.motion_profile_index_param} not found in compensation table; using first profile"
            )
        return profiles[0]

    def _profile_contact_detection_params(self) -> dict:
        keys = [
            f"{self.motion_profile.profile_index}:{self.motion_profile.profile_name}",
            f"{self.motion_profile.profile_index}:",
            f":{self.motion_profile.profile_name}",
        ]
        for key in keys:
            params = self.compensation.contact_detection_by_profile.get(key)
            if isinstance(params, dict):
                return params
        return {}

    def _build_contact_detection(self) -> dict[str, dict[str, float]]:
        result: dict[str, dict[str, float]] = {}
        profile_detection = self._profile_contact_detection_params()
        for side in (SIDE_LEFT, SIDE_RIGHT):
            table_params = profile_detection.get(side, {}) or self.compensation.contact_detection.get(side, {})
            cal = self.calibration[side]
            sign_override = (
                self.left_contact_metric_sign_param if side == SIDE_LEFT else self.right_contact_metric_sign_param
            )
            if sign_override != 0:
                metric_sign = 1.0 if sign_override > 0 else -1.0
            else:
                metric_sign = float_from_mapping(table_params, "metric_sign", float(cal.inward_sign))
                metric_sign = 1.0 if metric_sign >= 0.0 else -1.0

            enter = (
                self.contact_current_threshold
                if self.contact_current_threshold > 0.0
                else float_from_mapping(table_params, "enter_threshold", 35.0)
            )
            exit_threshold = (
                self.contact_exit_threshold
                if self.contact_exit_threshold > 0.0
                else float_from_mapping(table_params, "exit_threshold", max(6.0, 0.6 * enter))
            )
            strong = (
                self.contact_strong_threshold
                if self.contact_strong_threshold > 0.0
                else float_from_mapping(table_params, "strong_threshold", max(70.0, 1.8 * enter))
            )
            enter = max(enter, 1.0)
            exit_threshold = max(exit_threshold, 0.0)
            if exit_threshold >= enter:
                exit_threshold = 0.65 * enter
            strong = max(strong, enter)
            result[side] = {
                "metric_sign": metric_sign,
                "enter_threshold": enter,
                "exit_threshold": exit_threshold,
                "strong_threshold": strong,
            }
        return result

    def _build_current_inward_sign(self) -> dict[str, int]:
        result: dict[str, int] = {}
        for side in (SIDE_LEFT, SIDE_RIGHT):
            sign_override = (
                self.left_current_inward_sign_param if side == SIDE_LEFT else self.right_current_inward_sign_param
            )
            if sign_override != 0:
                sign = sign_override
            else:
                sign = int(self.contact_detection[side]["metric_sign"])
            result[side] = 1 if sign >= 0 else -1
        return result

    def _compute_center_gain(self) -> float:
        if self.center_gain_override > 0.0:
            return self.center_gain_override
        left = self.calibration[SIDE_LEFT]
        right = self.calibration[SIDE_RIGHT]
        left_delta = abs(left.close_pos - left.open_pos)
        right_delta = abs(right.close_pos - right.open_pos)
        delta_ticks = 0.5 * (left_delta + right_delta)
        delta_theta = delta_ticks * 2.0 * math.pi / max(self.servo_ticks_per_rev, 1.0)
        return 0.5 * self.finger_length_m * delta_theta

    def _initial_feedback(self) -> dict[str, ServoFeedback]:
        return {
            SIDE_LEFT: ServoFeedback(
                pos=float(self.calibration[SIDE_LEFT].open_pos),
                close_ratio=0.0,
                speed=0.0,
                load=0.0,
                voltage=0.0,
                temp=25.0,
                moving=0.0,
                current=0.0,
            ),
            SIDE_RIGHT: ServoFeedback(
                pos=float(self.calibration[SIDE_RIGHT].open_pos),
                close_ratio=0.0,
                speed=0.0,
                load=0.0,
                voltage=0.0,
                temp=25.0,
                moving=0.0,
                current=0.0,
            ),
        }

    def _imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        self.latest_roll_deg, self.latest_pitch_deg, self.latest_yaw_deg = quaternion_to_euler_deg(q.x, q.y, q.z, q.w)
        self.latest_attitude_s = time.monotonic()

    def _command_cb(self, msg: Float64) -> None:
        self._handle_command(CommandIntent(self._classify_scalar(float(msg.data)), float(msg.data)))

    def _command_pair_cb(self, msg: GripperCommandPair) -> None:
        left = clamp(float(msg.left_pos), 0.0, 100.0)
        right = clamp(float(msg.right_pos), 0.0, 100.0)
        if left >= self.open_enter_threshold or right >= self.open_enter_threshold:
            mode: Literal["open", "grasp", "hold"] = "open"
        elif left <= self.close_enter_threshold and right <= self.close_enter_threshold:
            mode = "grasp"
        else:
            mode = "hold"
        self._handle_command(CommandIntent(mode, 0.5 * (left + right)))

    def _classify_scalar(self, value: float) -> Literal["open", "grasp", "hold"]:
        value = clamp(value, 0.0, 100.0)
        if value >= self.open_enter_threshold:
            return "open"
        if value <= self.close_enter_threshold:
            return "grasp"
        return "hold"

    def _handle_command(self, intent: CommandIntent) -> None:
        self.last_command_s = time.monotonic()
        if intent.mode == "open":
            self.fault_requires_open_reset = False
            if self.state != STATE_OPEN:
                self.get_logger().warn(
                    f"received open command value={intent.value:.1f}; leaving state={self.state}"
                )
            self.fault_reason = ""
            self._enter_state(STATE_OPEN)
        elif intent.mode == "grasp":
            if self.state in (STATE_OPEN, STATE_FAULT):
                if self.state == STATE_FAULT and self.fault_requires_open_reset:
                    now = time.monotonic()
                    if now - self.last_fault_grasp_warn_s > 1.0:
                        self.last_fault_grasp_warn_s = now
                        self.get_logger().warn(
                            f"ignoring grasp command while fault requires open reset: {self.fault_reason}"
                        )
                    return
                self.get_logger().info(f"received grasp command value={intent.value:.1f}")
                self.fault_reason = ""
                self._enter_state(STATE_SEARCH_OBJECT)

    def _enter_state(self, state: str, reason: str = "") -> None:
        if state == self.state and state != STATE_FAULT:
            return
        self.state = state
        self.state_started_s = time.monotonic()
        self.centered_since_s = None
        if state in (STATE_OPEN, STATE_SEARCH_OBJECT, STATE_FAULT):
            self.left_contact_cycles = 0
            self.right_contact_cycles = 0
            self.left_contact = False
            self.right_contact = False
        if state == STATE_FAULT:
            self.fault_reason = reason or self.fault_reason or "fault"
            self.get_logger().error(f"HLS gripper fault: {self.fault_reason}")
        else:
            self.get_logger().info(f"HLS gripper state -> {state}")

    def _set_fault(self, reason: str) -> None:
        self.fault_requires_open_reset = True
        self._enter_state(STATE_FAULT, reason)

    def _control_timer_cb(self) -> None:
        try:
            self._update_feedback()
            self._check_safety()
            self._run_state_machine()
        except Exception as exc:
            self._set_fault(str(exc))
        finally:
            self._publish_feedback()
            self._publish_standard_status()

    def _update_feedback(self) -> None:
        if self.dry_run:
            self._update_dry_run_feedback()
        else:
            if self.bus is None:
                raise RuntimeError("HLS bus is not connected")
            self.feedback[SIDE_LEFT] = self.bus.read_feedback(self.calibration[SIDE_LEFT].servo_id, self.calibration[SIDE_LEFT])
            self.feedback[SIDE_RIGHT] = self.bus.read_feedback(self.calibration[SIDE_RIGHT].servo_id, self.calibration[SIDE_RIGHT])

        for side in (SIDE_LEFT, SIDE_RIGHT):
            fb = self.feedback[side]
            fb.baseline_current = self.compensation.baseline_current(
                side,
                fb.close_ratio,
                self.latest_roll_deg,
                self.latest_pitch_deg,
                profile_index=self.motion_profile.profile_index,
                profile_name=self.motion_profile.profile_name,
                direction=self._baseline_direction(),
            )
            fb.residual_current = fb.current - fb.baseline_current
        self.last_feedback_s = time.monotonic()

    def _update_dry_run_feedback(self) -> None:
        dt = 1.0 / max(self.control_rate_hz, 1.0)
        elapsed = time.monotonic() - self.state_started_s
        pattern = self.dry_run_contact_pattern.strip().lower()
        for side in (SIDE_LEFT, SIDE_RIGHT):
            fb = self.feedback[side]
            target_ratio = 0.0
            if self.state == STATE_SEARCH_OBJECT:
                target_ratio = 1.0
            elif self.state == STATE_LEFT_CONTACT:
                target_ratio = fb.close_ratio if side == SIDE_LEFT else 1.0
            elif self.state == STATE_RIGHT_CONTACT:
                target_ratio = 1.0 if side == SIDE_LEFT else fb.close_ratio
            elif self.state in (STATE_BOTH_CONTACT, STATE_CENTERING, STATE_CENTERED, STATE_FINAL_GRIP, STATE_LIFT_READY):
                target_ratio = 0.45
            fb.close_ratio += clamp(target_ratio - fb.close_ratio, -0.35 * dt, 0.35 * dt)
            fb.pos = ratio_to_pos(
                self.calibration[side].open_pos,
                self.calibration[side].close_pos,
                fb.close_ratio,
            )
            fb.speed = 0.0 if abs(target_ratio - fb.close_ratio) < 0.01 else 5.0
            if pattern == "none":
                contact_sim = False
            elif pattern == "left_only":
                contact_sim = (
                    side == SIDE_LEFT
                    and self.state != STATE_OPEN
                    and (elapsed > 0.8 or self.state != STATE_SEARCH_OBJECT)
                )
            elif pattern == "right_only":
                contact_sim = (
                    side == SIDE_RIGHT
                    and self.state != STATE_OPEN
                    and (elapsed > 0.8 or self.state != STATE_SEARCH_OBJECT)
                )
            else:
                contact_sim = self.state not in (STATE_OPEN, STATE_SEARCH_OBJECT) or elapsed > 0.8
            fb.current = self.contact_detection[side]["metric_sign"] * (
                self.low_current + 20.0 if contact_sim else 0.0
            )
            fb.load = fb.current
            fb.temp = 25.0

    def _check_safety(self) -> None:
        now = time.monotonic()
        if self.last_feedback_s is not None and now - self.last_feedback_s > self.feedback_timeout_s:
            raise RuntimeError("feedback timeout")
        if (
            self.latest_attitude_s is not None
            and self.attitude_timeout_s > 0.0
            and now - self.latest_attitude_s > self.attitude_timeout_s
        ):
            raise RuntimeError("attitude timeout")
        if (
            self.command_timeout_s > 0.0
            and self.last_command_s is not None
            and now - self.last_command_s > self.command_timeout_s
            and self.state != STATE_OPEN
        ):
            raise RuntimeError("command timeout")

        for side in (SIDE_LEFT, SIDE_RIGHT):
            fb = self.feedback[side]
            if self.max_current > 0.0 and abs(fb.current) > self.max_current:
                raise RuntimeError(f"{side} current limit reached: {fb.current:.0f}")
            if self.max_temp > 0.0 and fb.temp >= self.max_temp:
                raise RuntimeError(f"{side} temperature limit reached: {fb.temp:.0f}")

    def _run_state_machine(self) -> None:
        now = time.monotonic()
        if self.state == STATE_OPEN:
            self._write_open_if_due(now)
            return
        if self.state == STATE_FAULT:
            self._write_open_if_due(now)
            return
        if self.state == STATE_SEARCH_OBJECT:
            if now - self.state_started_s > self.search_timeout_s:
                self._set_fault("search timeout")
                return
            self._update_contacts()
            self._write_search_commands_if_due(now)
            if self.left_contact and self.right_contact:
                self._enter_state(STATE_BOTH_CONTACT)
            elif self.left_contact:
                self._enter_state(STATE_LEFT_CONTACT)
            elif self.right_contact:
                self._enter_state(STATE_RIGHT_CONTACT)
            return
        if self.state in (STATE_LEFT_CONTACT, STATE_RIGHT_CONTACT):
            if now - self.state_started_s > self.single_contact_timeout_s:
                self._set_fault("single contact timeout")
                return
            self._update_contacts()
            self._write_asymmetric_contact_commands(now)
            if self.left_contact and self.right_contact:
                self._enter_state(STATE_BOTH_CONTACT)
            return
        if self.state == STATE_BOTH_CONTACT:
            self._write_low_current_if_due(now)
            self._enter_state(STATE_CENTERING)
            return
        if self.state == STATE_CENTERING:
            self._update_contacts()
            if now - self.state_started_s > self.center_timeout_s:
                if self.center_timeout_action == "fault":
                    self._set_fault("centering timeout")
                else:
                    self.get_logger().warn(
                        "centering timeout; keeping inward force and continuing to final grip"
                    )
                    self._enter_state(STATE_FINAL_GRIP)
                return
            self._write_grip_chase_commands_if_due(
                now,
                self.center_hold_current,
                self.center_push_current,
            )
            if self._is_centered(now):
                self._enter_state(STATE_CENTERED)
            return
        if self.state == STATE_CENTERED:
            self._update_contacts()
            if not (self.left_contact and self.right_contact):
                self._enter_state(STATE_CENTERING)
                self._write_grip_chase_commands_if_due(
                    now,
                    self.center_hold_current,
                    self.center_push_current,
                )
                return
            self._write_grip_chase_commands_if_due(
                now,
                self.center_hold_current,
                self.center_push_current,
            )
            if now - self.state_started_s >= 0.2:
                self._enter_state(STATE_FINAL_GRIP)
            return
        if self.state == STATE_FINAL_GRIP:
            if now - self.state_started_s > max(self.final_grip_timeout_s, self.final_grip_ramp_s + 0.5):
                self._set_fault("final grip timeout")
                return
            self._update_contacts()
            self._write_final_grip_current(now)
            if now - self.state_started_s >= self.final_grip_ramp_s:
                self._enter_state(STATE_LIFT_READY)
            return
        if self.state == STATE_LIFT_READY:
            self._update_contacts()
            self._write_lift_current_if_due(now)

    def _write_open_if_due(self, now: float) -> None:
        if now - self.last_open_write_s < 0.25:
            return
        self.last_open_write_s = now
        self.goal_left_pos = 100.0
        self.goal_right_pos = 100.0
        if self.dry_run:
            return
        assert self.bus is not None
        for side in (SIDE_LEFT, SIDE_RIGHT):
            cal = self.calibration[side]
            self.bus.write_position(cal.servo_id, cal.open_pos, self.open_speed, self.open_acc, self.open_torque_limit)

    def _write_search_commands_if_due(self, now: float) -> None:
        if now - self.last_search_write_s < 0.15:
            return
        self.last_search_write_s = now
        self.goal_left_pos = 0.0
        self.goal_right_pos = 0.0
        if self.dry_run:
            return
        assert self.bus is not None
        for side in (SIDE_LEFT, SIDE_RIGHT):
            cal = self.calibration[side]
            self.bus.write_position(cal.servo_id, cal.close_pos, self.search_speed, self.search_acc, self.search_torque_limit)

    def _write_asymmetric_contact_commands(self, now: float) -> None:
        if now - min(self.last_ele_write_s, self.last_search_write_s) < 0.10:
            return
        if self.dry_run:
            return
        assert self.bus is not None
        if self.left_contact:
            cal = self.calibration[SIDE_LEFT]
            self.bus.write_current(cal.servo_id, self.current_inward_sign[SIDE_LEFT] * self.low_current)
        else:
            cal = self.calibration[SIDE_LEFT]
            self.bus.write_position(cal.servo_id, cal.close_pos, self.search_speed, self.search_acc, self.search_torque_limit)

        if self.right_contact:
            cal = self.calibration[SIDE_RIGHT]
            self.bus.write_current(cal.servo_id, self.current_inward_sign[SIDE_RIGHT] * self.low_current)
        else:
            cal = self.calibration[SIDE_RIGHT]
            self.bus.write_position(cal.servo_id, cal.close_pos, self.search_speed, self.search_acc, self.search_torque_limit)
        self.last_ele_write_s = now
        self.last_search_write_s = now

    def _write_low_current_if_due(self, now: float) -> None:
        if now - self.last_ele_write_s < 0.08:
            return
        self.last_ele_write_s = now
        self.goal_left_pos = self._compat_pos(self.feedback[SIDE_LEFT].close_ratio)
        self.goal_right_pos = self._compat_pos(self.feedback[SIDE_RIGHT].close_ratio)
        if self.dry_run:
            return
        assert self.bus is not None
        for side in (SIDE_LEFT, SIDE_RIGHT):
            cal = self.calibration[side]
            self.bus.write_current(cal.servo_id, self.current_inward_sign[side] * self.low_current)

    def _write_centering_current_if_due(self, now: float) -> None:
        if now - self.last_ele_write_s < 0.08:
            return
        self.last_ele_write_s = now
        self.goal_left_pos = self._compat_pos(self.feedback[SIDE_LEFT].close_ratio)
        self.goal_right_pos = self._compat_pos(self.feedback[SIDE_RIGHT].close_ratio)

        left_current, right_current = self._differential_current_pair(
            self.center_hold_current,
            self.center_push_current,
        )

        if self.dry_run:
            return
        assert self.bus is not None
        self._write_signed_current_pair(left_current, right_current)

    def _write_final_grip_current(self, now: float) -> None:
        alpha = clamp((now - self.state_started_s) / max(self.final_grip_ramp_s, 1e-3), 0.0, 1.0)
        current = int(round(self.low_current + (self.lift_current - self.low_current) * alpha))
        push_current = max(current, self.center_push_current)
        self._write_grip_chase_commands_if_due(now, current, push_current)

    def _write_lift_current_if_due(self, now: float) -> None:
        hold_current = max(self.center_hold_current, int(round(0.65 * self.lift_current)))
        self._write_grip_chase_commands_if_due(now, hold_current, self.lift_current)

    def _write_current_pair(self, current: int, now: float, push_current: int | None = None) -> None:
        if now - self.last_ele_write_s < 0.08:
            return
        self.last_ele_write_s = now
        left_current, right_current = self._differential_current_pair(
            current,
            push_current if push_current is not None else current,
        )
        if self.dry_run:
            return
        assert self.bus is not None
        self._write_signed_current_pair(left_current, right_current)

    def _write_grip_chase_commands_if_due(self, now: float, hold_current: int, push_current: int) -> None:
        if now - min(self.last_ele_write_s, self.last_search_write_s) < 0.08:
            return
        self.last_ele_write_s = now
        self.last_search_write_s = now

        left_current, right_current = self._differential_current_pair(hold_current, push_current)
        chase_floor = max(hold_current, min(push_current, self.grip_chase_min_current))
        left_current = max(left_current, chase_floor)
        right_current = max(right_current, chase_floor)
        if not self.left_contact:
            left_current = max(left_current, push_current)
        if not self.right_contact:
            right_current = max(right_current, push_current)

        self.goal_left_pos = self._compat_pos(self.feedback[SIDE_LEFT].close_ratio)
        self.goal_right_pos = self._compat_pos(self.feedback[SIDE_RIGHT].close_ratio)

        if not self.left_contact:
            self.goal_left_pos = 0.0
        if not self.right_contact:
            self.goal_right_pos = 0.0

        if self.dry_run:
            return
        assert self.bus is not None

        self._write_grip_chase_side(SIDE_LEFT, left_current)
        self._write_grip_chase_side(SIDE_RIGHT, right_current)

    def _write_grip_chase_side(self, side: str, current: int) -> None:
        cal = self.calibration[side]
        self.bus.write_current(cal.servo_id, self.current_inward_sign[side] * current)

    def _differential_current_pair(self, hold_current: int, push_current: int) -> tuple[int, int]:
        hold_current = max(0, int(hold_current))
        push_current = max(hold_current, int(push_current))
        left_current = hold_current
        right_current = hold_current

        error_ratio = self._center_error_ratio()
        deadband_ratio = self.center_deadband_m / max(self.center_gain_m_per_ratio, 1e-6)
        if error_ratio > deadband_ratio:
            right_current = push_current
        elif error_ratio < -deadband_ratio:
            left_current = push_current
        return left_current, right_current

    def _write_signed_current_pair(self, left_current: int, right_current: int) -> None:
        for side in (SIDE_LEFT, SIDE_RIGHT):
            cal = self.calibration[side]
            current = left_current if side == SIDE_LEFT else right_current
            self.bus.write_current(cal.servo_id, self.current_inward_sign[side] * current)

    def _update_contacts(self) -> None:
        left_metric = self._contact_metric(SIDE_LEFT)
        right_metric = self._contact_metric(SIDE_RIGHT)
        self.left_contact_cycles, self.left_contact = self._update_one_contact(
            SIDE_LEFT,
            left_metric,
            self.left_contact_cycles,
            self.left_contact,
        )
        self.right_contact_cycles, self.right_contact = self._update_one_contact(
            SIDE_RIGHT,
            right_metric,
            self.right_contact_cycles,
            self.right_contact,
        )

    def _update_one_contact(self, side: str, metric: float, cycles: int, is_contact: bool) -> tuple[int, bool]:
        thresholds = self.contact_detection[side]
        confirm_cycles = max(1, self.contact_confirm_cycles)
        if is_contact:
            if metric <= thresholds["exit_threshold"]:
                cycles = max(0, cycles - 1)
            else:
                cycles = max(cycles, confirm_cycles)
            return cycles, cycles > 0
        if metric >= thresholds["strong_threshold"]:
            cycles = confirm_cycles
        elif metric >= thresholds["enter_threshold"]:
            cycles += 1
        else:
            cycles = 0
        return cycles, cycles >= confirm_cycles

    def _contact_metric(self, side: str) -> float:
        return self.contact_detection[side]["metric_sign"] * self.feedback[side].residual_current

    def _baseline_direction(self) -> str:
        if self.state in (STATE_OPEN, STATE_FAULT):
            return "opening"
        return "closing"

    def _at_close_limit(self, side: str) -> bool:
        return self.feedback[side].close_ratio >= self.single_contact_limit_ratio

    def _single_contact_need_motion(self) -> bool:
        if self.state == STATE_LEFT_CONTACT and self.left_contact and not self.right_contact:
            return self._at_close_limit(SIDE_RIGHT)
        if self.state == STATE_RIGHT_CONTACT and self.right_contact and not self.left_contact:
            return self._at_close_limit(SIDE_LEFT)
        return False

    def _single_contact_direction(self) -> float:
        if not self._single_contact_need_motion():
            return 0.0
        if self.state == STATE_LEFT_CONTACT:
            return 1.0
        if self.state == STATE_RIGHT_CONTACT:
            return -1.0
        return 0.0

    def _center_error_ratio(self) -> float:
        return self.feedback[SIDE_LEFT].close_ratio - self.feedback[SIDE_RIGHT].close_ratio - self.center_bias

    def _center_error_m(self) -> float:
        return self.center_sign * self.center_gain_m_per_ratio * self._center_error_ratio()

    def _is_centered(self, now: float) -> bool:
        if abs(self._center_error_m()) <= self.center_deadband_m and self.left_contact and self.right_contact:
            if self.centered_since_s is None:
                self.centered_since_s = now
            return now - self.centered_since_s >= self.center_stable_time_s
        self.centered_since_s = None
        return False

    def _compat_pos(self, close_ratio: float) -> float:
        return clamp((1.0 - close_ratio) * 100.0, 0.0, 100.0)

    def _publish_feedback(self) -> None:
        msg = GripperFeedback()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gripper"
        left_fb = self.feedback[SIDE_LEFT]
        right_fb = self.feedback[SIDE_RIGHT]
        msg.left_pos = self._compat_pos(left_fb.close_ratio)
        msg.right_pos = self._compat_pos(right_fb.close_ratio)
        msg.left_load = left_fb.load
        msg.right_load = right_fb.load
        msg.left_current = left_fb.current
        msg.right_current = right_fb.current
        msg.left_position_error = self.goal_left_pos - msg.left_pos
        msg.right_position_error = self.goal_right_pos - msg.right_pos
        msg.left_goal_pos = self.goal_left_pos
        msg.right_goal_pos = self.goal_right_pos
        self.feedback_pub.publish(msg)

    def _publish_standard_status(self) -> None:
        left_fb = self.feedback[SIDE_LEFT]
        right_fb = self.feedback[SIDE_RIGHT]
        string_msg = String()
        string_msg.data = self.state
        self.state_pub.publish(string_msg)

        string_msg = String()
        string_msg.data = self.fault_reason if self.state == STATE_FAULT else ""
        self.fault_reason_pub.publish(string_msg)

        string_msg = String()
        string_msg.data = self.motion_profile.profile_name
        self.motion_profile_pub.publish(string_msg)

        values = {
            "left_close_ratio": left_fb.close_ratio,
            "right_close_ratio": right_fb.close_ratio,
            "center_error_ratio": self._center_error_ratio(),
            "center_error_m": self._center_error_m(),
            "left_current": left_fb.current,
            "right_current": right_fb.current,
            "left_current_baseline": left_fb.baseline_current,
            "right_current_baseline": right_fb.baseline_current,
            "left_current_residual": left_fb.residual_current,
            "right_current_residual": right_fb.residual_current,
            "left_contact_metric": self._contact_metric(SIDE_LEFT),
            "right_contact_metric": self._contact_metric(SIDE_RIGHT),
            "single_contact_direction": self._single_contact_direction(),
            "motion_profile_index": float(self.motion_profile.profile_index),
            "motion_profile_speed": float(self.search_speed),
            "motion_profile_acc": float(self.search_acc),
            "motion_profile_torque_limit": float(self.search_torque_limit),
            "roll_deg": self.latest_roll_deg,
            "pitch_deg": self.latest_pitch_deg,
        }
        for name, value in values.items():
            msg = Float64()
            msg.data = float(value)
            self.float_status_pubs[name].publish(msg)

        flags = {
            "left_contact": bool(self.left_contact),
            "right_contact": bool(self.right_contact),
            "both_contact": bool(self.left_contact and self.right_contact),
            "centered": self.state in (STATE_CENTERED, STATE_FINAL_GRIP, STATE_LIFT_READY)
            and bool(self.left_contact and self.right_contact),
            "safe_to_lift": self.state == STATE_LIFT_READY and bool(self.left_contact and self.right_contact),
            "fault": self.state == STATE_FAULT,
            "single_contact_need_motion": self._single_contact_need_motion(),
            "left_at_close_limit": self._at_close_limit(SIDE_LEFT),
            "right_at_close_limit": self._at_close_limit(SIDE_RIGHT),
        }
        for name, value in flags.items():
            msg = Bool()
            msg.data = bool(value)
            self.bool_status_pubs[name].publish(msg)

    def destroy_node(self) -> bool:
        try:
            if self.bus is not None:
                for side in (SIDE_LEFT, SIDE_RIGHT):
                    cal = self.calibration[side]
                    try:
                        self.bus.write_position(cal.servo_id, cal.open_pos, self.open_speed, self.open_acc, self.open_torque_limit)
                    except Exception as exc:
                        self.get_logger().warn(f"failed to open {side} during shutdown: {exc}")
                self.bus.close()
        finally:
            return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = HlsGripperNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
