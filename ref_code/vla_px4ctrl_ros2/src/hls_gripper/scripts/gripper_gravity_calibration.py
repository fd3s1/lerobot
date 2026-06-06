#!/usr/bin/env python3
#
# Build a no-load gravity/friction compensation table for a two-servo HLS gripper.
#
# Typical workflow:
#   1. Remove the object from the gripper and secure the aircraft.
#   2. Run `collect` at one or more aircraft attitudes.
#   3. Run `fit` on the collected CSV to produce a JSON table.
#   4. The flight gripper controller reads the JSON table and interpolates
#      current_baseline(close_ratio, roll_deg, pitch_deg).

import argparse
import copy
import csv
import importlib
import json
import math
import os
import statistics
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
DEFAULT_LIMITS_PATH = PACKAGE_DIR / "config" / "hls_gripper_limits.json"
DEFAULT_RAW_DIR = PACKAGE_DIR / "config" / "calibration" / "raw"
DEFAULT_SESSION_EMPTY_CSV = DEFAULT_RAW_DIR / "gravity_empty_session.csv"
DEFAULT_SESSION_CONTACT_CSV = DEFAULT_RAW_DIR / "gravity_contact_session.csv"
DEFAULT_SESSION_JSON = PACKAGE_DIR / "config" / "gravity_compensation.json"
DEFAULT_SESSION_CURVE_CSV = PACKAGE_DIR / "config" / "gravity_compensation_curve.csv"

SDK_LOADED = False
COMM_SUCCESS = None
PortHandler = None
hls = None
HLS_PRESENT_CURRENT_H = None
HLS_PRESENT_POSITION_L = None
HLS_PRESENT_POSITION_H = None
HLS_PRESENT_SPEED_L = None
HLS_PRESENT_LOAD_L = None
HLS_PRESENT_VOLTAGE = None
HLS_PRESENT_TEMPERATURE = None
HLS_MOVING = None
HLS_PRESENT_CURRENT_L = None
HLS_MODEL_L = None
HLS_ID = None
HLS_BAUD_RATE = None
HLS_MIN_ANGLE_LIMIT_L = None
HLS_MAX_ANGLE_LIMIT_L = None
HLS_MODE = None


SCHEMA_VERSION = 1
DEFAULT_FIELDS = [
    "schema_version",
    "utc_time",
    "monotonic_s",
    "cycle",
    "direction",
    "segment",
    "session_phase",
    "profile_index",
    "profile_name",
    "speed_cmd",
    "acc_cmd",
    "torque_limit",
    "object_label",
    "trial_kind",
    "trial_index",
    "contact_phase",
    "target_ratio",
    "sample_index",
    "left_id",
    "right_id",
    "left_open",
    "left_clear",
    "left_close",
    "right_open",
    "right_clear",
    "right_close",
    "left_inward_sign",
    "right_inward_sign",
    "left_active",
    "right_active",
    "target_left_pos",
    "target_right_pos",
    "attitude_source",
    "attitude_age_s",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "left_pos",
    "left_close_ratio",
    "left_speed",
    "left_load",
    "left_voltage",
    "left_temp",
    "left_moving",
    "left_current",
    "left_current_baseline",
    "left_current_residual",
    "right_pos",
    "right_close_ratio",
    "right_speed",
    "right_load",
    "right_voltage",
    "right_temp",
    "right_moving",
    "right_current",
    "right_current_baseline",
    "right_current_residual",
]


def default_sdk_candidates():
    candidates = []
    ref_code_dir = PACKAGE_DIR.parent.parent.parent
    candidates.append(ref_code_dir / "FT-servo" / "FTServo_Python-main" / "FTServo_Python-main")
    candidates.append(Path.cwd() / "lerobot" / "ref_code" / "FT-servo" / "FTServo_Python-main" / "FTServo_Python-main")
    candidates.append(Path.cwd() / "FT-servo" / "FTServo_Python-main" / "FTServo_Python-main")
    return candidates


def load_sdk(sdk_root=None):
    global SDK_LOADED
    global COMM_SUCCESS
    global PortHandler
    global hls
    global HLS_PRESENT_CURRENT_H
    global HLS_PRESENT_POSITION_L
    global HLS_PRESENT_POSITION_H
    global HLS_PRESENT_SPEED_L
    global HLS_PRESENT_LOAD_L
    global HLS_PRESENT_VOLTAGE
    global HLS_PRESENT_TEMPERATURE
    global HLS_MOVING
    global HLS_PRESENT_CURRENT_L
    global HLS_MODEL_L
    global HLS_ID
    global HLS_BAUD_RATE
    global HLS_MIN_ANGLE_LIMIT_L
    global HLS_MAX_ANGLE_LIMIT_L
    global HLS_MODE

    if SDK_LOADED:
        return
    sdk_paths = []
    if sdk_root:
        sdk_paths.append(Path(sdk_root).expanduser())
    sdk_paths.extend(default_sdk_candidates())
    for sdk_path in sdk_paths:
        if (sdk_path / "scservo_sdk").is_dir():
            sys.path.insert(0, str(sdk_path))
            break
    try:
        sdk = importlib.import_module("scservo_sdk")
    except ImportError as exc:
        raise RuntimeError(
            "collect requires the FTServo SDK and pyserial. Pass --sdk-root if the SDK "
            "is not in lerobot/ref_code/FT-servo/FTServo_Python-main/FTServo_Python-main. "
            "You can still run fit on an existing CSV without hardware access. "
            "Import error: %s" % exc
        )

    COMM_SUCCESS = sdk.COMM_SUCCESS
    PortHandler = sdk.PortHandler
    hls = sdk.hls
    HLS_PRESENT_CURRENT_H = sdk.HLS_PRESENT_CURRENT_H
    HLS_PRESENT_POSITION_L = sdk.HLS_PRESENT_POSITION_L
    HLS_PRESENT_POSITION_H = sdk.HLS_PRESENT_POSITION_H
    HLS_PRESENT_SPEED_L = sdk.HLS_PRESENT_SPEED_L
    HLS_PRESENT_LOAD_L = sdk.HLS_PRESENT_LOAD_L
    HLS_PRESENT_VOLTAGE = sdk.HLS_PRESENT_VOLTAGE
    HLS_PRESENT_TEMPERATURE = sdk.HLS_PRESENT_TEMPERATURE
    HLS_MOVING = sdk.HLS_MOVING
    HLS_PRESENT_CURRENT_L = sdk.HLS_PRESENT_CURRENT_L
    HLS_MODEL_L = sdk.HLS_MODEL_L
    HLS_ID = sdk.HLS_ID
    HLS_BAUD_RATE = sdk.HLS_BAUD_RATE
    HLS_MIN_ANGLE_LIMIT_L = sdk.HLS_MIN_ANGLE_LIMIT_L
    HLS_MAX_ANGLE_LIMIT_L = sdk.HLS_MAX_ANGLE_LIMIT_L
    HLS_MODE = sdk.HLS_MODE
    SDK_LOADED = True


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def finite_float(value, default=float("nan")):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if math.isfinite(result):
        return result
    return default


def ratio_to_pos(open_pos, close_pos, ratio):
    return int(round(open_pos + (close_pos - open_pos) * ratio))


def pos_to_ratio(open_pos, close_pos, pos):
    span = close_pos - open_pos
    if span == 0:
        return float("nan")
    return (pos - open_pos) / float(span)


def clamp(value, low, high):
    return max(low, min(high, value))


def check(packet, result, error, label):
    if result != COMM_SUCCESS:
        raise RuntimeError("%s: %s" % (label, packet.getTxRxResult(result)))
    if error:
        raise RuntimeError("%s: %s" % (label, packet.getRxPacketError(error)))


def read_mode(packet, servo_id):
    mode, result, error = packet.read1ByteTxRx(servo_id, HLS_MODE)
    check(packet, result, error, "read mode id=%s" % servo_id)
    return mode


def write_mode_checked(packet, servo_id, mode_name, mode_fn, expected_mode):
    result, error = packet.EnableTorque(servo_id, 0)
    check(packet, result, error, "disable torque id=%s" % servo_id)

    result, error = mode_fn(servo_id)
    check(packet, result, error, "set %s id=%s" % (mode_name, servo_id))
    time.sleep(0.05)
    mode = read_mode(packet, servo_id)

    if mode != expected_mode:
        print(
            "id=%s mode readback is %s after %s; retrying with EEPROM unlock"
            % (servo_id, mode, mode_name)
        )
        result, error = packet.unLockEprom(servo_id)
        check(packet, result, error, "unlock EPROM id=%s" % servo_id)
        time.sleep(0.02)
        result, error = mode_fn(servo_id)
        check(packet, result, error, "set %s after unlock id=%s" % (mode_name, servo_id))
        time.sleep(0.05)
        result, error = packet.LockEprom(servo_id)
        check(packet, result, error, "lock EPROM id=%s" % servo_id)
        mode = read_mode(packet, servo_id)

    if mode != expected_mode:
        raise RuntimeError(
            "failed to switch id=%s to %s: mode readback=%s expected=%s"
            % (servo_id, mode_name, mode, expected_mode)
        )

    result, error = packet.EnableTorque(servo_id, 1)
    check(packet, result, error, "enable torque id=%s" % servo_id)
    print("id=%s mode=%s (%s)" % (servo_id, mode, mode_name))


def read_feedback(packet, servo_id):
    length = HLS_PRESENT_CURRENT_H - HLS_PRESENT_POSITION_L + 1
    data, result, error = packet.readTxRx(servo_id, HLS_PRESENT_POSITION_L, length)
    check(packet, result, error, "read feedback id=%s" % servo_id)

    def word(addr):
        off = addr - HLS_PRESENT_POSITION_L
        return packet.scs_makeword(data[off], data[off + 1])

    return {
        "pos": packet.scs_tohost(word(HLS_PRESENT_POSITION_L), 15),
        "speed": packet.scs_tohost(word(HLS_PRESENT_SPEED_L), 15),
        "load": packet.scs_tohost(word(HLS_PRESENT_LOAD_L), 10),
        "voltage": data[HLS_PRESENT_VOLTAGE - HLS_PRESENT_POSITION_L],
        "temp": data[HLS_PRESENT_TEMPERATURE - HLS_PRESENT_POSITION_L],
        "moving": data[HLS_MOVING - HLS_PRESENT_POSITION_L],
        "current": packet.scs_tohost(word(HLS_PRESENT_CURRENT_L), 15),
    }


def write_position(packet, servo_id, position, speed, acc, torque_limit):
    result, error = packet.WritePosEx(servo_id, position, speed, acc, torque_limit)
    check(packet, result, error, "write position id=%s" % servo_id)


def setup_servo_position_mode(packet, servo_id):
    model, result, error = packet.ping(servo_id)
    check(packet, result, error, "ping id=%s" % servo_id)
    print("id=%s model=%s" % (servo_id, model))
    write_mode_checked(packet, servo_id, "ServoMode", packet.ServoMode, 0)


def safety_check(args, left_fb, right_fb):
    if args.max_current:
        if abs(left_fb["current"]) > args.max_current:
            raise RuntimeError("left current limit reached: %s" % left_fb["current"])
        if abs(right_fb["current"]) > args.max_current:
            raise RuntimeError("right current limit reached: %s" % right_fb["current"])
    if left_fb["temp"] >= args.max_temp:
        raise RuntimeError("left temperature limit reached: %s" % left_fb["temp"])
    if right_fb["temp"] >= args.max_temp:
        raise RuntimeError("right temperature limit reached: %s" % right_fb["temp"])


def quaternion_to_euler_deg(x, y, z, w):
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


class AttitudeReader(object):
    def __init__(self, source, topic, roll_deg, pitch_deg, yaw_deg):
        self.source = source
        self.topic = topic
        self.manual = (roll_deg, pitch_deg, yaw_deg)
        self.latest = None
        self.node = None
        self.rclpy = None
        self.thread = None
        self.stop_event = threading.Event()

    def start(self):
        if self.source == "manual":
            self.latest = (self.manual[0], self.manual[1], self.manual[2], time.monotonic())
            return
        if self.source == "none":
            self.latest = (float("nan"), float("nan"), float("nan"), time.monotonic())
            return
        if self.source not in ("ros-imu", "ros-pose"):
            raise RuntimeError("unknown attitude source: %s" % self.source)

        try:
            import rclpy
            from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
            if self.source == "ros-imu":
                from sensor_msgs.msg import Imu as MessageType
            else:
                from geometry_msgs.msg import PoseStamped as MessageType
        except ImportError as exc:
            raise RuntimeError(
                "ROS2 attitude source requested but ROS2 Python modules are unavailable: %s" % exc
            )

        self.rclpy = rclpy
        rclpy.init(args=None)
        self.node = rclpy.create_node("hls_gripper_gravity_calibration_attitude")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.node.create_subscription(MessageType, self.topic, self._ros_callback, qos)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _ros_callback(self, msg):
        if self.source == "ros-imu":
            q = msg.orientation
        else:
            q = msg.pose.orientation
        roll, pitch, yaw = quaternion_to_euler_deg(q.x, q.y, q.z, q.w)
        self.latest = (roll, pitch, yaw, time.monotonic())

    def _spin(self):
        while not self.stop_event.is_set():
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def read(self):
        if self.latest is None:
            return {
                "roll_deg": float("nan"),
                "pitch_deg": float("nan"),
                "yaw_deg": float("nan"),
                "attitude_age_s": float("nan"),
            }
        roll, pitch, yaw, stamp = self.latest
        return {
            "roll_deg": roll,
            "pitch_deg": pitch,
            "yaw_deg": yaw,
            "attitude_age_s": time.monotonic() - stamp,
        }

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.node:
            self.node.destroy_node()
        if self.rclpy:
            self.rclpy.shutdown()


def make_ratio_sequence(points, cycle):
    if points < 2:
        return [0.0]
    ratios = [i / float(points - 1) for i in range(points)]
    if cycle % 2:
        ratios.reverse()
    return ratios


def make_ratio_sequence_between(start, stop, points, cycle):
    if points < 2:
        ratios = [start]
    else:
        ratios = [start + (stop - start) * i / float(points - 1) for i in range(points)]
    if cycle % 2:
        ratios.reverse()
    return ratios


def parse_profiles(text, name_prefix="p"):
    profiles = []
    for index, chunk in enumerate(text.split(",")):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = chunk.split(":")
        if len(parts) != 3:
            raise RuntimeError(
                "invalid profile '%s'; expected speed:acc:torque_limit, for example 5:3:90"
                % chunk
            )
        try:
            speed = int(parts[0])
            acc = int(parts[1])
            torque_limit = int(parts[2])
        except ValueError as exc:
            raise RuntimeError(
                "invalid profile '%s'; speed, acc, and torque_limit must be integers" % chunk
            ) from exc
        if speed <= 0 or acc <= 0 or torque_limit <= 0:
            raise RuntimeError("invalid profile '%s'; all values must be positive" % chunk)
        profiles.append(
            {
                "profile_index": index,
                "profile_name": "%s%s" % (name_prefix, index),
                "speed": speed,
                "acc": acc,
                "torque_limit": torque_limit,
                "label": "%s:%s:%s" % (speed, acc, torque_limit),
            }
        )
    if not profiles:
        raise RuntimeError("at least one profile is required")
    return profiles


def parse_opening_profiles(args):
    text = getattr(args, "opening_profiles", "").strip()
    if text:
        return parse_profiles(text, name_prefix="o")
    return [
        {
            "profile_index": 0,
            "profile_name": "o0",
            "speed": int(getattr(args, "open_speed", getattr(args, "inter_segment_open_speed", 40))),
            "acc": int(getattr(args, "open_acc", getattr(args, "inter_segment_open_acc", 10))),
            "torque_limit": int(
                getattr(args, "open_torque_limit", getattr(args, "inter_segment_open_torque_limit", 300))
            ),
            "label": "%s:%s:%s"
            % (
                int(getattr(args, "open_speed", getattr(args, "inter_segment_open_speed", 40))),
                int(getattr(args, "open_acc", getattr(args, "inter_segment_open_acc", 10))),
                int(getattr(args, "open_torque_limit", getattr(args, "inter_segment_open_torque_limit", 300))),
            ),
        }
    ]


def parse_object_labels(text):
    return [item.strip() for item in text.split(",") if item.strip()]


def ensure_parent(path):
    parent = Path(path).expanduser().resolve().parent
    parent.mkdir(parents=True, exist_ok=True)


def collect(args):
    load_sdk(args.sdk_root)
    ensure_parent(args.output)

    attitude = AttitudeReader(
        args.attitude_source,
        args.attitude_topic,
        args.roll_deg,
        args.pitch_deg,
        args.yaw_deg,
    )
    attitude.start()

    port = PortHandler(args.port)
    packet = hls(port)

    if not port.openPort():
        raise RuntimeError("failed to open port %s" % args.port)
    if not port.setBaudRate(args.baud):
        raise RuntimeError("failed to set baudrate %s" % args.baud)

    try:
        setup_servo_position_mode(packet, args.left_id)
        setup_servo_position_mode(packet, args.right_id)

        with open(args.output, "w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=DEFAULT_FIELDS)
            writer.writeheader()

            sample_index = 0
            for cycle in range(args.cycles):
                ratios = make_ratio_sequence(args.sweep_points, cycle)
                direction = "closing" if cycle % 2 == 0 else "opening"
                for target_ratio in ratios:
                    target_left = ratio_to_pos(args.left_open, args.left_close, target_ratio)
                    target_right = ratio_to_pos(args.right_open, args.right_close, target_ratio)
                    write_position(packet, args.left_id, target_left, args.speed, args.acc, args.torque_limit)
                    write_position(packet, args.right_id, target_right, args.speed, args.acc, args.torque_limit)
                    time.sleep(args.settle_time)

                    samples_here = max(1, int(round(args.sample_duration * args.sample_hz)))
                    period = 1.0 / args.sample_hz
                    for _ in range(samples_here):
                        left_fb = read_feedback(packet, args.left_id)
                        right_fb = read_feedback(packet, args.right_id)
                        safety_check(args, left_fb, right_fb)
                        att = attitude.read()
                        row = {
                            "schema_version": SCHEMA_VERSION,
                            "utc_time": utc_now(),
                            "monotonic_s": "%.6f" % time.monotonic(),
                            "cycle": cycle,
                            "direction": direction,
                            "segment": "both",
                            "target_ratio": "%.6f" % target_ratio,
                            "sample_index": sample_index,
                            "left_id": args.left_id,
                            "right_id": args.right_id,
                            "left_open": args.left_open,
                            "left_clear": "",
                            "left_close": args.left_close,
                            "right_open": args.right_open,
                            "right_clear": "",
                            "right_close": args.right_close,
                            "left_inward_sign": args.left_inward_sign,
                            "right_inward_sign": args.right_inward_sign,
                            "left_active": 1,
                            "right_active": 1,
                            "target_left_pos": target_left,
                            "target_right_pos": target_right,
                            "attitude_source": args.attitude_source,
                            "attitude_age_s": format_float(att["attitude_age_s"]),
                            "roll_deg": format_float(att["roll_deg"]),
                            "pitch_deg": format_float(att["pitch_deg"]),
                            "yaw_deg": format_float(att["yaw_deg"]),
                        }
                        add_feedback_to_row(row, "left", left_fb, args.left_open, args.left_close)
                        add_feedback_to_row(row, "right", right_fb, args.right_open, args.right_close)
                        writer.writerow(row)
                        csv_file.flush()
                        sample_index += 1
                        time.sleep(period)

                    print(
                        "cycle=%s direction=%s ratio=%.3f target=(%s,%s) pos=(%s,%s)"
                        % (cycle, direction, target_ratio, target_left, target_right, left_fb["pos"], right_fb["pos"])
                    )

        if not args.no_park:
            park_ratio = clamp(args.park_ratio, 0.0, 1.0)
            write_position(
                packet,
                args.left_id,
                ratio_to_pos(args.left_open, args.left_close, park_ratio),
                args.speed,
                args.acc,
                args.torque_limit,
            )
            write_position(
                packet,
                args.right_id,
                ratio_to_pos(args.right_open, args.right_close, park_ratio),
                args.speed,
                args.acc,
                args.torque_limit,
            )
            print("parked at close_ratio=%.3f" % park_ratio)

        if args.disable_torque_at_end:
            for servo_id in (args.left_id, args.right_id):
                result, error = packet.EnableTorque(servo_id, 0)
                check(packet, result, error, "disable torque id=%s" % servo_id)

    finally:
        attitude.close()
        port.closePort()

    print("wrote raw samples: %s" % args.output)


def load_limits_config(path):
    if not path:
        return None
    config_path = Path(path).expanduser()
    if not config_path.exists():
        return None
    with open(config_path) as json_file:
        return json.load(json_file)


def servo_limit_value(config, side, *names):
    servo = config.get("servos", {}).get(side, {})
    for name in names:
        if name in servo and servo[name] is not None:
            return servo[name]
    return None


def fill_missing_arg(args, name, value):
    if getattr(args, name) is None and value is not None:
        setattr(args, name, int(value))


def apply_collect_full_limits_config(args):
    config = load_limits_config(args.limits_json)
    if config is None:
        return

    fill_missing_arg(args, "left_open", servo_limit_value(config, "left", "open_pos", "open"))
    fill_missing_arg(args, "left_clear", servo_limit_value(config, "left", "clear_pos", "clear"))
    fill_missing_arg(args, "left_max", servo_limit_value(config, "left", "close_pos", "max_pos", "max"))
    fill_missing_arg(args, "right_open", servo_limit_value(config, "right", "open_pos", "open"))
    fill_missing_arg(args, "right_clear", servo_limit_value(config, "right", "clear_pos", "clear"))
    fill_missing_arg(args, "right_max", servo_limit_value(config, "right", "close_pos", "max_pos", "max"))

    print("loaded gripper limits: %s" % Path(args.limits_json).expanduser())


def validate_full_collect_args(args):
    missing = [
        name
        for name in (
            "left_open",
            "left_clear",
            "left_max",
            "right_open",
            "right_clear",
            "right_max",
        )
        if getattr(args, name) is None
    ]
    if missing:
        raise RuntimeError(
            "missing collect-full position limits: %s. Pass them explicitly or provide --limits-json."
            % ", ".join("--" + name.replace("_", "-") for name in missing)
        )
    if args.left_open == args.left_max:
        raise RuntimeError("--left-open and --left-max must differ")
    if args.right_open == args.right_max:
        raise RuntimeError("--right-open and --right-max must differ")

    left_clear_ratio = pos_to_ratio(args.left_open, args.left_max, args.left_clear)
    right_clear_ratio = pos_to_ratio(args.right_open, args.right_max, args.right_clear)
    if not 0.0 < left_clear_ratio < 1.0:
        raise RuntimeError(
            "--left-clear must lie between --left-open and --left-max in the full close-ratio range"
        )
    if not 0.0 < right_clear_ratio < 1.0:
        raise RuntimeError(
            "--right-clear must lie between --right-open and --right-max in the full close-ratio range"
        )
    return left_clear_ratio, right_clear_ratio


def build_full_segments(left_clear_ratio, right_clear_ratio, args):
    return [
        {
            "name": "both_clear",
            "left_start": 0.0,
            "left_stop": left_clear_ratio,
            "right_start": 0.0,
            "right_stop": right_clear_ratio,
            "left_active": 1,
            "right_active": 1,
            "points": args.sweep_points_clear,
        },
        {
            "name": "left_full",
            "left_start": 0.0,
            "left_stop": 1.0,
            "right_start": 0.0,
            "right_stop": 0.0,
            "left_active": 1,
            "right_active": 0,
            "points": args.sweep_points_extension,
        },
        {
            "name": "right_full",
            "left_start": 0.0,
            "left_stop": 0.0,
            "right_start": 0.0,
            "right_stop": 1.0,
            "left_active": 0,
            "right_active": 1,
            "points": args.sweep_points_extension,
        },
    ]


def open_transition_speed(args, profile):
    speed = getattr(args, "inter_segment_open_speed", 0)
    return int(speed) if speed and int(speed) > 0 else int(profile["speed"])


def open_transition_acc(args, profile):
    acc = getattr(args, "inter_segment_open_acc", 0)
    return int(acc) if acc and int(acc) > 0 else int(profile["acc"])


def open_transition_torque_limit(args, profile):
    torque_limit = getattr(args, "inter_segment_open_torque_limit", 0)
    return int(torque_limit) if torque_limit and int(torque_limit) > 0 else int(profile["torque_limit"])


def open_transition_settle_time(args):
    return float(getattr(args, "inter_segment_open_settle_time", 0.8))


def move_open_between_empty_segments(packet, args, profile, label):
    write_position(
        packet,
        args.left_id,
        args.left_open,
        open_transition_speed(args, profile),
        open_transition_acc(args, profile),
        open_transition_torque_limit(args, profile),
    )
    write_position(
        packet,
        args.right_id,
        args.right_open,
        open_transition_speed(args, profile),
        open_transition_acc(args, profile),
        open_transition_torque_limit(args, profile),
    )
    left_fb, right_fb = wait_for_position_targets(
        packet,
        args,
        args.left_open,
        args.right_open,
        getattr(args, "inter_segment_open_timeout", 12.0),
        getattr(args, "position_tolerance", 35.0),
        "empty transition %s" % label,
    )
    time.sleep(open_transition_settle_time(args))
    print(
        "empty transition=%s parked_open target=(%s,%s) pos=(%s,%s)"
        % (label, args.left_open, args.right_open, left_fb["pos"], right_fb["pos"])
    )


def wait_for_position_targets(packet, args, target_left, target_right, timeout_s, tolerance_ticks, label):
    deadline = time.monotonic() + max(0.0, timeout_s)
    left_fb = read_feedback(packet, args.left_id)
    right_fb = read_feedback(packet, args.right_id)
    while True:
        safety_check(args, left_fb, right_fb)
        left_error = abs(float(left_fb["pos"]) - float(target_left))
        right_error = abs(float(right_fb["pos"]) - float(target_right))
        if left_error <= tolerance_ticks and right_error <= tolerance_ticks:
            return left_fb, right_fb
        if time.monotonic() >= deadline:
            message = (
                "%s did not settle: target=(%s,%s) pos=(%s,%s) error=(%.1f,%.1f) "
                "tolerance=%.1f timeout=%.1fs"
                % (
                    label,
                    target_left,
                    target_right,
                    left_fb["pos"],
                    right_fb["pos"],
                    left_error,
                    right_error,
                    tolerance_ticks,
                    timeout_s,
                )
            )
            if getattr(args, "allow_unsettled", False):
                print("warning: %s" % message)
                return left_fb, right_fb
            raise RuntimeError(message)
        time.sleep(0.05)
        left_fb = read_feedback(packet, args.left_id)
        right_fb = read_feedback(packet, args.right_id)


def empty_active_target_ratio(segment, left_ratio, right_ratio):
    active_ratios = []
    if segment["left_active"]:
        active_ratios.append(left_ratio)
    if segment["right_active"]:
        active_ratios.append(right_ratio)
    if not active_ratios:
        return 0.0
    return sum(active_ratios) / float(len(active_ratios))


def build_empty_row(
    args,
    attitude,
    segment,
    profile,
    cycle,
    direction,
    target_ratio,
    target_left,
    target_right,
    sample_index,
    left_fb,
    right_fb,
    motion_phase,
):
    att = attitude.read()
    row = {
        "schema_version": SCHEMA_VERSION,
        "utc_time": utc_now(),
        "monotonic_s": "%.6f" % time.monotonic(),
        "cycle": cycle,
        "direction": direction,
        "segment": segment["name"],
        "session_phase": "empty",
        "profile_index": profile["profile_index"],
        "profile_name": profile["profile_name"],
        "speed_cmd": profile["speed"],
        "acc_cmd": profile["acc"],
        "torque_limit": profile["torque_limit"],
        "object_label": "",
        "trial_kind": "",
        "trial_index": "",
        "contact_phase": motion_phase,
        "target_ratio": "%.6f" % target_ratio,
        "sample_index": sample_index,
        "left_id": args.left_id,
        "right_id": args.right_id,
        "left_open": args.left_open,
        "left_clear": args.left_clear,
        "left_close": args.left_max,
        "right_open": args.right_open,
        "right_clear": args.right_clear,
        "right_close": args.right_max,
        "left_inward_sign": args.left_inward_sign,
        "right_inward_sign": args.right_inward_sign,
        "left_active": segment["left_active"],
        "right_active": segment["right_active"],
        "target_left_pos": target_left,
        "target_right_pos": target_right,
        "attitude_source": args.attitude_source,
        "attitude_age_s": format_float(att["attitude_age_s"]),
        "roll_deg": format_float(att["roll_deg"]),
        "pitch_deg": format_float(att["pitch_deg"]),
        "yaw_deg": format_float(att["yaw_deg"]),
    }
    add_feedback_to_row(row, "left", left_fb, args.left_open, args.left_max)
    add_feedback_to_row(row, "right", right_fb, args.right_open, args.right_max)
    return row


def write_empty_rows_for_settled_target(
    packet,
    writer,
    csv_file,
    args,
    attitude,
    segment,
    profile,
    cycle,
    direction,
    left_ratio,
    right_ratio,
    target_left,
    target_right,
    sample_index,
):
    left_fb, right_fb = wait_for_position_targets(
        packet,
        args,
        target_left,
        target_right,
        getattr(args, "move_timeout", 12.0),
        getattr(args, "position_tolerance", 35.0),
        "%s profile=%s cycle=%s" % (segment["name"], profile["profile_index"], cycle),
    )
    time.sleep(args.settle_time)

    samples_here = max(1, int(round(args.sample_duration * args.sample_hz)))
    period = 1.0 / args.sample_hz
    target_ratio = empty_active_target_ratio(segment, left_ratio, right_ratio)
    for _ in range(samples_here):
        left_fb = read_feedback(packet, args.left_id)
        right_fb = read_feedback(packet, args.right_id)
        safety_check(args, left_fb, right_fb)
        row = build_empty_row(
            args,
            attitude,
            segment,
            profile,
            cycle,
            direction,
            target_ratio,
            target_left,
            target_right,
            sample_index,
            left_fb,
            right_fb,
            "settled",
        )
        writer.writerow(row)
        csv_file.flush()
        sample_index += 1
        time.sleep(period)
    return sample_index, left_fb, right_fb


def write_empty_rows_for_moving_sweep(
    packet,
    writer,
    csv_file,
    args,
    attitude,
    segment,
    profile,
    cycle,
    direction,
    start_left_ratio,
    stop_left_ratio,
    start_right_ratio,
    stop_right_ratio,
    sample_index,
):
    start_left = ratio_to_pos(args.left_open, args.left_max, start_left_ratio)
    start_right = ratio_to_pos(args.right_open, args.right_max, start_right_ratio)
    stop_left = ratio_to_pos(args.left_open, args.left_max, stop_left_ratio)
    stop_right = ratio_to_pos(args.right_open, args.right_max, stop_right_ratio)
    write_position(packet, args.left_id, start_left, profile["speed"], profile["acc"], profile["torque_limit"])
    write_position(packet, args.right_id, start_right, profile["speed"], profile["acc"], profile["torque_limit"])
    wait_for_position_targets(
        packet,
        args,
        start_left,
        start_right,
        getattr(args, "move_timeout", 12.0),
        getattr(args, "position_tolerance", 35.0),
        "%s profile=%s cycle=%s start"
        % (segment["name"], profile["profile_index"], cycle),
    )
    time.sleep(args.settle_time)

    write_position(packet, args.left_id, stop_left, profile["speed"], profile["acc"], profile["torque_limit"])
    write_position(packet, args.right_id, stop_right, profile["speed"], profile["acc"], profile["torque_limit"])

    period = 1.0 / args.sample_hz
    deadline = time.monotonic() + max(0.0, getattr(args, "move_timeout", 12.0))
    target_ratio = empty_active_target_ratio(segment, stop_left_ratio, stop_right_ratio)
    left_fb = read_feedback(packet, args.left_id)
    right_fb = read_feedback(packet, args.right_id)
    while True:
        safety_check(args, left_fb, right_fb)
        row = build_empty_row(
            args,
            attitude,
            segment,
            profile,
            cycle,
            direction,
            target_ratio,
            stop_left,
            stop_right,
            sample_index,
            left_fb,
            right_fb,
            "moving",
        )
        writer.writerow(row)
        csv_file.flush()
        sample_index += 1

        left_error = abs(float(left_fb["pos"]) - float(stop_left))
        right_error = abs(float(right_fb["pos"]) - float(stop_right))
        if left_error <= args.position_tolerance and right_error <= args.position_tolerance:
            break
        if time.monotonic() >= deadline:
            message = (
                "%s moving sweep did not settle: target=(%s,%s) pos=(%s,%s) "
                "error=(%.1f,%.1f) tolerance=%.1f timeout=%.1fs"
                % (
                    segment["name"],
                    stop_left,
                    stop_right,
                    left_fb["pos"],
                    right_fb["pos"],
                    left_error,
                    right_error,
                    args.position_tolerance,
                    getattr(args, "move_timeout", 12.0),
                )
            )
            if getattr(args, "allow_unsettled", False):
                print("warning: %s" % message)
                break
            raise RuntimeError(message)
        time.sleep(period)
        left_fb = read_feedback(packet, args.left_id)
        right_fb = read_feedback(packet, args.right_id)

    if args.sample_duration > 0.0:
        samples_here = max(1, int(round(args.sample_duration * args.sample_hz)))
        for _ in range(samples_here):
            left_fb = read_feedback(packet, args.left_id)
            right_fb = read_feedback(packet, args.right_id)
            safety_check(args, left_fb, right_fb)
            row = build_empty_row(
                args,
                attitude,
                segment,
                profile,
                cycle,
                direction,
                target_ratio,
                stop_left,
                stop_right,
                sample_index,
                left_fb,
                right_fb,
                "settled_tail",
            )
            writer.writerow(row)
            csv_file.flush()
            sample_index += 1
            time.sleep(period)
    return sample_index, left_fb, right_fb, stop_left, stop_right


def write_full_range_samples(
    packet,
    writer,
    csv_file,
    args,
    attitude,
    profiles,
    opening_profiles=None,
    sample_index=0,
):
    left_clear_ratio, right_clear_ratio = validate_full_collect_args(args)
    segments = build_full_segments(left_clear_ratio, right_clear_ratio, args)
    opening_profiles = list(opening_profiles or [])

    for profile_position, profile in enumerate(profiles):
        opening_profile = None
        if opening_profiles:
            opening_profile = opening_profiles[min(profile_position, len(opening_profiles) - 1)]
        print(
            "empty profile=%s speed=%s acc=%s torque_limit=%s"
            % (
                profile["profile_index"],
                profile["speed"],
                profile["acc"],
                profile["torque_limit"],
            )
        )
        if args.empty_sweep_direction == "bidirectional" and opening_profile:
            print(
                "empty opening_profile=%s speed=%s acc=%s torque_limit=%s"
                % (
                    opening_profile["profile_name"],
                    opening_profile["speed"],
                    opening_profile["acc"],
                    opening_profile["torque_limit"],
                )
            )
        for segment in segments:
            for cycle in range(args.cycles):
                move_open_between_empty_segments(
                    packet,
                    args,
                    profile,
                    "profile_%s_before_%s_cycle_%s"
                    % (profile["profile_index"], segment["name"], cycle),
                )
                direction = "closing"
                if args.empty_sample_mode == "moving":
                    start_left_ratio = segment["left_start"]
                    stop_left_ratio = segment["left_stop"]
                    start_right_ratio = segment["right_start"]
                    stop_right_ratio = segment["right_stop"]
                    sample_index, left_fb, right_fb, target_left, target_right = write_empty_rows_for_moving_sweep(
                        packet,
                        writer,
                        csv_file,
                        args,
                        attitude,
                        segment,
                        profile,
                        cycle,
                        direction,
                        start_left_ratio,
                        stop_left_ratio,
                        start_right_ratio,
                        stop_right_ratio,
                        sample_index,
                    )
                    print(
                        "segment=%s profile=%s cycle=%s direction=%s sweep=(%.3f,%.3f)->(%.3f,%.3f) "
                        "target=(%s,%s) pos=(%s,%s)"
                        % (
                            segment["name"],
                            profile["profile_index"],
                            cycle,
                            direction,
                            start_left_ratio,
                            start_right_ratio,
                            stop_left_ratio,
                            stop_right_ratio,
                            target_left,
                            target_right,
                            left_fb["pos"],
                            right_fb["pos"],
                        )
                    )
                    if args.empty_sweep_direction == "bidirectional" and opening_profile:
                        sample_index, left_fb, right_fb, target_left, target_right = write_empty_rows_for_moving_sweep(
                            packet,
                            writer,
                            csv_file,
                            args,
                            attitude,
                            segment,
                            opening_profile,
                            cycle,
                            "opening",
                            segment["left_stop"],
                            segment["left_start"],
                            segment["right_stop"],
                            segment["right_start"],
                            sample_index,
                        )
                        print(
                            "segment=%s profile=%s cycle=%s direction=opening sweep=(%.3f,%.3f)->(%.3f,%.3f) "
                            "target=(%s,%s) pos=(%s,%s)"
                            % (
                                segment["name"],
                                opening_profile["profile_name"],
                                cycle,
                                segment["left_stop"],
                                segment["right_stop"],
                                segment["left_start"],
                                segment["right_start"],
                                target_left,
                                target_right,
                                left_fb["pos"],
                                right_fb["pos"],
                            )
                        )
                else:
                    left_ratios = make_ratio_sequence_between(
                        segment["left_start"], segment["left_stop"], segment["points"], 0
                    )
                    right_ratios = make_ratio_sequence_between(
                        segment["right_start"], segment["right_stop"], segment["points"], 0
                    )
                    for left_ratio, right_ratio in zip(left_ratios, right_ratios):
                        target_left = ratio_to_pos(args.left_open, args.left_max, left_ratio)
                        target_right = ratio_to_pos(args.right_open, args.right_max, right_ratio)
                        write_position(
                            packet,
                            args.left_id,
                            target_left,
                            profile["speed"],
                            profile["acc"],
                            profile["torque_limit"],
                        )
                        write_position(
                            packet,
                            args.right_id,
                            target_right,
                            profile["speed"],
                            profile["acc"],
                            profile["torque_limit"],
                        )
                        sample_index, left_fb, right_fb = write_empty_rows_for_settled_target(
                            packet,
                            writer,
                            csv_file,
                            args,
                            attitude,
                            segment,
                            profile,
                            cycle,
                            direction,
                            left_ratio,
                            right_ratio,
                            target_left,
                            target_right,
                            sample_index,
                        )

                        print(
                            "segment=%s profile=%s cycle=%s direction=%s left_ratio=%.3f right_ratio=%.3f "
                            "target=(%s,%s) pos=(%s,%s)"
                            % (
                                segment["name"],
                                profile["profile_index"],
                                cycle,
                                direction,
                                left_ratio,
                                right_ratio,
                                target_left,
                                target_right,
                                left_fb["pos"],
                                right_fb["pos"],
                            )
                        )
                    if args.empty_sweep_direction == "bidirectional" and opening_profile:
                        left_ratios = make_ratio_sequence_between(
                            segment["left_start"], segment["left_stop"], segment["points"], 1
                        )
                        right_ratios = make_ratio_sequence_between(
                            segment["right_start"], segment["right_stop"], segment["points"], 1
                        )
                        for left_ratio, right_ratio in zip(left_ratios, right_ratios):
                            target_left = ratio_to_pos(args.left_open, args.left_max, left_ratio)
                            target_right = ratio_to_pos(args.right_open, args.right_max, right_ratio)
                            write_position(
                                packet,
                                args.left_id,
                                target_left,
                                opening_profile["speed"],
                                opening_profile["acc"],
                                opening_profile["torque_limit"],
                            )
                            write_position(
                                packet,
                                args.right_id,
                                target_right,
                                opening_profile["speed"],
                                opening_profile["acc"],
                                opening_profile["torque_limit"],
                            )
                            sample_index, left_fb, right_fb = write_empty_rows_for_settled_target(
                                packet,
                                writer,
                                csv_file,
                                args,
                                attitude,
                                segment,
                                opening_profile,
                                cycle,
                                "opening",
                                left_ratio,
                                right_ratio,
                                target_left,
                                target_right,
                                sample_index,
                            )

                            print(
                                "segment=%s profile=%s cycle=%s direction=opening left_ratio=%.3f right_ratio=%.3f "
                                "target=(%s,%s) pos=(%s,%s)"
                                % (
                                    segment["name"],
                                    opening_profile["profile_name"],
                                    cycle,
                                    left_ratio,
                                    right_ratio,
                                    target_left,
                                    target_right,
                                    left_fb["pos"],
                                    right_fb["pos"],
                                )
                            )
            move_open_between_empty_segments(
                packet,
                args,
                profile,
                "profile_%s_after_%s" % (profile["profile_index"], segment["name"]),
            )
    return sample_index


def collect_full(args):
    apply_collect_full_limits_config(args)
    if args.profiles.strip():
        profiles = parse_profiles(args.profiles)
    else:
        profiles = [
            {
                "profile_index": 0,
                "profile_name": "p0",
                "speed": args.speed,
                "acc": args.acc,
                "torque_limit": args.torque_limit,
                "label": "%s:%s:%s" % (args.speed, args.acc, args.torque_limit),
            }
        ]
    opening_profiles = parse_opening_profiles(args)
    load_sdk(args.sdk_root)
    ensure_parent(args.output_csv)
    ensure_parent(args.output_json)
    if args.curve_csv:
        ensure_parent(args.curve_csv)

    validate_full_collect_args(args)

    attitude = AttitudeReader(
        args.attitude_source,
        args.attitude_topic,
        args.roll_deg,
        args.pitch_deg,
        args.yaw_deg,
    )
    attitude.start()

    port = PortHandler(args.port)
    packet = hls(port)

    if not port.openPort():
        raise RuntimeError("failed to open port %s" % args.port)
    if not port.setBaudRate(args.baud):
        raise RuntimeError("failed to set baudrate %s" % args.baud)

    try:
        setup_servo_position_mode(packet, args.left_id)
        setup_servo_position_mode(packet, args.right_id)

        with open(args.output_csv, "w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=DEFAULT_FIELDS)
            writer.writeheader()

            write_full_range_samples(
                packet,
                writer,
                csv_file,
                args,
                attitude,
                profiles,
                opening_profiles,
                sample_index=0,
            )

        if not args.no_park:
            park_profile = profiles[0]
            park_ratio = clamp(args.park_ratio, 0.0, 1.0)
            write_position(
                packet,
                args.left_id,
                ratio_to_pos(args.left_open, args.left_max, park_ratio),
                park_profile["speed"],
                park_profile["acc"],
                park_profile["torque_limit"],
            )
            write_position(
                packet,
                args.right_id,
                ratio_to_pos(args.right_open, args.right_max, park_ratio),
                park_profile["speed"],
                park_profile["acc"],
                park_profile["torque_limit"],
            )
            print("parked at full close_ratio=%.3f" % park_ratio)

        if args.disable_torque_at_end:
            for servo_id in (args.left_id, args.right_id):
                result, error = packet.EnableTorque(servo_id, 0)
                check(packet, result, error, "disable torque id=%s" % servo_id)

    finally:
        attitude.close()
        port.closePort()

    print("wrote full raw samples: %s" % args.output_csv)
    rows = read_csv_rows(args.output_csv)
    table = build_compensation_table(rows, args, args.output_csv)
    table.setdefault("collection", {})
    table["collection"]["profiles"] = profiles
    table["collection"]["opening_profiles"] = opening_profiles
    write_compensation_outputs(table, args.output_json, args.curve_csv)


def format_float(value):
    if value is None or not math.isfinite(value):
        return ""
    return "%.6f" % value


def add_feedback_to_row(row, side, feedback, open_pos, close_pos):
    row["%s_pos" % side] = feedback["pos"]
    row["%s_close_ratio" % side] = "%.6f" % pos_to_ratio(open_pos, close_pos, feedback["pos"])
    row["%s_speed" % side] = feedback["speed"]
    row["%s_load" % side] = feedback["load"]
    row["%s_voltage" % side] = feedback["voltage"]
    row["%s_temp" % side] = feedback["temp"]
    row["%s_moving" % side] = feedback["moving"]
    row["%s_current" % side] = feedback["current"]


def median_abs_deviation(values, center):
    if not values:
        return None
    return statistics.median([abs(value - center) for value in values])


def quantile(values, fraction):
    clean = sorted(value for value in values if value is not None and math.isfinite(value))
    if not clean:
        return None
    if len(clean) == 1:
        return clean[0]
    fraction = clamp(float(fraction), 0.0, 1.0)
    position = fraction * (len(clean) - 1)
    low = int(math.floor(position))
    high = int(math.ceil(position))
    if low == high:
        return clean[low]
    alpha = position - low
    return clean[low] * (1.0 - alpha) + clean[high] * alpha


def bin_ratio(value, bins):
    if not math.isfinite(value):
        return None
    if bins <= 1:
        return 0.0
    value = clamp(value, 0.0, 1.0)
    idx = int(round(value * (bins - 1)))
    return idx / float(bins - 1)


def bin_angle(value, size_deg):
    if not math.isfinite(value) or size_deg <= 0:
        return None
    return round(value / size_deg) * size_deg


def read_csv_rows(path):
    with open(path, newline="") as csv_file:
        return list(csv.DictReader(csv_file))


def collect_side_points(rows, side, args):
    buckets = {}
    skipped_inactive = 0
    skipped_speed = 0
    skipped_invalid = 0
    for row in rows:
        active = row.get("%s_active" % side)
        if active not in (None, ""):
            try:
                if int(float(active)) == 0:
                    skipped_inactive += 1
                    continue
            except (TypeError, ValueError):
                pass

        speed = finite_float(row.get("%s_speed" % side))
        if math.isfinite(speed) and args.max_abs_speed >= 0 and abs(speed) > args.max_abs_speed:
            skipped_speed += 1
            continue

        ratio = finite_float(row.get("%s_close_ratio" % side))
        current = finite_float(row.get("%s_current" % side))
        load = finite_float(row.get("%s_load" % side))
        pos = finite_float(row.get("%s_pos" % side))
        roll = finite_float(row.get("roll_deg"))
        pitch = finite_float(row.get("pitch_deg"))
        if not math.isfinite(ratio) or not math.isfinite(current):
            skipped_invalid += 1
            continue

        key = (
            row.get("direction", ""),
            row.get("profile_index", ""),
            row.get("profile_name", ""),
            bin_ratio(ratio, args.position_bins),
            bin_angle(roll, args.roll_bin_size_deg),
            bin_angle(pitch, args.pitch_bin_size_deg),
        )
        bucket = buckets.setdefault(
            key,
            {
                "current": [],
                "load": [],
                "pos": [],
                "abs_speed": [],
                "roll": [],
                "pitch": [],
                "speed_cmd": [],
                "acc_cmd": [],
                "torque_limit": [],
            },
        )
        bucket["current"].append(current)
        if math.isfinite(load):
            bucket["load"].append(load)
        if math.isfinite(pos):
            bucket["pos"].append(pos)
        if math.isfinite(speed):
            bucket["abs_speed"].append(abs(speed))
        if math.isfinite(roll):
            bucket["roll"].append(roll)
        if math.isfinite(pitch):
            bucket["pitch"].append(pitch)
        for name in ("speed_cmd", "acc_cmd", "torque_limit"):
            value = finite_float(row.get(name))
            if math.isfinite(value):
                bucket[name].append(value)

    points = []
    skipped_samples = 0
    for key, bucket in buckets.items():
        sample_count = len(bucket["current"])
        if sample_count < args.min_samples:
            skipped_samples += sample_count
            continue
        currents = bucket["current"]
        current_median = statistics.median(bucket["current"])
        current_mad = median_abs_deviation(currents, current_median)
        load_median = statistics.median(bucket["load"]) if bucket["load"] else None
        pos_median = statistics.median(bucket["pos"]) if bucket["pos"] else None
        speed_abs_median = statistics.median(bucket["abs_speed"]) if bucket["abs_speed"] else None
        points.append(
            {
                "direction": key[0],
                "profile_index": first_int([{"value": key[1]}], "value", None),
                "profile_name": key[2],
                "speed_cmd": statistics.median(bucket["speed_cmd"]) if bucket["speed_cmd"] else None,
                "acc_cmd": statistics.median(bucket["acc_cmd"]) if bucket["acc_cmd"] else None,
                "torque_limit": statistics.median(bucket["torque_limit"]) if bucket["torque_limit"] else None,
                "close_ratio": key[3],
                "roll_bin_deg": key[4],
                "pitch_bin_deg": key[5],
                "roll_median_deg": statistics.median(bucket["roll"]) if bucket["roll"] else None,
                "pitch_median_deg": statistics.median(bucket["pitch"]) if bucket["pitch"] else None,
                "pos_median": pos_median,
                "current_median": current_median,
                "current_mad": current_mad,
                "current_q05": quantile(currents, 0.05),
                "current_q50": quantile(currents, 0.50),
                "current_q95": quantile(currents, 0.95),
                "current_q99": quantile(currents, 0.99),
                "current_low": quantile(currents, 0.05),
                "current_high": quantile(currents, 0.99),
                "load_median": load_median,
                "load_mad": median_abs_deviation(bucket["load"], load_median)
                if load_median is not None
                else None,
                "speed_abs_median": speed_abs_median,
                "samples": sample_count,
            }
        )

    points.sort(
        key=lambda point: (
            point.get("direction", ""),
            none_last(point.get("profile_index")),
            none_last(point["roll_bin_deg"]),
            none_last(point["pitch_bin_deg"]),
            none_last(point["close_ratio"]),
        )
    )
    return points, {
        "skipped_inactive": skipped_inactive,
        "skipped_for_speed": skipped_speed,
        "skipped_invalid": skipped_invalid,
        "skipped_for_min_samples": skipped_samples,
    }


def none_last(value):
    if value is None:
        return float("inf")
    return value


def first_int(rows, field, default=None):
    for row in rows:
        value = row.get(field)
        try:
            return int(float(value))
        except (TypeError, ValueError):
            pass
    return default


def first_str(rows, field, default=""):
    for row in rows:
        value = row.get(field)
        if value:
            return value
    return default


def collect_profiles_from_rows(rows, direction=None):
    profiles = {}
    for row in rows:
        if direction and row.get("direction") != direction:
            continue
        index = first_int([{"value": row.get("profile_index")}], "value", None)
        if index is None:
            continue
        if index in profiles:
            continue
        profiles[index] = {
            "profile_index": index,
            "profile_name": row.get("profile_name") or "p%s" % index,
            "speed": first_int([{"value": row.get("speed_cmd")}], "value", 0),
            "acc": first_int([{"value": row.get("acc_cmd")}], "value", 0),
            "torque_limit": first_int([{"value": row.get("torque_limit")}], "value", 0),
        }
    return [profiles[index] for index in sorted(profiles)]


def build_compensation_table(rows, args, source_csv):
    if not rows:
        raise RuntimeError("input CSV has no rows: %s" % source_csv)

    left_points, left_stats = collect_side_points(rows, "left", args)
    right_points, right_stats = collect_side_points(rows, "right", args)
    table = {
        "schema_version": SCHEMA_VERSION,
        "created_at": utc_now(),
        "source_csv": os.path.abspath(source_csv),
        "description": "No-load HLS gripper current/load baseline table.",
        "runtime_usage": {
            "baseline": "interpolate current_median by side, close_ratio, roll_deg, pitch_deg",
            "current_residual": "measured_current - baseline_current",
            "contact_metric": "inward_sign * current_residual",
            "gravity_feedforward_start": "start with a limited fraction of baseline_current before using full compensation",
        },
        "fit": {
            "position_bins": args.position_bins,
            "roll_bin_size_deg": args.roll_bin_size_deg,
            "pitch_bin_size_deg": args.pitch_bin_size_deg,
            "max_abs_speed": args.max_abs_speed,
            "min_samples": args.min_samples,
        },
        "servos": {
            "left": {
                "id": first_int(rows, "left_id"),
                "open_pos": first_int(rows, "left_open"),
                "clear_pos": first_int(rows, "left_clear"),
                "close_pos": first_int(rows, "left_close"),
                "inward_sign": first_int(rows, "left_inward_sign"),
                "points": left_points,
                "stats": left_stats,
            },
            "right": {
                "id": first_int(rows, "right_id"),
                "open_pos": first_int(rows, "right_open"),
                "clear_pos": first_int(rows, "right_clear"),
                "close_pos": first_int(rows, "right_close"),
                "inward_sign": first_int(rows, "right_inward_sign"),
                "points": right_points,
                "stats": right_stats,
            },
        },
        "collection": {
            "attitude_source": first_str(rows, "attitude_source"),
            "row_count": len(rows),
            "profiles": collect_profiles_from_rows(rows, direction="closing"),
            "opening_profiles": collect_profiles_from_rows(rows, direction="opening"),
        },
    }
    return table


def write_compensation_outputs(table, output, curve_csv):
    ensure_parent(output)
    with open(output, "w") as json_file:
        json.dump(table, json_file, indent=2, sort_keys=True)
        json_file.write("\n")

    if curve_csv:
        write_curve_csv(curve_csv, table)

    print("wrote table: %s" % output)
    print(
        "left points=%s right points=%s"
        % (
            len(table["servos"]["left"]["points"]),
            len(table["servos"]["right"]["points"]),
        )
    )
    if curve_csv:
        print("wrote curve CSV: %s" % curve_csv)


def fit(args):
    rows = read_csv_rows(args.input)
    table = build_compensation_table(rows, args, args.input)
    write_compensation_outputs(table, args.output, args.curve_csv)


def profile_matches_point(point, profile_index=None, profile_name=None, direction=None):
    if direction not in (None, ""):
        point_direction = str(point.get("direction", ""))
        if point_direction and point_direction != str(direction):
            return False
    if profile_index not in (None, ""):
        point_index = first_int([{"value": point.get("profile_index")}], "value", None)
        target_index = first_int([{"value": profile_index}], "value", None)
        if target_index is not None and point_index != target_index:
            return False
    if profile_name not in (None, ""):
        point_name = str(point.get("profile_name", ""))
        if point_name and point_name != str(profile_name):
            return False
    return True


def table_baseline_current(
    table,
    side,
    close_ratio,
    roll_deg,
    pitch_deg,
    profile_index=None,
    profile_name=None,
    direction=None,
):
    points = table.get("servos", {}).get(side, {}).get("points", [])
    if not points:
        return 0.0
    profile_points = [
        point
        for point in points
        if profile_matches_point(
            point,
            profile_index=profile_index,
            profile_name=profile_name,
            direction=direction,
        )
    ]
    if profile_points:
        points = profile_points
    elif direction not in (None, ""):
        direction_points = [
            point
            for point in points
            if profile_matches_point(point, direction=direction)
        ]
        if direction_points:
            points = direction_points
    close_ratio = clamp(finite_float(close_ratio, 0.0), 0.0, 1.0)
    roll_deg = finite_float(roll_deg, 0.0)
    pitch_deg = finite_float(pitch_deg, 0.0)
    best_point = points[0]
    best_score = float("inf")
    for point in points:
        ratio = finite_float(point.get("close_ratio"), 0.0)
        roll = finite_float(point.get("roll_bin_deg"), 0.0)
        pitch = finite_float(point.get("pitch_bin_deg"), 0.0)
        score = abs(ratio - close_ratio) / 0.05 + abs(roll - roll_deg) / 5.0 + abs(pitch - pitch_deg) / 5.0
        if score < best_score:
            best_score = score
            best_point = point
    return finite_float(best_point.get("current_median"), 0.0)


def baseline_noise_stats(table, side, profile_index=None, profile_name=None, direction=None):
    points = table.get("servos", {}).get(side, {}).get("points", [])
    if profile_index not in (None, "") or profile_name not in (None, "") or direction not in (None, ""):
        filtered = [
            point
            for point in points
            if profile_matches_point(
                point,
                profile_index=profile_index,
                profile_name=profile_name,
                direction=direction,
            )
        ]
        if filtered:
            points = filtered
    mads = []
    excesses = []
    for point in points:
        median = finite_float(point.get("current_median"))
        mad = finite_float(point.get("current_mad"))
        q05 = finite_float(point.get("current_q05"))
        q99 = finite_float(point.get("current_q99"))
        if math.isfinite(mad):
            mads.append(abs(mad))
        if math.isfinite(median) and math.isfinite(q99):
            excesses.append(abs(q99 - median))
        if math.isfinite(median) and math.isfinite(q05):
            excesses.append(abs(median - q05))
    median_mad = statistics.median(mads) if mads else 0.0
    q90_excess = quantile(excesses, 0.90) or 0.0
    return {
        "median_mad": median_mad,
        "q90_excess": q90_excess,
        "samples": len(points),
    }


def contact_expected_kinds(side):
    if side == "left":
        return {"center_contact", "left_first_contact"}
    return {"center_contact", "right_first_contact"}


def servo_inward_sign_from_table(table, side):
    return first_int(
        [
            {
                "value": table.get("servos", {}).get(side, {}).get(
                    "inward_sign", 1 if side == "left" else -1
                )
            }
        ],
        "value",
        1 if side == "left" else -1,
    )


def fit_contact_detection_side(
    table,
    contact_rows,
    args,
    side,
    warning_prefix="",
    profile_index=None,
    profile_name=None,
):
    residuals = []
    expected = contact_expected_kinds(side)
    for row in contact_rows:
        if row.get("trial_kind") not in expected:
            continue
        ratio = finite_float(row.get("%s_close_ratio" % side))
        if not math.isfinite(ratio) or ratio < args.contact_fit_min_ratio:
            continue
        residual = finite_float(row.get("%s_current_residual" % side))
        if math.isfinite(residual):
            residuals.append(residual)

    inward_sign = servo_inward_sign_from_table(table, side)
    noise = baseline_noise_stats(
        table,
        side,
        profile_index=profile_index,
        profile_name=profile_name,
        direction="closing",
    )
    baseline_margin = max(
        args.contact_min_enter_threshold,
        3.0 * noise["median_mad"],
        noise["q90_excess"],
    )

    if not residuals:
        warning = "%s%s has no usable contact samples; using fallback thresholds" % (warning_prefix, side)
        return {
            "metric_sign": 1 if inward_sign >= 0 else -1,
            "enter_threshold": max(35.0, baseline_margin),
            "exit_threshold": max(args.contact_min_exit_threshold, min(20.0, 0.65 * max(35.0, baseline_margin))),
            "strong_threshold": max(70.0, 1.8 * max(35.0, baseline_margin)),
            "baseline_noise_mad": noise["median_mad"],
            "baseline_q90_excess": noise["q90_excess"],
            "contact_samples": 0,
            "warning": warning,
        }

    positive_tail = quantile(residuals, 0.90) or 0.0
    negative_tail = quantile([-value for value in residuals], 0.90) or 0.0
    metric_sign = 1 if positive_tail >= negative_tail else -1
    metrics = [metric_sign * value for value in residuals]
    metric_q10 = quantile(metrics, 0.10) or 0.0
    metric_q25 = quantile(metrics, 0.25) or 0.0
    metric_q50 = quantile(metrics, 0.50) or 0.0
    metric_q75 = quantile(metrics, 0.75) or 0.0
    metric_q90 = quantile(metrics, 0.90) or 0.0

    warnings = []
    if metric_q50 <= baseline_margin:
        warnings.append(
            "%s%s contact median %.2f is close to no-load margin %.2f"
            % (warning_prefix, side, metric_q50, baseline_margin)
        )
    enter = max(baseline_margin, 0.35 * max(metric_q50, baseline_margin))
    if metric_q25 > baseline_margin:
        enter = min(max(enter, 0.5 * metric_q25), 0.8 * metric_q25)
    exit_threshold = max(args.contact_min_exit_threshold, min(0.65 * enter, 0.6 * baseline_margin))
    if exit_threshold >= enter:
        exit_threshold = 0.65 * enter
    strong_threshold = max(1.6 * enter, min(metric_q90, 1.25 * max(metric_q75, enter)))
    if strong_threshold < enter:
        strong_threshold = 1.8 * enter

    return {
        "metric_sign": metric_sign,
        "enter_threshold": enter,
        "exit_threshold": exit_threshold,
        "strong_threshold": strong_threshold,
        "baseline_noise_mad": noise["median_mad"],
        "baseline_q90_excess": noise["q90_excess"],
        "contact_samples": len(metrics),
        "contact_metric_q10": metric_q10,
        "contact_metric_q25": metric_q25,
        "contact_metric_median": metric_q50,
        "contact_metric_q75": metric_q75,
        "contact_metric_q90": metric_q90,
        "warning": "; ".join(warnings),
    }


def contact_detection_warnings(detection):
    warnings = []
    for side in ("left", "right"):
        warning = detection.get(side, {}).get("warning", "")
        if warning:
            warnings.append(warning)
    return warnings


def fit_contact_detection_group(table, contact_rows, args, warning_prefix="", profile_index=None, profile_name=None):
    group = {}
    for side in ("left", "right"):
        group[side] = fit_contact_detection_side(
            table,
            contact_rows,
            args,
            side,
            warning_prefix,
            profile_index=profile_index,
            profile_name=profile_name,
        )
    group["warnings"] = contact_detection_warnings(group)
    return group


def fit_contact_detection(table, contact_rows, args, contact_csv_path):
    detection = {
        "schema_version": 1,
        "created_at": utc_now(),
        "source_contact_csv": os.path.abspath(contact_csv_path) if contact_csv_path else "",
        "algorithm": "signed current residual with no-load noise margin and contact-sample quantiles",
        "profiles": {},
    }
    aggregate = fit_contact_detection_group(table, contact_rows, args)
    detection["left"] = aggregate["left"]
    detection["right"] = aggregate["right"]
    detection["warnings"] = list(aggregate["warnings"])

    rows_by_profile = {}
    for row in contact_rows:
        profile_index = row.get("profile_index", "")
        profile_name = row.get("profile_name", "")
        if profile_index == "" and profile_name == "":
            continue
        key = "%s:%s" % (profile_index, profile_name)
        rows_by_profile.setdefault(key, []).append(row)

    for key, rows in sorted(rows_by_profile.items()):
        profile_index, profile_name = key.split(":", 1)
        warning_prefix = "profile %s " % (profile_name or profile_index)
        group = fit_contact_detection_group(
            table,
            rows,
            args,
            warning_prefix,
            profile_index=profile_index,
            profile_name=profile_name,
        )
        group["profile_index"] = first_int([{"value": profile_index}], "value", None)
        group["profile_name"] = profile_name
        detection["profiles"][key] = group
        detection["warnings"].extend(group["warnings"])
    return detection


def comm_error_text(packet, result, error):
    parts = []
    if result != COMM_SUCCESS:
        parts.append(packet.getTxRxResult(result))
    if error:
        parts.append(packet.getRxPacketError(error))
    return "; ".join(parts) if parts else ""


def read_optional(packet, label, read_fn):
    value, result, error = read_fn()
    if result == COMM_SUCCESS and error == 0:
        return {"label": label, "ok": True, "value": value, "error": ""}
    return {
        "label": label,
        "ok": False,
        "value": None,
        "error": comm_error_text(packet, result, error),
    }


def print_read_result(result):
    label = result["label"]
    if result["ok"]:
        print("  %-18s %s" % (label + ":", result["value"]))
    else:
        print("  %-18s FAILED %s" % (label + ":", result["error"]))


def result_value_text(result):
    if result["ok"]:
        return str(result["value"])
    return "ERR"


def read_servo_registers(packet, servo_id, include_limits):
    results = [
        read_optional(packet, "model_l", lambda servo_id=servo_id: packet.read1ByteTxRx(servo_id, HLS_MODEL_L)),
        read_optional(packet, "id_register", lambda servo_id=servo_id: packet.read1ByteTxRx(servo_id, HLS_ID)),
        read_optional(packet, "baud_register", lambda servo_id=servo_id: packet.read1ByteTxRx(servo_id, HLS_BAUD_RATE)),
        read_optional(packet, "mode", lambda servo_id=servo_id: packet.read1ByteTxRx(servo_id, HLS_MODE)),
        read_optional(packet, "present_pos", lambda servo_id=servo_id: packet.ReadPos(servo_id)),
        read_optional(packet, "current", lambda servo_id=servo_id: packet.ReadCurrent(servo_id)),
        read_optional(packet, "temperature", lambda servo_id=servo_id: packet.ReadTemper(servo_id)),
    ]
    if include_limits:
        results.extend(
            [
                read_optional(
                    packet,
                    "min_limit_raw",
                    lambda servo_id=servo_id: packet.read2ByteTxRx(servo_id, HLS_MIN_ANGLE_LIMIT_L),
                ),
                read_optional(
                    packet,
                    "max_limit_raw",
                    lambda servo_id=servo_id: packet.read2ByteTxRx(servo_id, HLS_MAX_ANGLE_LIMIT_L),
                ),
            ]
        )
    return results


def results_to_record(side, servo_id, results):
    record = {"side": side, "id": servo_id}
    for result in results:
        record[result["label"]] = result["value"]
        record[result["label"] + "_ok"] = result["ok"]
        if result["error"]:
            record[result["label"] + "_error"] = result["error"]
    return record


def read_dynamic_row(packet, sides):
    row = {"time_utc": datetime.now(timezone.utc).strftime("%H:%M:%S")}
    for side, servo_id in sides:
        results = {
            result["label"]: result
            for result in read_servo_registers(packet, servo_id, include_limits=False)
        }
        row[f"{side}_pos"] = result_value_text(results["present_pos"])
        row[f"{side}_current"] = result_value_text(results["current"])
        row[f"{side}_temp"] = result_value_text(results["temperature"])
    return row


def print_watch_header():
    print(
        "\nWatching present positions every cycle. Move the gripper by hand and record present_pos. "
        "Press Ctrl+C to stop."
    )
    print("-" * 70)
    print(
        "%-10s | %8s %8s | %7s %7s | %5s %5s"
        % ("time_utc", "L_pos", "R_pos", "L_cur", "R_cur", "L_tmp", "R_tmp")
    )
    print("-" * 70)


def print_watch_row(row):
    print(
        "%-10s | %8s %8s | %7s %7s | %5s %5s"
        % (
            row["time_utc"],
            row["left_pos"],
            row["right_pos"],
            row["left_current"],
            row["right_current"],
            row["left_temp"],
            row["right_temp"],
        ),
        flush=True,
    )


def read_limits(args):
    load_sdk(args.sdk_root)
    port = PortHandler(args.port)
    packet = hls(port)

    if not port.openPort():
        raise RuntimeError("failed to open port %s" % args.port)
    if not port.setBaudRate(args.baud):
        raise RuntimeError("failed to set baudrate %s" % args.baud)

    sides = (("left", args.left_id), ("right", args.right_id))
    records = []
    try:
        print("port=%s baud=%s" % (args.port, args.baud))
        print("read-limits is read-only: it does not change mode, torque, or position.")
        for side, servo_id in sides:
            print("\n%s id=%s" % (side, servo_id))
            results = read_servo_registers(packet, servo_id, include_limits=True)
            for result in results:
                print_read_result(result)
            records.append(results_to_record(side, servo_id, results))

        if args.watch:
            if args.watch_period <= 0.0:
                raise RuntimeError("--watch-period must be positive")
            print_watch_header()
            try:
                while True:
                    print_watch_row(read_dynamic_row(packet, sides))
                    time.sleep(args.watch_period)
            except KeyboardInterrupt:
                print("\nwatch stopped")
    finally:
        port.closePort()

    if args.output_json:
        ensure_parent(args.output_json)
        with open(args.output_json, "w") as json_file:
            json.dump(
                {
                    "schema_version": 1,
                    "utc_time": utc_now(),
                    "port": args.port,
                    "baud": args.baud,
                    "servos": records,
                },
                json_file,
                indent=2,
                sort_keys=True,
            )
            json_file.write("\n")
        print("\nwrote readback JSON: %s" % args.output_json)

    print(
        "\nUse present_pos to record open/clear/max at the actual mechanical posture. "
        "min_limit_raw and max_limit_raw are only the stored Windows angle limits."
    )


def write_open_pose(packet, args):
    write_position(
        packet,
        args.left_id,
        args.left_open,
        args.open_speed,
        args.open_acc,
        args.open_torque_limit,
    )
    write_position(
        packet,
        args.right_id,
        args.right_open,
        args.open_speed,
        args.open_acc,
        args.open_torque_limit,
    )
    time.sleep(args.open_settle_time)


def prompt_plain_step(step, total_steps, title, lines, allow_skip=True):
    print("")
    print("[STEP %s/%s] %s" % (step, total_steps, title))
    for line in lines:
        print("- %s" % line)
    if allow_skip:
        print("Press Enter to start, or type s then Enter to skip.")
    else:
        print("Press Enter to start.")
    try:
        return input("> ").strip().lower()
    except EOFError:
        return ""


def add_contact_baseline_to_row(row, table):
    for side in ("left", "right"):
        baseline = table_baseline_current(
            table,
            side,
            row.get("%s_close_ratio" % side),
            row.get("roll_deg"),
            row.get("pitch_deg"),
            row.get("profile_index"),
            row.get("profile_name"),
            row.get("direction"),
        )
        current = finite_float(row.get("%s_current" % side), 0.0)
        row["%s_current_baseline" % side] = "%.6f" % baseline
        row["%s_current_residual" % side] = "%.6f" % (current - baseline)


def collect_one_contact_trial(
    packet,
    writer,
    csv_file,
    args,
    attitude,
    table,
    object_label,
    trial_kind,
    trial_index,
    profile,
    sample_index,
):
    target_ratio = clamp(args.contact_close_ratio, 0.0, 1.0)
    target_left = ratio_to_pos(args.left_open, args.left_max, target_ratio)
    target_right = ratio_to_pos(args.right_open, args.right_max, target_ratio)

    write_position(packet, args.left_id, target_left, profile["speed"], profile["acc"], profile["torque_limit"])
    write_position(packet, args.right_id, target_right, profile["speed"], profile["acc"], profile["torque_limit"])

    samples_here = max(1, int(round(args.contact_sample_duration * args.contact_sample_hz)))
    period = 1.0 / args.contact_sample_hz
    max_left_residual = 0.0
    max_right_residual = 0.0
    for _ in range(samples_here):
        left_fb = read_feedback(packet, args.left_id)
        right_fb = read_feedback(packet, args.right_id)
        safety_check(args, left_fb, right_fb)
        att = attitude.read()
        row = {
            "schema_version": SCHEMA_VERSION,
            "utc_time": utc_now(),
            "monotonic_s": "%.6f" % time.monotonic(),
            "cycle": "",
            "direction": "closing",
            "segment": "contact",
            "session_phase": "contact",
            "profile_index": profile["profile_index"],
            "profile_name": profile["profile_name"],
            "speed_cmd": profile["speed"],
            "acc_cmd": profile["acc"],
            "torque_limit": profile["torque_limit"],
            "object_label": object_label,
            "trial_kind": trial_kind,
            "trial_index": trial_index,
            "contact_phase": "closing",
            "target_ratio": "%.6f" % target_ratio,
            "sample_index": sample_index,
            "left_id": args.left_id,
            "right_id": args.right_id,
            "left_open": args.left_open,
            "left_clear": args.left_clear,
            "left_close": args.left_max,
            "right_open": args.right_open,
            "right_clear": args.right_clear,
            "right_close": args.right_max,
            "left_inward_sign": args.left_inward_sign,
            "right_inward_sign": args.right_inward_sign,
            "left_active": 1,
            "right_active": 1,
            "target_left_pos": target_left,
            "target_right_pos": target_right,
            "attitude_source": args.attitude_source,
            "attitude_age_s": format_float(att["attitude_age_s"]),
            "roll_deg": format_float(att["roll_deg"]),
            "pitch_deg": format_float(att["pitch_deg"]),
            "yaw_deg": format_float(att["yaw_deg"]),
        }
        add_feedback_to_row(row, "left", left_fb, args.left_open, args.left_max)
        add_feedback_to_row(row, "right", right_fb, args.right_open, args.right_max)
        add_contact_baseline_to_row(row, table)
        max_left_residual = max(max_left_residual, abs(finite_float(row["left_current_residual"], 0.0)))
        max_right_residual = max(max_right_residual, abs(finite_float(row["right_current_residual"], 0.0)))
        writer.writerow(row)
        csv_file.flush()
        sample_index += 1
        time.sleep(period)

    print(
        "contact object=%s kind=%s trial=%s profile=%s samples=%s max_abs_residual=(%.1f, %.1f)"
        % (
            object_label,
            trial_kind,
            trial_index,
            profile["profile_name"],
            samples_here,
            max_left_residual,
            max_right_residual,
        )
    )
    return sample_index


def collect_contact_samples(packet, writer, csv_file, args, attitude, table, profiles):
    object_labels = parse_object_labels(args.object_labels)
    trial_kinds = [
        (
            "center_contact",
            [
                "Put the object near the center between the two fingers.",
                "Keep the aircraft fixed and keep your hands clear before pressing Enter.",
            ],
        ),
        (
            "left_first_contact",
            [
                "Place the object left-biased so the left finger should touch first.",
                "Type s then Enter if you want to skip this offset trial.",
            ],
        ),
        (
            "right_first_contact",
            [
                "Place the object right-biased so the right finger should touch first.",
                "Type s then Enter if you want to skip this offset trial.",
            ],
        ),
    ]

    sample_index = 0
    if not object_labels or args.contact_trials_per_object <= 0:
        print("no contact objects requested; skipping contact collection")
        return sample_index

    total_steps = len(object_labels) * args.contact_trials_per_object * len(trial_kinds) * len(profiles)
    step = 1
    previous_object = None
    for object_label in object_labels:
        if previous_object is not None:
            write_open_pose(packet, args)
            print("")
            print("[CHANGE OBJECT] Remove %s, place %s." % (previous_object, object_label))
            print("The gripper is open. Press Enter when ready.")
            try:
                input("> ")
            except EOFError:
                pass
        previous_object = object_label

        for trial_index in range(args.contact_trials_per_object):
            for trial_kind, lines in trial_kinds:
                for profile in profiles:
                    write_open_pose(packet, args)
                    response = prompt_plain_step(
                        step,
                        total_steps,
                        "Place object: %s / %s / trial %s / profile %s (%s,%s,%s)"
                        % (
                            object_label,
                            trial_kind,
                            trial_index + 1,
                            profile["profile_name"],
                            profile["speed"],
                            profile["acc"],
                            profile["torque_limit"],
                        ),
                        lines
                        + [
                            "Propellers must be removed or motors disabled.",
                            "The terminal will not refresh while waiting for this input.",
                        ],
                    )
                    step += 1
                    if response == "s":
                        print(
                            "skipped object=%s kind=%s trial=%s profile=%s"
                            % (object_label, trial_kind, trial_index + 1, profile["profile_name"])
                        )
                        continue
                    sample_index = collect_one_contact_trial(
                        packet,
                        writer,
                        csv_file,
                        args,
                        attitude,
                        table,
                        object_label,
                        trial_kind,
                        trial_index + 1,
                        profile,
                        sample_index,
                    )
                    write_open_pose(packet, args)
    return sample_index


def load_baseline_table_for_session(args):
    baseline_path = args.baseline_json
    if not baseline_path:
        baseline_path = args.output_json
    if not baseline_path or not Path(baseline_path).expanduser().is_file():
        raise RuntimeError("--skip-empty requires --baseline-json or an existing --output-json")
    with open(Path(baseline_path).expanduser()) as json_file:
        table = json.load(json_file)
    table = copy.deepcopy(table)
    print("loaded baseline table: %s" % Path(baseline_path).expanduser())
    return table


def calibrate_session(args):
    if args.interactive_ui != "plain":
        raise RuntimeError("only --interactive-ui plain is currently supported")
    apply_collect_full_limits_config(args)
    validate_full_collect_args(args)
    profiles = parse_profiles(args.profiles)
    opening_profiles = parse_opening_profiles(args)
    if args.contact_profiles.strip().lower() == "same":
        contact_profiles = profiles
    elif args.contact_profiles.strip().lower() == "legacy":
        contact_profiles = [
            {
                "profile_index": 0,
                "profile_name": "legacy_contact",
                "speed": args.contact_speed,
                "acc": args.contact_acc,
                "torque_limit": args.contact_torque_limit,
                "label": "%s:%s:%s" % (args.contact_speed, args.contact_acc, args.contact_torque_limit),
            }
        ]
    else:
        contact_profiles = parse_profiles(args.contact_profiles)
    load_sdk(args.sdk_root)

    ensure_parent(args.output_json)
    ensure_parent(args.output_csv)
    ensure_parent(args.contact_csv)
    if args.curve_csv:
        ensure_parent(args.curve_csv)

    attitude = AttitudeReader(
        args.attitude_source,
        args.attitude_topic,
        args.roll_deg,
        args.pitch_deg,
        args.yaw_deg,
    )
    attitude.start()

    port = PortHandler(args.port)
    packet = hls(port)

    if not port.openPort():
        raise RuntimeError("failed to open port %s" % args.port)
    if not port.setBaudRate(args.baud):
        raise RuntimeError("failed to set baudrate %s" % args.baud)

    table = None
    try:
        setup_servo_position_mode(packet, args.left_id)
        setup_servo_position_mode(packet, args.right_id)
        write_open_pose(packet, args)

        if args.skip_empty:
            table = load_baseline_table_for_session(args)
        else:
            prompt_plain_step(
                1,
                1,
                "Prepare empty gripper calibration",
                [
                    "Remove all objects from the gripper.",
                    "Secure the aircraft and remove propellers or disable motors.",
                    "This step will sweep the empty gripper through all no-load profiles.",
                ],
                allow_skip=False,
            )
            with open(args.output_csv, "w", newline="") as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=DEFAULT_FIELDS)
                writer.writeheader()
                write_full_range_samples(
                    packet,
                    writer,
                    csv_file,
                    args,
                    attitude,
                    profiles,
                    opening_profiles,
                    sample_index=0,
                )
            print("wrote empty raw samples: %s" % args.output_csv)
            empty_rows = read_csv_rows(args.output_csv)
            table = build_compensation_table(empty_rows, args, args.output_csv)
            table.setdefault("collection", {})
            table["collection"]["profiles"] = profiles
            table["collection"]["opening_profiles"] = opening_profiles
            table["collection"]["empty_csv"] = os.path.abspath(args.output_csv)
            write_compensation_outputs(table, args.output_json, args.curve_csv)

        object_labels = parse_object_labels(args.object_labels)
        if object_labels and args.contact_trials_per_object > 0:
            with open(args.contact_csv, "w", newline="") as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=DEFAULT_FIELDS)
                writer.writeheader()
                collect_contact_samples(packet, writer, csv_file, args, attitude, table, contact_profiles)
            print("wrote contact raw samples: %s" % args.contact_csv)
            contact_rows = read_csv_rows(args.contact_csv)
            table["contact_detection"] = fit_contact_detection(table, contact_rows, args, args.contact_csv)
            table.setdefault("collection", {})
            table["collection"]["contact_csv"] = os.path.abspath(args.contact_csv)
            table["collection"]["object_labels"] = object_labels
            table["collection"]["contact_profiles"] = contact_profiles
            table["runtime_usage"] = dict(table.get("runtime_usage", {}))
            table["runtime_usage"][
                "contact_metric"
            ] = "contact_metric_sign * (measured_current - baseline_current), with enter/exit/strong thresholds from contact_detection"
            if table["contact_detection"].get("warnings"):
                print("contact detection warnings:")
                for warning in table["contact_detection"]["warnings"]:
                    print("  - %s" % warning)
        else:
            print("no contact labels supplied; final JSON will contain no contact_detection section")

        write_compensation_outputs(table, args.output_json, args.curve_csv)
        write_open_pose(packet, args)

        if args.disable_torque_at_end:
            for servo_id in (args.left_id, args.right_id):
                result, error = packet.EnableTorque(servo_id, 0)
                check(packet, result, error, "disable torque id=%s" % servo_id)

    finally:
        try:
            if not args.disable_torque_at_end:
                write_open_pose(packet, args)
        except Exception as exc:
            print("warning: failed to open gripper during cleanup: %s" % exc)
        attitude.close()
        port.closePort()


def write_curve_csv(path, table):
    ensure_parent(path)
    fields = [
        "side",
        "id",
        "direction",
        "profile_index",
        "profile_name",
        "speed_cmd",
        "acc_cmd",
        "torque_limit",
        "close_ratio",
        "roll_bin_deg",
        "pitch_bin_deg",
        "roll_median_deg",
        "pitch_median_deg",
        "pos_median",
        "current_median",
        "current_mad",
        "current_q05",
        "current_q50",
        "current_q95",
        "current_q99",
        "current_low",
        "current_high",
        "load_median",
        "load_mad",
        "speed_abs_median",
        "samples",
    ]
    with open(path, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        for side in ("left", "right"):
            servo = table["servos"][side]
            for point in servo["points"]:
                row = dict(point)
                row["side"] = side
                row["id"] = servo["id"]
                writer.writerow(row)


def add_collect_args(subparsers):
    parser = subparsers.add_parser("collect", help="collect no-load servo feedback samples")
    parser.add_argument("--port", default="/dev/ttyACM1")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument(
        "--sdk-root",
        default="",
        help=(
            "Path containing scservo_sdk. Defaults to the FT-servo reference SDK "
            "next to this ROS2 workspace."
        ),
    )
    parser.add_argument("--left-id", type=int, default=1)
    parser.add_argument("--right-id", type=int, default=2)
    parser.add_argument("--left-open", type=int, required=True)
    parser.add_argument("--left-close", type=int, required=True)
    parser.add_argument("--right-open", type=int, required=True)
    parser.add_argument("--right-close", type=int, required=True)
    parser.add_argument("--left-inward-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--right-inward-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--sweep-points", type=int, default=41)
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--speed", type=int, default=10)
    parser.add_argument("--acc", type=int, default=5)
    parser.add_argument("--torque-limit", type=int, default=120)
    parser.add_argument("--settle-time", type=float, default=0.5)
    parser.add_argument("--sample-duration", type=float, default=0.35)
    parser.add_argument("--sample-hz", type=float, default=30.0)
    parser.add_argument("--max-current", type=int, default=0)
    parser.add_argument("--max-temp", type=int, default=70)
    parser.add_argument("--attitude-source", choices=("manual", "none", "ros-imu", "ros-pose"), default="manual")
    parser.add_argument("--attitude-topic", default="/mavros/imu/data")
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=0.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--park-ratio", type=float, default=0.0)
    parser.add_argument("--no-park", action="store_true")
    parser.add_argument("--disable-torque-at-end", action="store_true")
    parser.add_argument("--output", required=True)
    parser.set_defaults(func=collect)


def add_collect_full_args(subparsers):
    parser = subparsers.add_parser("collect-full", help="collect three-stage full-range no-load samples and fit JSON")
    parser.add_argument("--port", default="/dev/ttyACM1")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument(
        "--sdk-root",
        default="",
        help=(
            "Path containing scservo_sdk. Defaults to the FT-servo reference SDK "
            "next to this ROS2 workspace."
        ),
    )
    parser.add_argument("--left-id", type=int, default=1)
    parser.add_argument("--right-id", type=int, default=2)
    parser.add_argument(
        "--limits-json",
        default=str(DEFAULT_LIMITS_PATH),
        help="JSON file containing default open/clear/max positions for collect-full.",
    )
    parser.add_argument("--left-open", type=int, default=None)
    parser.add_argument("--left-clear", type=int, default=None)
    parser.add_argument("--left-max", type=int, default=None)
    parser.add_argument("--right-open", type=int, default=None)
    parser.add_argument("--right-clear", type=int, default=None)
    parser.add_argument("--right-max", type=int, default=None)
    parser.add_argument("--left-inward-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--right-inward-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--sweep-points-clear", type=int, default=31)
    parser.add_argument("--sweep-points-extension", type=int, default=21)
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument(
        "--profiles",
        default="",
        help="Optional speed:acc:torque_limit list. If omitted, --speed/--acc/--torque-limit are used.",
    )
    parser.add_argument("--empty-sample-mode", choices=("moving", "settled"), default="moving")
    parser.add_argument(
        "--empty-sweep-direction",
        choices=("closing", "bidirectional"),
        default="bidirectional",
        help="closing repeats inward sweeps; bidirectional alternates inward/outward sweeps.",
    )
    parser.add_argument(
        "--opening-profiles",
        default="40:10:300",
        help="Opening sweep speed:acc:torque_limit list. Defaults to the runtime open profile.",
    )
    parser.add_argument("--speed", type=int, default=8)
    parser.add_argument("--acc", type=int, default=4)
    parser.add_argument("--torque-limit", type=int, default=80)
    parser.add_argument("--inter-segment-open-speed", type=int, default=5)
    parser.add_argument("--inter-segment-open-acc", type=int, default=3)
    parser.add_argument("--inter-segment-open-torque-limit", type=int, default=80)
    parser.add_argument("--inter-segment-open-settle-time", type=float, default=0.8)
    parser.add_argument("--inter-segment-open-timeout", type=float, default=12.0)
    parser.add_argument("--move-timeout", type=float, default=12.0)
    parser.add_argument("--position-tolerance", type=float, default=35.0)
    parser.add_argument("--allow-unsettled", action="store_true")
    parser.add_argument("--settle-time", type=float, default=0.5)
    parser.add_argument("--sample-duration", type=float, default=0.35)
    parser.add_argument("--sample-hz", type=float, default=30.0)
    parser.add_argument("--max-current", type=int, default=0)
    parser.add_argument("--max-temp", type=int, default=70)
    parser.add_argument("--attitude-source", choices=("manual", "none", "ros-imu", "ros-pose"), default="manual")
    parser.add_argument("--attitude-topic", default="/mavros/imu/data")
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=0.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--park-ratio", type=float, default=0.0)
    parser.add_argument("--no-park", action="store_true")
    parser.add_argument("--disable-torque-at-end", action="store_true")
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--curve-csv", default="")
    parser.add_argument("--position-bins", type=int, default=41)
    parser.add_argument("--roll-bin-size-deg", type=float, default=5.0)
    parser.add_argument("--pitch-bin-size-deg", type=float, default=5.0)
    parser.add_argument("--max-abs-speed", type=float, default=-1.0)
    parser.add_argument("--min-samples", type=int, default=3)
    parser.set_defaults(func=collect_full)


def add_calibrate_session_args(subparsers):
    parser = subparsers.add_parser(
        "calibrate-session",
        help="interactive no-load plus contact calibration session",
    )
    parser.add_argument("--port", default="/dev/ttyACM1")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument(
        "--sdk-root",
        default="",
        help=(
            "Path containing scservo_sdk. Defaults to the FT-servo reference SDK "
            "next to this ROS2 workspace."
        ),
    )
    parser.add_argument("--left-id", type=int, default=1)
    parser.add_argument("--right-id", type=int, default=2)
    parser.add_argument(
        "--limits-json",
        default=str(DEFAULT_LIMITS_PATH),
        help="JSON file containing default open/clear/max positions for calibration.",
    )
    parser.add_argument("--left-open", type=int, default=None)
    parser.add_argument("--left-clear", type=int, default=None)
    parser.add_argument("--left-max", type=int, default=None)
    parser.add_argument("--right-open", type=int, default=None)
    parser.add_argument("--right-clear", type=int, default=None)
    parser.add_argument("--right-max", type=int, default=None)
    parser.add_argument("--left-inward-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--right-inward-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--sweep-points-clear", type=int, default=31)
    parser.add_argument("--sweep-points-extension", type=int, default=21)
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--profiles", default="5:3:90,10:5:120,20:8:150")
    parser.add_argument("--empty-sample-mode", choices=("moving", "settled"), default="moving")
    parser.add_argument(
        "--empty-sweep-direction",
        choices=("closing", "bidirectional"),
        default="bidirectional",
        help="closing repeats inward sweeps; bidirectional alternates inward/outward sweeps.",
    )
    parser.add_argument(
        "--opening-profiles",
        default="40:10:300",
        help="Opening sweep speed:acc:torque_limit list. Defaults to the runtime open profile.",
    )
    parser.add_argument("--inter-segment-open-speed", type=int, default=5)
    parser.add_argument("--inter-segment-open-acc", type=int, default=3)
    parser.add_argument("--inter-segment-open-torque-limit", type=int, default=90)
    parser.add_argument("--inter-segment-open-settle-time", type=float, default=0.8)
    parser.add_argument("--inter-segment-open-timeout", type=float, default=12.0)
    parser.add_argument("--move-timeout", type=float, default=12.0)
    parser.add_argument("--position-tolerance", type=float, default=35.0)
    parser.add_argument("--allow-unsettled", action="store_true")
    parser.add_argument("--settle-time", type=float, default=0.5)
    parser.add_argument("--sample-duration", type=float, default=0.35)
    parser.add_argument("--sample-hz", type=float, default=30.0)
    parser.add_argument("--max-current", type=int, default=0)
    parser.add_argument("--max-temp", type=int, default=70)
    parser.add_argument("--attitude-source", choices=("manual", "none", "ros-imu", "ros-pose"), default="manual")
    parser.add_argument("--attitude-topic", default="/mavros/imu/data")
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=0.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--open-speed", type=int, default=40)
    parser.add_argument("--open-acc", type=int, default=10)
    parser.add_argument("--open-torque-limit", type=int, default=300)
    parser.add_argument("--open-settle-time", type=float, default=0.6)
    parser.add_argument("--contact-speed", type=int, default=5)
    parser.add_argument("--contact-acc", type=int, default=3)
    parser.add_argument("--contact-torque-limit", type=int, default=80)
    parser.add_argument(
        "--contact-profiles",
        default="same",
        help="'same' uses --profiles, 'legacy' uses --contact-speed/acc/torque-limit, or pass speed:acc:torque,...",
    )
    parser.add_argument("--contact-close-ratio", type=float, default=1.0)
    parser.add_argument("--contact-sample-duration", type=float, default=2.0)
    parser.add_argument("--contact-sample-hz", type=float, default=30.0)
    parser.add_argument("--contact-fit-min-ratio", type=float, default=0.20)
    parser.add_argument("--contact-min-enter-threshold", type=float, default=12.0)
    parser.add_argument("--contact-min-exit-threshold", type=float, default=6.0)
    parser.add_argument("--object-labels", default="foam,bottle,box")
    parser.add_argument("--contact-trials-per-object", type=int, default=3)
    parser.add_argument("--interactive-ui", choices=("plain",), default="plain")
    parser.add_argument("--skip-empty", action="store_true")
    parser.add_argument("--baseline-json", default="")
    parser.add_argument("--output-csv", default=str(DEFAULT_SESSION_EMPTY_CSV))
    parser.add_argument("--contact-csv", default=str(DEFAULT_SESSION_CONTACT_CSV))
    parser.add_argument("--output-json", default=str(DEFAULT_SESSION_JSON))
    parser.add_argument("--curve-csv", default=str(DEFAULT_SESSION_CURVE_CSV))
    parser.add_argument("--position-bins", type=int, default=41)
    parser.add_argument("--roll-bin-size-deg", type=float, default=5.0)
    parser.add_argument("--pitch-bin-size-deg", type=float, default=5.0)
    parser.add_argument("--max-abs-speed", type=float, default=-1.0)
    parser.add_argument("--min-samples", type=int, default=3)
    parser.add_argument("--disable-torque-at-end", action="store_true")
    parser.set_defaults(func=calibrate_session)


def add_fit_args(subparsers):
    parser = subparsers.add_parser("fit", help="build a compensation table from collected CSV")
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--curve-csv", default="")
    parser.add_argument("--position-bins", type=int, default=41)
    parser.add_argument("--roll-bin-size-deg", type=float, default=5.0)
    parser.add_argument("--pitch-bin-size-deg", type=float, default=5.0)
    parser.add_argument("--max-abs-speed", type=float, default=10.0)
    parser.add_argument("--min-samples", type=int, default=3)
    parser.set_defaults(func=fit)


def add_read_limits_args(subparsers):
    parser = subparsers.add_parser(
        "read-limits",
        help="read HLS present positions and stored angle limit registers without writing to servos",
    )
    parser.add_argument("--port", default="/dev/ttyACM1")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument(
        "--sdk-root",
        default="",
        help=(
            "Path containing scservo_sdk. Defaults to the FT-servo reference SDK "
            "next to this ROS2 workspace."
        ),
    )
    parser.add_argument("--left-id", type=int, default=1)
    parser.add_argument("--right-id", type=int, default=2)
    parser.add_argument("--output-json", default="")
    parser.add_argument("--watch", action="store_true", help="keep printing present positions until Ctrl+C")
    parser.add_argument("--watch-period", type=float, default=2.0, help="seconds between watch updates")
    parser.set_defaults(func=read_limits)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Collect and fit HLS two-finger gripper gravity compensation data."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    add_read_limits_args(subparsers)
    add_collect_args(subparsers)
    add_collect_full_args(subparsers)
    add_calibrate_session_args(subparsers)
    add_fit_args(subparsers)
    return parser.parse_args()


def main():
    args = parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
