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
    "right_pos",
    "right_close_ratio",
    "right_speed",
    "right_load",
    "right_voltage",
    "right_temp",
    "right_moving",
    "right_current",
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

    result, error = packet.EnableTorque(servo_id, 0)
    check(packet, result, error, "disable torque id=%s" % servo_id)

    result, error = packet.ServoMode(servo_id)
    check(packet, result, error, "set ServoMode id=%s" % servo_id)

    result, error = packet.EnableTorque(servo_id, 1)
    check(packet, result, error, "enable torque id=%s" % servo_id)


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
        self.node.create_subscription(MessageType, self.topic, self._ros_callback, 10)
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
                        "cycle=%s direction=%s ratio=%.3f left_pos=%s right_pos=%s"
                        % (cycle, direction, target_ratio, left_fb["pos"], right_fb["pos"])
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


def collect_full(args):
    apply_collect_full_limits_config(args)
    load_sdk(args.sdk_root)
    ensure_parent(args.output_csv)
    ensure_parent(args.output_json)
    if args.curve_csv:
        ensure_parent(args.curve_csv)

    left_clear_ratio, right_clear_ratio = validate_full_collect_args(args)

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

    segments = [
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
            "name": "left_extension",
            "left_start": left_clear_ratio,
            "left_stop": 1.0,
            "right_start": 0.0,
            "right_stop": 0.0,
            "left_active": 1,
            "right_active": 0,
            "points": args.sweep_points_extension,
        },
        {
            "name": "right_extension",
            "left_start": 0.0,
            "left_stop": 0.0,
            "right_start": right_clear_ratio,
            "right_stop": 1.0,
            "left_active": 0,
            "right_active": 1,
            "points": args.sweep_points_extension,
        },
    ]

    try:
        setup_servo_position_mode(packet, args.left_id)
        setup_servo_position_mode(packet, args.right_id)

        with open(args.output_csv, "w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=DEFAULT_FIELDS)
            writer.writeheader()

            sample_index = 0
            for segment in segments:
                for cycle in range(args.cycles):
                    left_ratios = make_ratio_sequence_between(
                        segment["left_start"], segment["left_stop"], segment["points"], cycle
                    )
                    right_ratios = make_ratio_sequence_between(
                        segment["right_start"], segment["right_stop"], segment["points"], cycle
                    )
                    direction = "closing" if cycle % 2 == 0 else "opening"
                    for left_ratio, right_ratio in zip(left_ratios, right_ratios):
                        target_left = ratio_to_pos(args.left_open, args.left_max, left_ratio)
                        target_right = ratio_to_pos(args.right_open, args.right_max, right_ratio)
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
                            active_ratios = []
                            if segment["left_active"]:
                                active_ratios.append(left_ratio)
                            if segment["right_active"]:
                                active_ratios.append(right_ratio)
                            target_ratio = sum(active_ratios) / float(len(active_ratios))
                            row = {
                                "schema_version": SCHEMA_VERSION,
                                "utc_time": utc_now(),
                                "monotonic_s": "%.6f" % time.monotonic(),
                                "cycle": cycle,
                                "direction": direction,
                                "segment": segment["name"],
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
                            writer.writerow(row)
                            csv_file.flush()
                            sample_index += 1
                            time.sleep(period)

                        print(
                            "segment=%s cycle=%s direction=%s left_ratio=%.3f right_ratio=%.3f "
                            "left_pos=%s right_pos=%s"
                            % (
                                segment["name"],
                                cycle,
                                direction,
                                left_ratio,
                                right_ratio,
                                left_fb["pos"],
                                right_fb["pos"],
                            )
                        )

        if not args.no_park:
            park_ratio = clamp(args.park_ratio, 0.0, 1.0)
            write_position(
                packet,
                args.left_id,
                ratio_to_pos(args.left_open, args.left_max, park_ratio),
                args.speed,
                args.acc,
                args.torque_limit,
            )
            write_position(
                packet,
                args.right_id,
                ratio_to_pos(args.right_open, args.right_max, park_ratio),
                args.speed,
                args.acc,
                args.torque_limit,
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

    points = []
    skipped_samples = 0
    for key, bucket in buckets.items():
        sample_count = len(bucket["current"])
        if sample_count < args.min_samples:
            skipped_samples += sample_count
            continue
        current_median = statistics.median(bucket["current"])
        load_median = statistics.median(bucket["load"]) if bucket["load"] else None
        pos_median = statistics.median(bucket["pos"]) if bucket["pos"] else None
        speed_abs_median = statistics.median(bucket["abs_speed"]) if bucket["abs_speed"] else None
        points.append(
            {
                "close_ratio": key[0],
                "roll_bin_deg": key[1],
                "pitch_bin_deg": key[2],
                "roll_median_deg": statistics.median(bucket["roll"]) if bucket["roll"] else None,
                "pitch_median_deg": statistics.median(bucket["pitch"]) if bucket["pitch"] else None,
                "pos_median": pos_median,
                "current_median": current_median,
                "current_mad": median_abs_deviation(bucket["current"], current_median),
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


def write_curve_csv(path, table):
    ensure_parent(path)
    fields = [
        "side",
        "id",
        "close_ratio",
        "roll_bin_deg",
        "pitch_bin_deg",
        "roll_median_deg",
        "pitch_median_deg",
        "pos_median",
        "current_median",
        "current_mad",
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
    parser.add_argument("--speed", type=int, default=8)
    parser.add_argument("--acc", type=int, default=4)
    parser.add_argument("--torque-limit", type=int, default=80)
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
    parser.add_argument("--max-abs-speed", type=float, default=10.0)
    parser.add_argument("--min-samples", type=int, default=3)
    parser.set_defaults(func=collect_full)


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
    add_fit_args(subparsers)
    return parser.parse_args()


def main():
    args = parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
