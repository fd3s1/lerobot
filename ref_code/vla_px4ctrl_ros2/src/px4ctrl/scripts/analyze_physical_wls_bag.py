#!/usr/bin/python3

from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import Counter
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import re
import struct
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


PHYSICAL_TOPIC = "/mavros/tunnel/in"
ATTITUDE_TOPIC = "/mavros/setpoint_raw/attitude"
MOCAP_POSE_TOPIC = "/mavros/vision_pose/pose"
MOCAP_TWIST_TOPIC = "/vla_drone1/twist"
COMBINED_STATE_TOPIC = "/px4ctrl/simulink/actual_state"
MOCAP_STATUS_TOPIC = "/px4ctrl/mocap_state_status"
PAYLOAD_TYPE = 42001
PAYLOAD_MAGIC = b"PCTL"
PAYLOAD_SIZES = {1: 52, 2: 64}
REQUIRED_FLAGS = {1: 0x7, 2: 0xF}
ANGULAR_ACCELERATION_LIMIT = (120.0, 120.0, 60.0)
HEADER = struct.Struct("<4sBBHIQ")
VALUES_V1 = struct.Struct("<8f")
ANGULAR_ACCELERATION = struct.Struct("<3f")

EXPECTED_TOPICS = (
    PHYSICAL_TOPIC,
    ATTITUDE_TOPIC,
    "/mavros/local_position/odom",
    "/mavros/imu/data",
    MOCAP_POSE_TOPIC,
    MOCAP_TWIST_TOPIC,
    "/mavros/battery",
    "/mavros/state",
    "/px4ctrl/state",
    COMBINED_STATE_TOPIC,
    MOCAP_STATUS_TOPIC,
    "/px4ctrl/simulink/ude_debug",
)


@dataclass
class TopicStats:
    count: int = 0
    first_ns: int = 0
    last_ns: int = 0

    def add(self, timestamp_ns: int) -> None:
        if self.count == 0:
            self.first_ns = timestamp_ns
        self.last_ns = timestamp_ns
        self.count += 1

    @property
    def duration_s(self) -> float:
        return max(0.0, (self.last_ns - self.first_ns) * 1e-9)

    @property
    def rate_hz(self) -> float:
        if self.count < 2 or self.duration_s <= 0.0:
            return 0.0
        return (self.count - 1) / self.duration_s


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round(fraction * (len(ordered) - 1))))
    return ordered[index]


def nearest_offsets_ms(reference_ns: list[int], candidates_ns: list[int]) -> list[float]:
    if not reference_ns or not candidates_ns:
        return []
    offsets = []
    for stamp in reference_ns:
        index = bisect_left(candidates_ns, stamp)
        differences = []
        if index < len(candidates_ns):
            differences.append(abs(candidates_ns[index] - stamp))
        if index > 0:
            differences.append(abs(candidates_ns[index - 1] - stamp))
        offsets.append(min(differences) * 1e-6)
    return offsets


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate px4ctrl physical TUNNEL traffic in a ROS2 bag."
    )
    parser.add_argument("bag", type=Path)
    parser.add_argument("--storage-id", default="sqlite3")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    bag_path = args.bag.expanduser().resolve()
    if not bag_path.exists():
        print(f"FAIL: bag does not exist: {bag_path}", file=sys.stderr)
        return 2

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=args.storage_id),
            rosbag2_py.ConverterOptions("", ""),
        )
    except RuntimeError as exc:
        print(f"FAIL: cannot open bag {bag_path}: {exc}", file=sys.stderr)
        return 2

    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    tunnel_type = topic_types.get(PHYSICAL_TOPIC)
    attitude_type = topic_types.get(ATTITUDE_TOPIC)
    tunnel_class = get_message(tunnel_type) if tunnel_type else None
    attitude_class = get_message(attitude_type) if attitude_type else None
    combined_state_type = topic_types.get(COMBINED_STATE_TOPIC)
    mocap_status_type = topic_types.get(MOCAP_STATUS_TOPIC)
    combined_state_class = get_message(combined_state_type) if combined_state_type else None
    mocap_status_class = get_message(mocap_status_type) if mocap_status_type else None

    stats: dict[str, TopicStats] = {}
    physical_stamps: list[int] = []
    attitude_stamps: list[int] = []
    sequence_previous: int | None = None
    sequence_lost = 0
    sequence_duplicates = 0
    sequence_reordered = 0
    invalid = Counter()
    modes = Counter()
    versions = Counter()
    thrust_values: list[float] = []
    quaternion_norms: list[float] = []
    body_rate_dot_values: list[tuple[float, float, float]] = []
    inactive_physical_samples = 0
    combined_state_frames = Counter()
    mocap_status_reasons = Counter()
    pose_twist_dt_values: list[float] = []
    mocap_odom_dt_values: list[float] = []
    pose_to_odom_dt_values: list[float] = []
    twist_to_odom_dt_values: list[float] = []
    prediction_dt_values: list[float] = []
    prediction_weight_values: list[float] = []
    position_correction_norms: list[float] = []
    prediction_valid_count = 0
    source_transition_count = 0

    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        stats.setdefault(topic, TopicStats()).add(timestamp_ns)

        if topic == COMBINED_STATE_TOPIC and combined_state_class is not None:
            try:
                message = deserialize_message(serialized, combined_state_class)
                combined_state_frames[message.child_frame_id] += 1
            except Exception:
                invalid["combined_state_deserialize"] += 1
            continue

        if topic == MOCAP_STATUS_TOPIC and mocap_status_class is not None:
            try:
                message = deserialize_message(serialized, mocap_status_class)
                status_text = message.data
            except Exception:
                invalid["mocap_status_deserialize"] += 1
                continue
            reason_match = re.search(r"reason='([^']*)'", status_text)
            reason = reason_match.group(1) if reason_match else "unparsed"
            mocap_status_reasons[reason] += 1
            pair_match = re.search(r"pose_twist_dt=([-+0-9.eE]+)", status_text)
            odom_match = re.search(r"mocap_odom_dt=([-+0-9.eE]+)", status_text)
            pose_odom_match = re.search(r"pose_to_odom_dt=([-+0-9.eE]+)", status_text)
            twist_odom_match = re.search(r"twist_to_odom_dt=([-+0-9.eE]+)", status_text)
            prediction_dt_match = re.search(r"prediction_dt=([-+0-9.eE]+)", status_text)
            prediction_weight_match = re.search(
                r"prediction_weight=([-+0-9.eE]+)", status_text
            )
            prediction_valid_match = re.search(r"prediction_valid=(true|false)", status_text)
            source_transition_match = re.search(
                r"source_transition=(true|false)", status_text
            )
            position_correction_match = re.search(
                r"position_correction=\[([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)\]",
                status_text,
            )
            if pair_match:
                pose_twist_dt_values.append(float(pair_match.group(1)))
            if odom_match:
                mocap_odom_dt_values.append(float(odom_match.group(1)))
            if pose_odom_match:
                pose_to_odom_dt_values.append(float(pose_odom_match.group(1)))
            if twist_odom_match:
                twist_to_odom_dt_values.append(float(twist_odom_match.group(1)))
            if prediction_dt_match:
                prediction_dt_values.append(float(prediction_dt_match.group(1)))
            if prediction_weight_match:
                prediction_weight_values.append(float(prediction_weight_match.group(1)))
            if prediction_valid_match and prediction_valid_match.group(1) == "true":
                prediction_valid_count += 1
            if source_transition_match and source_transition_match.group(1) == "true":
                source_transition_count += 1
            if position_correction_match:
                correction = tuple(
                    float(position_correction_match.group(index)) for index in range(1, 4)
                )
                position_correction_norms.append(
                    math.sqrt(sum(component * component for component in correction))
                )
            continue

        if topic == ATTITUDE_TOPIC:
            attitude_stamps.append(timestamp_ns)
            if attitude_class is not None:
                try:
                    deserialize_message(serialized, attitude_class)
                except Exception:
                    invalid["attitude_deserialize"] += 1
            continue

        if topic != PHYSICAL_TOPIC or tunnel_class is None:
            continue

        physical_stamps.append(timestamp_ns)
        try:
            message = deserialize_message(serialized, tunnel_class)
        except Exception:
            invalid["tunnel_deserialize"] += 1
            continue

        if int(message.payload_type) != PAYLOAD_TYPE:
            invalid["payload_type"] += 1
            continue
        payload_length = int(message.payload_length)
        if payload_length < HEADER.size:
            invalid["payload_length"] += 1
            continue

        payload = bytes(message.payload[:payload_length])
        if len(payload) != payload_length:
            invalid["payload_buffer"] += 1
            continue

        magic, version, mode, flags, sequence, source_time_us = HEADER.unpack_from(payload, 0)
        expected_size = PAYLOAD_SIZES.get(version)
        if expected_size is None:
            invalid["version"] += 1
            continue
        versions[str(version)] += 1
        if payload_length != expected_size:
            invalid["payload_length"] += 1
            continue
        values = VALUES_V1.unpack_from(payload, HEADER.size)
        quaternion = values[0:4]
        body_rate = values[4:7]
        total_thrust_n = values[7]
        body_rate_dot = (0.0, 0.0, 0.0)
        if version == 2:
            body_rate_dot = ANGULAR_ACCELERATION.unpack_from(
                payload, HEADER.size + VALUES_V1.size
            )
            body_rate_dot_values.append(body_rate_dot)

        if magic != PAYLOAD_MAGIC:
            invalid["magic"] += 1
        if mode not in (0, 1):
            invalid["mode"] += 1
        modes[str(mode)] += 1
        if flags & 0x4:
            if (flags & REQUIRED_FLAGS[version]) != REQUIRED_FLAGS[version]:
                invalid["required_flags"] += 1
        else:
            inactive_physical_samples += 1
        if source_time_us == 0:
            invalid["source_time"] += 1
        if not all(math.isfinite(value) for value in values):
            invalid["nonfinite"] += 1

        q_norm = math.sqrt(sum(value * value for value in quaternion))
        quaternion_norms.append(q_norm)
        if not math.isfinite(q_norm) or abs(q_norm - 1.0) > 0.02:
            invalid["quaternion_norm"] += 1
        if not all(math.isfinite(value) for value in body_rate):
            invalid["body_rate"] += 1
        if not all(math.isfinite(value) for value in body_rate_dot):
            invalid["body_rate_dot"] += 1
        elif any(
            abs(value) > limit
            for value, limit in zip(body_rate_dot, ANGULAR_ACCELERATION_LIMIT)
        ):
            invalid["body_rate_dot_limit"] += 1
        thrust_values.append(total_thrust_n)
        if not math.isfinite(total_thrust_n) or not 0.0 <= total_thrust_n <= 100.0:
            invalid["physical_thrust"] += 1

        if sequence_previous is not None:
            delta = (sequence - sequence_previous) & 0xFFFFFFFF
            if delta == 0:
                sequence_duplicates += 1
            elif delta < 0x80000000:
                sequence_lost += max(0, delta - 1)
            else:
                sequence_reordered += 1
        sequence_previous = sequence

    errors: list[str] = []
    warnings: list[str] = []
    physical_stats = stats.get(PHYSICAL_TOPIC, TopicStats())
    attitude_stats = stats.get(ATTITUDE_TOPIC, TopicStats())

    if physical_stats.count == 0:
        errors.append(f"{PHYSICAL_TOPIC} is missing")
    elif physical_stats.duration_s >= 2.0:
        if physical_stats.rate_hz < 20.0:
            errors.append(f"physical setpoint rate is only {physical_stats.rate_hz:.1f} Hz")
        elif physical_stats.rate_hz < 80.0:
            warnings.append(f"physical setpoint rate is below 80 Hz: {physical_stats.rate_hz:.1f} Hz")

    if attitude_stats.count == 0:
        errors.append(f"{ATTITUDE_TOPIC} normalized backup is missing")
    elif attitude_stats.duration_s >= 2.0 and attitude_stats.rate_hz < 20.0:
        errors.append(f"normalized backup rate is only {attitude_stats.rate_hz:.1f} Hz")

    for topic, fail_rate, warn_rate in (
        ("/mavros/local_position/odom", 10.0, 20.0),
        ("/mavros/imu/data", 20.0, 50.0),
        (MOCAP_POSE_TOPIC, 10.0, 20.0),
        (MOCAP_TWIST_TOPIC, 10.0, 20.0),
        (COMBINED_STATE_TOPIC, 20.0, 50.0),
    ):
        item = stats.get(topic, TopicStats())
        if item.count == 0:
            errors.append(f"{topic} is missing")
        elif item.duration_s >= 2.0 and item.rate_hz < fail_rate:
            errors.append(f"{topic} rate is only {item.rate_hz:.1f} Hz")
        elif item.duration_s >= 2.0 and item.rate_hz < warn_rate:
            warnings.append(f"{topic} rate is low: {item.rate_hz:.1f} Hz")

    for topic in ("/mavros/state", "/px4ctrl/state", "/mavros/battery"):
        if stats.get(topic, TopicStats()).count == 0:
            warnings.append(f"{topic} is missing")

    mocap_status_count = sum(mocap_status_reasons.values())
    mocap_synced_count = mocap_status_reasons.get("mocap_synced", 0)
    if mocap_status_count == 0:
        errors.append(f"{MOCAP_STATUS_TOPIC} is missing; composed-state source cannot be verified")
    else:
        fallback_count = mocap_status_count - mocap_synced_count
        fallback_fraction = fallback_count / mocap_status_count
        if fallback_fraction > 0.05:
            errors.append(
                f"combined feedback fell back from mocap for {100.0 * fallback_fraction:.1f}% "
                f"of status samples ({fallback_count}/{mocap_status_count})"
            )
        elif fallback_count:
            warnings.append(
                f"combined feedback had {fallback_count} non-mocap status samples "
                f"({100.0 * fallback_fraction:.2f}%)"
            )

    combined_count = sum(combined_state_frames.values())
    expected_frame = "actual_state_mocap_synced_pv_odom_attitude"
    expected_combined_count = combined_state_frames.get(expected_frame, 0)
    if combined_count == 0:
        errors.append(f"{COMBINED_STATE_TOPIC} is missing")
    elif expected_combined_count != combined_count:
        fallback_count = combined_count - expected_combined_count
        fallback_fraction = fallback_count / combined_count
        message = (
            f"combined-state output used fallback frames for {100.0 * fallback_fraction:.2f}% "
            f"of samples ({fallback_count}/{combined_count})"
        )
        if fallback_fraction > 0.05:
            errors.append(message)
        else:
            warnings.append(message)

    if invalid:
        errors.append("invalid physical samples: " + ", ".join(f"{k}={v}" for k, v in sorted(invalid.items())))

    received_with_loss = physical_stats.count + sequence_lost
    loss_fraction = sequence_lost / received_with_loss if received_with_loss else 0.0
    if loss_fraction > 0.01:
        errors.append(f"TUNNEL sequence loss is {100.0 * loss_fraction:.2f}% ({sequence_lost} samples)")
    elif sequence_lost:
        warnings.append(f"TUNNEL sequence lost {sequence_lost} samples ({100.0 * loss_fraction:.3f}%)")
    if sequence_duplicates:
        warnings.append(f"TUNNEL sequence has {sequence_duplicates} duplicates")
    if sequence_reordered:
        errors.append(f"TUNNEL sequence has {sequence_reordered} reordered samples")

    sync_offsets = nearest_offsets_ms(physical_stamps, attitude_stamps)
    sync_p95_ms = percentile(sync_offsets, 0.95)
    if sync_offsets and sync_p95_ms > 50.0:
        errors.append(f"physical/backup timestamp nearest-neighbor p95 is {sync_p95_ms:.2f} ms")
    elif sync_offsets and sync_p95_ms > 10.0:
        warnings.append(f"physical/backup timestamp nearest-neighbor p95 is {sync_p95_ms:.2f} ms")

    print(f"Bag: {bag_path}")
    print("Topic statistics:")
    for topic in EXPECTED_TOPICS:
        item = stats.get(topic, TopicStats())
        print(f"  {topic}: count={item.count}, rate={item.rate_hz:.2f} Hz, span={item.duration_s:.2f} s")
    print("Physical protocol:")
    print(
        f"  versions={dict(versions)}, modes={dict(modes)}, lost={sequence_lost}, "
        f"duplicate={sequence_duplicates}, reordered={sequence_reordered}, "
        f"inactive={inactive_physical_samples}"
    )
    if quaternion_norms:
        print(f"  quaternion norm=[{min(quaternion_norms):.6f}, {max(quaternion_norms):.6f}]")
    if thrust_values:
        print(f"  total thrust=[{min(thrust_values):.3f}, {max(thrust_values):.3f}] N")
    if body_rate_dot_values:
        axis_peaks = tuple(
            max(abs(value[axis]) for value in body_rate_dot_values)
            for axis in range(3)
        )
        print(
            "  desired angular acceleration peak="
            f"[{axis_peaks[0]:.3f}, {axis_peaks[1]:.3f}, {axis_peaks[2]:.3f}] rad/s^2"
        )
    if sync_offsets:
        print(f"  physical/backup nearest offset: median={percentile(sync_offsets, 0.5):.3f} ms, p95={sync_p95_ms:.3f} ms")
    print("Combined control feedback (p/v=mocap, q/w=FCU odom):")
    print(f"  state frames={dict(combined_state_frames)}")
    print(f"  mocap status reasons={dict(mocap_status_reasons)}")
    if pose_twist_dt_values:
        print(
            f"  pose/twist dt: median={1000.0 * percentile(pose_twist_dt_values, 0.5):.2f} ms, "
            f"p95={1000.0 * percentile(pose_twist_dt_values, 0.95):.2f} ms"
        )
    if mocap_odom_dt_values:
        print(
            f"  mocap/FCU odom dt: median={1000.0 * percentile(mocap_odom_dt_values, 0.5):.2f} ms, "
            f"p95={1000.0 * percentile(mocap_odom_dt_values, 0.95):.2f} ms"
        )
    if pose_to_odom_dt_values:
        print(
            f"  signed pose->FCU dt: median={1000.0 * percentile(pose_to_odom_dt_values, 0.5):.2f} ms, "
            f"p95={1000.0 * percentile(pose_to_odom_dt_values, 0.95):.2f} ms"
        )
    if prediction_weight_values:
        print(
            f"  prediction weight: median={percentile(prediction_weight_values, 0.5):.3f}, "
            f"p05={percentile(prediction_weight_values, 0.05):.3f}, "
            f"valid={prediction_valid_count}/{len(prediction_weight_values)}, "
            f"source_transition={source_transition_count}"
        )
    if position_correction_norms:
        print(
            f"  position correction norm: median={percentile(position_correction_norms, 0.5):.4f} m, "
            f"p95={percentile(position_correction_norms, 0.95):.4f} m"
        )

    for warning in warnings:
        print(f"WARN: {warning}")
    for error in errors:
        print(f"FAIL: {error}")

    report = {
        "bag": str(bag_path),
        "result": "FAIL" if errors else ("WARN" if warnings else "PASS"),
        "topics": {
            topic: {
                **asdict(item),
                "duration_s": item.duration_s,
                "rate_hz": item.rate_hz,
            }
            for topic, item in stats.items()
        },
        "physical": {
            "versions": dict(versions),
            "modes": dict(modes),
            "invalid": dict(invalid),
            "sequence_lost": sequence_lost,
            "sequence_duplicates": sequence_duplicates,
            "sequence_reordered": sequence_reordered,
            "inactive_samples": inactive_physical_samples,
            "sequence_loss_fraction": loss_fraction,
            "quaternion_norm_min": min(quaternion_norms) if quaternion_norms else None,
            "quaternion_norm_max": max(quaternion_norms) if quaternion_norms else None,
            "total_thrust_min_n": min(thrust_values) if thrust_values else None,
            "total_thrust_max_n": max(thrust_values) if thrust_values else None,
            "body_rate_dot_peak_abs_radps2": [
                max(abs(value[axis]) for value in body_rate_dot_values)
                for axis in range(3)
            ] if body_rate_dot_values else None,
            "backup_sync_median_ms": percentile(sync_offsets, 0.5) if sync_offsets else None,
            "backup_sync_p95_ms": sync_p95_ms if sync_offsets else None,
        },
        "combined_feedback": {
            "definition": "position_velocity_from_mocap_attitude_rates_from_fcu_odom",
            "state_frames": dict(combined_state_frames),
            "mocap_status_reasons": dict(mocap_status_reasons),
            "pose_twist_dt_median_ms": 1000.0 * percentile(pose_twist_dt_values, 0.5) if pose_twist_dt_values else None,
            "pose_twist_dt_p95_ms": 1000.0 * percentile(pose_twist_dt_values, 0.95) if pose_twist_dt_values else None,
            "mocap_odom_dt_median_ms": 1000.0 * percentile(mocap_odom_dt_values, 0.5) if mocap_odom_dt_values else None,
            "mocap_odom_dt_p95_ms": 1000.0 * percentile(mocap_odom_dt_values, 0.95) if mocap_odom_dt_values else None,
            "pose_to_odom_dt_median_ms": 1000.0 * percentile(pose_to_odom_dt_values, 0.5) if pose_to_odom_dt_values else None,
            "pose_to_odom_dt_p95_ms": 1000.0 * percentile(pose_to_odom_dt_values, 0.95) if pose_to_odom_dt_values else None,
            "twist_to_odom_dt_median_ms": 1000.0 * percentile(twist_to_odom_dt_values, 0.5) if twist_to_odom_dt_values else None,
            "twist_to_odom_dt_p95_ms": 1000.0 * percentile(twist_to_odom_dt_values, 0.95) if twist_to_odom_dt_values else None,
            "prediction_dt_median_ms": 1000.0 * percentile(prediction_dt_values, 0.5) if prediction_dt_values else None,
            "prediction_dt_p95_ms": 1000.0 * percentile(prediction_dt_values, 0.95) if prediction_dt_values else None,
            "prediction_weight_median": percentile(prediction_weight_values, 0.5) if prediction_weight_values else None,
            "prediction_weight_p05": percentile(prediction_weight_values, 0.05) if prediction_weight_values else None,
            "prediction_valid_count": prediction_valid_count,
            "prediction_status_count": len(prediction_weight_values),
            "source_transition_count": source_transition_count,
            "position_correction_norm_median_m": percentile(position_correction_norms, 0.5) if position_correction_norms else None,
            "position_correction_norm_p95_m": percentile(position_correction_norms, 0.95) if position_correction_norms else None,
        },
        "warnings": warnings,
        "errors": errors,
    }
    report_path = bag_path / "physical_wls_analysis.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Report: {report_path}")
    print(f"RESULT: {report['result']}")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
