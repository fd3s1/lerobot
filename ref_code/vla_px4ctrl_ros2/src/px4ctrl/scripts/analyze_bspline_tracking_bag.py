#!/usr/bin/python3

from __future__ import annotations

import argparse
from collections import defaultdict
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


PHASE_TOPIC = "/px4ctrl/bspline_test/state"
COMMAND_TOPIC = "/position_cmd_traj"
REFERENCE_TOPIC = "/px4ctrl/simulink/reference_state"
ERROR_TOPIC = "/px4ctrl/simulink/tracking_error"
SPEED_BINS = ("0-1_mps", "1-2_mps", "2-3_mps", "3-4_mps", "4-5_mps", "5plus_mps")


@dataclass
class ErrorSample:
    timestamp_ns: int
    phase: str
    reference_speed_mps: float
    ex: float
    ey: float
    ez: float
    evx: float
    evy: float
    evz: float
    yaw_error_rad: float

    @property
    def position_norm(self) -> float:
        return math.sqrt(self.ex * self.ex + self.ey * self.ey + self.ez * self.ez)

    @property
    def velocity_norm(self) -> float:
        return math.sqrt(self.evx * self.evx + self.evy * self.evy + self.evz * self.evz)


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def metric(values: list[float]) -> dict[str, float | int | None]:
    absolute = [abs(value) for value in values]
    return {
        "count": len(values),
        "rmse": math.sqrt(sum(value * value for value in values) / len(values)) if values else None,
        "p95_abs": percentile(absolute, 0.95),
        "max_abs": max(absolute) if absolute else None,
        "mean": sum(values) / len(values) if values else None,
    }


def summarize(samples: list[ErrorSample]) -> dict[str, object]:
    return {
        "count": len(samples),
        "reference_speed_mps": {
            "mean": sum(sample.reference_speed_mps for sample in samples) / len(samples) if samples else None,
            "max": max((sample.reference_speed_mps for sample in samples), default=None),
        },
        "position_m": {
            "x": metric([sample.ex for sample in samples]),
            "y": metric([sample.ey for sample in samples]),
            "z": metric([sample.ez for sample in samples]),
            "norm": metric([sample.position_norm for sample in samples]),
        },
        "velocity_mps": {
            "x": metric([sample.evx for sample in samples]),
            "y": metric([sample.evy for sample in samples]),
            "z": metric([sample.evz for sample in samples]),
            "norm": metric([sample.velocity_norm for sample in samples]),
        },
        "yaw_rad": metric([sample.yaw_error_rad for sample in samples]),
    }


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def speed_bin(speed: float) -> str:
    if speed < 1.0:
        return "0-1_mps"
    if speed < 2.0:
        return "1-2_mps"
    if speed < 3.0:
        return "2-3_mps"
    if speed < 4.0:
        return "3-4_mps"
    if speed < 5.0:
        return "4-5_mps"
    return "5plus_mps"


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze B-spline tracking errors by flight phase.")
    parser.add_argument("bag", type=Path)
    parser.add_argument("--storage-id", default="sqlite3")
    parser.add_argument(
        "--expected-lap-speeds",
        default="",
        help="Comma-separated requested lap speeds used to verify every planned lap.",
    )
    args = parser.parse_args()
    try:
        expected_lap_speeds = [
            float(value) for value in args.expected_lap_speeds.split(",") if value.strip()
        ]
    except ValueError:
        print("FAIL: --expected-lap-speeds must be a comma-separated numeric list", file=sys.stderr)
        return 2
    if any(not math.isfinite(speed) or speed <= 0.0 for speed in expected_lap_speeds):
        print("FAIL: --expected-lap-speeds values must be finite and positive", file=sys.stderr)
        return 2
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
        print(f"FAIL: cannot open bag: {exc}", file=sys.stderr)
        return 2

    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in (PHASE_TOPIC, COMMAND_TOPIC, REFERENCE_TOPIC, ERROR_TOPIC) if topic not in topic_types]
    if missing:
        print("FAIL: missing topics: " + ", ".join(missing), file=sys.stderr)
        return 2
    classes = {topic: get_message(topic_types[topic]) for topic in topic_types}

    phase = "UNMARKED"
    reference_speed = 0.0
    samples: list[ErrorSample] = []
    command_stamps: list[int] = []
    phase_counts: dict[str, int] = defaultdict(int)
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic == PHASE_TOPIC:
            message = deserialize_message(serialized, classes[topic])
            phase = message.data
            phase_counts[phase] += 1
        elif topic == COMMAND_TOPIC:
            command_stamps.append(timestamp_ns)
        elif topic == REFERENCE_TOPIC:
            message = deserialize_message(serialized, classes[topic])
            velocity = message.twist.twist.linear
            reference_speed = math.sqrt(
                velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z
            )
        elif topic == ERROR_TOPIC:
            message = deserialize_message(serialized, classes[topic])
            position = message.pose.pose.position
            velocity = message.twist.twist.linear
            samples.append(
                ErrorSample(
                    timestamp_ns=timestamp_ns,
                    phase=phase,
                    reference_speed_mps=reference_speed,
                    ex=position.x,
                    ey=position.y,
                    ez=position.z,
                    evx=velocity.x,
                    evy=velocity.y,
                    evz=velocity.z,
                    yaw_error_rad=yaw_from_quaternion(message.pose.pose.orientation),
                )
            )

    observed_lap_phases = sorted(
        {sample.phase for sample in samples if sample.phase.startswith("LAP_")},
        key=lambda name: int(name.removeprefix("LAP_")),
    )
    expected_lap_phases = [
        f"LAP_{index + 1}" for index in range(len(expected_lap_speeds))
    ]
    lap_phases = expected_lap_phases or observed_lap_phases
    tracking_phases = ("TRANSFER_IN", *lap_phases, "TRANSFER_OUT")
    by_phase = {
        name: summarize([sample for sample in samples if sample.phase == name])
        for name in tracking_phases
    }
    by_speed = {
        name: summarize([sample for sample in samples if speed_bin(sample.reference_speed_mps) == name and sample.phase.startswith("LAP_")])
        for name in SPEED_BINS
    }
    lap_samples = [sample for sample in samples if sample.phase.startswith("LAP_")]
    command_gaps = [
        (command_stamps[index] - command_stamps[index - 1]) * 1e-9
        for index in range(1, len(command_stamps))
    ]
    command_span_s = (command_stamps[-1] - command_stamps[0]) * 1e-9 if len(command_stamps) >= 2 else 0.0
    command_rate_hz = (len(command_stamps) - 1) / command_span_s if command_span_s > 0.0 else 0.0

    errors: list[str] = []
    warnings: list[str] = []
    for phase_name in lap_phases:
        if not any(sample.phase == phase_name for sample in samples):
            errors.append(f"{phase_name} has no tracking-error samples")
    for index, expected_speed in enumerate(expected_lap_speeds):
        phase_name = f"LAP_{index + 1}"
        peak = max(
            (
                sample.reference_speed_mps
                for sample in samples
                if sample.phase == phase_name
            ),
            default=0.0,
        )
        minimum_peak = 0.95 * expected_speed
        if peak < minimum_peak:
            warnings.append(
                f"{phase_name} reference peak was {peak:.2f} m/s, "
                f"below 95% of requested {expected_speed:.2f} m/s"
            )
    if command_rate_hz < 80.0:
        errors.append(f"trajectory command rate was only {command_rate_hz:.1f} Hz")
    max_gap_s = max(command_gaps, default=0.0)
    if max_gap_s > 0.08:
        errors.append(f"maximum trajectory command gap was {1000.0 * max_gap_s:.1f} ms")
    elif max_gap_s > 0.03:
        warnings.append(f"maximum trajectory command gap was {1000.0 * max_gap_s:.1f} ms")

    report = {
        "bag": str(bag_path),
        "result": "FAIL" if errors else ("WARN" if warnings else "PASS"),
        "phase_message_counts": dict(phase_counts),
        "command": {
            "count": len(command_stamps),
            "rate_hz": command_rate_hz,
            "max_gap_ms": 1000.0 * max_gap_s,
        },
        "all_laps": summarize(lap_samples),
        "by_phase": by_phase,
        "by_speed": by_speed,
        "warnings": warnings,
        "errors": errors,
    }
    report_path = bag_path / "bspline_tracking_analysis.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    csv_path = bag_path / "bspline_tracking_samples.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            (
                "timestamp_ns",
                "phase",
                "reference_speed_mps",
                "error_x_m",
                "error_y_m",
                "error_z_m",
                "error_norm_m",
                "velocity_error_x_mps",
                "velocity_error_y_mps",
                "velocity_error_z_mps",
                "velocity_error_norm_mps",
                "yaw_error_rad",
            )
        )
        for sample in samples:
            writer.writerow(
                (
                    sample.timestamp_ns,
                    sample.phase,
                    sample.reference_speed_mps,
                    sample.ex,
                    sample.ey,
                    sample.ez,
                    sample.position_norm,
                    sample.evx,
                    sample.evy,
                    sample.evz,
                    sample.velocity_norm,
                    sample.yaw_error_rad,
                )
            )

    print(f"Bag: {bag_path}")
    print(f"Command: count={len(command_stamps)}, rate={command_rate_hz:.2f} Hz, max_gap={1000.0 * max_gap_s:.2f} ms")
    for phase_name in tracking_phases:
        summary = by_phase[phase_name]
        position = summary["position_m"]["norm"]  # type: ignore[index]
        yaw = summary["yaw_rad"]  # type: ignore[assignment]
        print(
            f"{phase_name}: n={summary['count']} "
            f"position_rmse={position['rmse']}m position_p95={position['p95_abs']}m "
            f"yaw_rmse={yaw['rmse']}rad"
        )
    for warning in warnings:
        print(f"WARN: {warning}")
    for error in errors:
        print(f"FAIL: {error}")
    print(f"Report: {report_path}")
    print(f"Samples: {csv_path}")
    print(f"RESULT: {report['result']}")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
