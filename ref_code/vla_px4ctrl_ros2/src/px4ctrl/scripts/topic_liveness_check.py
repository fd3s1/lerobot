#!/usr/bin/env python3
"""Check several ROS 2 topics concurrently with permissive QoS.

This is used by shell launch/test wrappers before flight. It avoids spawning
one `ros2 topic echo` process per topic and avoids false negatives from
Reliable-vs-BestEffort QoS mismatches on MAVROS/VRPN sensor topics.
"""

from __future__ import annotations

import argparse
import importlib
import sys
import time
from dataclasses import dataclass

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass(frozen=True)
class TopicSpec:
    label: str
    topic: str
    type_name: str


def import_msg_type(type_name: str):
    parts = type_name.split("/")
    if len(parts) != 3 or parts[1] != "msg":
        raise ValueError(f"unsupported message type format: {type_name!r}")
    module = importlib.import_module(f"{parts[0]}.msg")
    return getattr(module, parts[2])


def parse_topic_spec(raw: str) -> TopicSpec:
    parts = raw.split(":", 2)
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            "--topic expects 'label:/topic:pkg/msg/Type', got " + repr(raw)
        )
    label, topic, type_name = (item.strip() for item in parts)
    if not label or not topic or not type_name:
        raise argparse.ArgumentTypeError("topic spec fields must not be empty: " + repr(raw))
    return TopicSpec(label=label, topic=topic, type_name=type_name)


def make_qos(reliability: ReliabilityPolicy) -> QoSProfile:
    return QoSProfile(
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=6.0)
    parser.add_argument("--log-prefix", default="[topic-liveness]")
    parser.add_argument(
        "--status-log-period",
        type=float,
        default=0.0,
        help="Periodic waiting log interval in seconds. 0 disables waiting countdown logs.",
    )
    parser.add_argument("--topic", action="append", type=parse_topic_spec, required=True)
    args = parser.parse_args()

    timeout_s = max(0.1, float(args.timeout))
    status_log_period_s = max(0.0, float(args.status_log_period))
    specs: list[TopicSpec] = args.topic
    seen: dict[str, bool] = {spec.topic: False for spec in specs}
    topic_to_label = {spec.topic: spec.label for spec in specs}

    rclpy.init(args=None)
    node = rclpy.create_node("topic_liveness_check")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    subscriptions = []

    def make_callback(topic: str):
        def callback(_msg) -> None:
            seen[topic] = True

        return callback

    try:
        for spec in specs:
            msg_type = import_msg_type(spec.type_name)
            # A BestEffort subscription is compatible with BestEffort and
            # Reliable publishers, and avoids noisy incompatible-QoS warnings
            # from sensor topics that only offer BestEffort.
            subscriptions.append(
                node.create_subscription(
                    msg_type,
                    spec.topic,
                    make_callback(spec.topic),
                    make_qos(ReliabilityPolicy.BEST_EFFORT),
                )
            )

        deadline = time.monotonic() + timeout_s
        next_log_s = time.monotonic() + status_log_period_s if status_log_period_s > 0.0 else float("inf")
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if all(seen.values()):
                for spec in specs:
                    print(f"{args.log_prefix} {spec.label} is live: {spec.topic}")
                return 0

            now = time.monotonic()
            if now >= next_log_s:
                missing = [topic_to_label[t] for t, ok in seen.items() if not ok]
                print(
                    f"{args.log_prefix} waiting for topics: {', '.join(missing)} "
                    f"remaining={max(0.0, deadline - now):.1f}s"
                )
                next_log_s = now + status_log_period_s

        missing_specs = [spec for spec in specs if not seen[spec.topic]]
        for spec in missing_specs:
            print(
                f"{args.log_prefix} WARNING: no fresh {spec.label} on {spec.topic}",
                file=sys.stderr,
            )
        return 1
    finally:
        # Keep subscriptions referenced until after spinning.
        subscriptions.clear()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
