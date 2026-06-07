#!/usr/bin/python3

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass

import rclpy
from mavros_msgs.msg import RCIn
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool, Float64, String


FLOAT_STATUS_FIELDS = (
    "left_close_ratio",
    "right_close_ratio",
    "left_raw_pos",
    "right_raw_pos",
    "left_close_segment_ratio",
    "right_close_segment_ratio",
    "center_error_ratio",
    "center_error_m",
    "left_current",
    "right_current",
    "left_current_residual",
    "right_current_residual",
    "single_contact_direction",
    "roll_deg",
    "pitch_deg",
)

BOOL_STATUS_FIELDS = (
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


@dataclass(frozen=True)
class HandheldHlsConfig:
    rc_topic: str
    status_topic: str
    command_topic: str
    rate_hz: float
    rc_timeout_s: float
    status_timeout_s: float
    ch10_index: int
    ch10_open_pwm: int
    ch10_close_pwm: int
    open_command: float
    close_command: float
    publish_period_s: float
    grasp_mode_stable_s: float
    open_mode_stable_s: float
    hold_mode_timeout_s: float
    rc_stale_mode: str
    csv_path: str


@dataclass
class StandardHlsStatus:
    state: str = "UNKNOWN"
    fault_reason: str = ""
    left_close_ratio: float = 0.0
    right_close_ratio: float = 0.0
    left_raw_pos: float = 0.0
    right_raw_pos: float = 0.0
    left_close_segment_ratio: float = 0.0
    right_close_segment_ratio: float = 0.0
    center_error_ratio: float = 0.0
    center_error_m: float = 0.0
    left_current: float = 0.0
    right_current: float = 0.0
    left_current_residual: float = 0.0
    right_current_residual: float = 0.0
    single_contact_direction: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    left_contact: bool = False
    right_contact: bool = False
    both_contact: bool = False
    centered: bool = False
    safe_to_lift: bool = False
    fault: bool = False
    single_contact_need_motion: bool = False
    left_at_close_limit: bool = False
    right_at_close_limit: bool = False


def status_prefix_from_topic(topic: str) -> str:
    topic = topic.rstrip("/")
    if topic.endswith("/status"):
        return topic[: -len("/status")]
    return topic


class HandheldHlsGraspTest(Node):
    def __init__(self, config: HandheldHlsConfig) -> None:
        super().__init__("handheld_hls_grasp_test")
        self.config = config
        self.status_prefix = status_prefix_from_topic(config.status_topic)
        self.rc: RCIn | None = None
        self.rc_received_s: float | None = None
        self.status = StandardHlsStatus()
        self.status_stamps: dict[str, float] = {}
        self.last_command_mode = "unknown"
        self.last_publish_s = 0.0
        self.last_log_s = 0.0
        self.latched_command_mode = "open"
        self.raw_command_mode = "open"
        self.raw_mode_started_s = time.monotonic()
        self.candidate_command_mode = "open"
        self.candidate_mode_started_s = self.raw_mode_started_s
        self.last_non_hold_raw_s = self.raw_mode_started_s
        self.csv_file = None
        self.csv_writer = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(RCIn, config.rc_topic, self._rc_cb, qos)
        self.create_subscription(String, f"{self.status_prefix}/state", self._string_cb("state"), 10)
        self.create_subscription(String, f"{self.status_prefix}/fault_reason", self._string_cb("fault_reason"), 10)
        for name in FLOAT_STATUS_FIELDS:
            self.create_subscription(Float64, f"{self.status_prefix}/{name}", self._float_cb(name), 10)
        for name in BOOL_STATUS_FIELDS:
            self.create_subscription(Bool, f"{self.status_prefix}/{name}", self._bool_cb(name), 10)
        self.command_pub = self.create_publisher(Float64, config.command_topic, 10)

        if config.csv_path:
            self.csv_file = open(config.csv_path, "w", newline="")
            self.csv_writer = csv.DictWriter(
                self.csv_file,
                fieldnames=[
                    "monotonic_s",
                    "command_mode",
                    "raw_command_mode",
                    "state",
                    "left_close_ratio",
                    "right_close_ratio",
                    "left_raw_pos",
                    "right_raw_pos",
                    "left_close_segment_ratio",
                    "right_close_segment_ratio",
                    "center_error_ratio",
                    "center_error_m",
                    "left_contact",
                    "right_contact",
                    "both_contact",
                    "centered",
                    "safe_to_lift",
                    "fault",
                    "fault_reason",
                    "left_current",
                    "right_current",
                    "left_current_residual",
                    "right_current_residual",
                    "single_contact_need_motion",
                    "single_contact_direction",
                    "left_at_close_limit",
                    "right_at_close_limit",
                    "roll_deg",
                    "pitch_deg",
                ],
            )
            self.csv_writer.writeheader()

        self.get_logger().info(
            "Handheld HLS grasp test started with standard HLS status topics. "
            f"status_prefix={self.status_prefix}. This node never publishes /position_cmd."
        )

    def _rc_cb(self, msg: RCIn) -> None:
        self.rc = msg
        self.rc_received_s = time.monotonic()

    def _string_cb(self, name: str):
        def callback(msg: String) -> None:
            setattr(self.status, name, str(msg.data))
            self.status_stamps[name] = time.monotonic()

        return callback

    def _float_cb(self, name: str):
        def callback(msg: Float64) -> None:
            setattr(self.status, name, float(msg.data))
            self.status_stamps[name] = time.monotonic()

        return callback

    def _bool_cb(self, name: str):
        def callback(msg: Bool) -> None:
            setattr(self.status, name, bool(msg.data))
            self.status_stamps[name] = time.monotonic()

        return callback

    def rc_fresh(self) -> bool:
        return self.rc is not None and self.rc_received_s is not None and time.monotonic() - self.rc_received_s <= self.config.rc_timeout_s

    def status_fresh(self) -> bool:
        now = time.monotonic()
        for name in ("state", "center_error_m", "safe_to_lift", "fault"):
            stamp = self.status_stamps.get(name)
            if stamp is None or now - stamp > self.config.status_timeout_s:
                return False
        return True

    def raw_command_mode_from_rc(self) -> str:
        if not self.rc_fresh() or self.rc is None:
            return "open" if self.config.rc_stale_mode == "open" else "hold"
        if self.config.ch10_index < 0 or self.config.ch10_index >= len(self.rc.channels):
            return "open" if self.config.rc_stale_mode == "open" else "hold"
        pwm = int(self.rc.channels[self.config.ch10_index])
        if pwm >= self.config.ch10_close_pwm:
            return "grasp"
        if pwm <= self.config.ch10_open_pwm:
            return "open"
        return "hold"

    def command_mode_from_rc(self) -> str:
        now = time.monotonic()
        raw_mode = self.raw_command_mode_from_rc()
        if raw_mode != self.raw_command_mode:
            self.raw_command_mode = raw_mode
            self.raw_mode_started_s = now

        if raw_mode == "hold":
            if now - self.last_non_hold_raw_s >= self.config.hold_mode_timeout_s:
                self.latched_command_mode = "open"
            return self.latched_command_mode

        self.last_non_hold_raw_s = now
        if raw_mode != self.candidate_command_mode:
            self.candidate_command_mode = raw_mode
            self.candidate_mode_started_s = now

        stable_time = now - self.candidate_mode_started_s
        required_stable_s = (
            self.config.open_mode_stable_s if raw_mode == "open" else self.config.grasp_mode_stable_s
        )
        if stable_time >= required_stable_s:
            self.latched_command_mode = raw_mode
        return self.latched_command_mode

    def publish_command(self, mode: str, force: bool = False) -> None:
        now = time.monotonic()
        if not force and mode == self.last_command_mode and now - self.last_publish_s < self.config.publish_period_s:
            return
        if mode == "hold":
            return
        msg = Float64()
        msg.data = self.config.open_command if mode == "open" else self.config.close_command
        self.command_pub.publish(msg)
        self.last_command_mode = mode
        self.last_publish_s = now

    def log_status(self, mode: str) -> None:
        now = time.monotonic()
        if now - self.last_log_s < 0.5:
            return
        self.last_log_s = now

        if not self.status_fresh():
            self.get_logger().info(f"CH10 mode={mode} raw={self.raw_command_mode} hls_status=missing/stale")
            return

        msg = self.status
        self.get_logger().info(
            f"CH10 mode={mode} raw={self.raw_command_mode} state={msg.state} "
            f"center={msg.center_error_m:+.4f}m ratio={msg.center_error_ratio:+.3f} "
            f"pos=({msg.left_raw_pos:.0f},{msg.right_raw_pos:.0f}) "
            f"seg=({msg.left_close_segment_ratio:.2f},{msg.right_close_segment_ratio:.2f}) "
            f"contact=({int(msg.left_contact)},{int(msg.right_contact)}) "
            f"limit=({int(msg.left_at_close_limit)},{int(msg.right_at_close_limit)}) "
            f"move={int(msg.single_contact_need_motion)} dir={msg.single_contact_direction:+.0f} "
            f"safe={int(msg.safe_to_lift)} fault={int(msg.fault)} "
            f"cur=({msg.left_current:.0f},{msg.right_current:.0f}) "
            f"res=({msg.left_current_residual:.0f},{msg.right_current_residual:.0f})"
        )

    def write_csv(self, mode: str) -> None:
        if self.csv_writer is None:
            return
        msg = self.status
        self.csv_writer.writerow(
            {
                "monotonic_s": f"{time.monotonic():.6f}",
                "command_mode": mode,
                "raw_command_mode": self.raw_command_mode,
                "state": msg.state,
                "left_close_ratio": f"{msg.left_close_ratio:.6f}",
                "right_close_ratio": f"{msg.right_close_ratio:.6f}",
                "left_raw_pos": f"{msg.left_raw_pos:.3f}",
                "right_raw_pos": f"{msg.right_raw_pos:.3f}",
                "left_close_segment_ratio": f"{msg.left_close_segment_ratio:.6f}",
                "right_close_segment_ratio": f"{msg.right_close_segment_ratio:.6f}",
                "center_error_ratio": f"{msg.center_error_ratio:.6f}",
                "center_error_m": f"{msg.center_error_m:.6f}",
                "left_contact": int(msg.left_contact),
                "right_contact": int(msg.right_contact),
                "both_contact": int(msg.both_contact),
                "centered": int(msg.centered),
                "safe_to_lift": int(msg.safe_to_lift),
                "fault": int(msg.fault),
                "fault_reason": msg.fault_reason,
                "left_current": f"{msg.left_current:.3f}",
                "right_current": f"{msg.right_current:.3f}",
                "left_current_residual": f"{msg.left_current_residual:.3f}",
                "right_current_residual": f"{msg.right_current_residual:.3f}",
                "single_contact_need_motion": int(msg.single_contact_need_motion),
                "single_contact_direction": f"{msg.single_contact_direction:.3f}",
                "left_at_close_limit": int(msg.left_at_close_limit),
                "right_at_close_limit": int(msg.right_at_close_limit),
                "roll_deg": f"{msg.roll_deg:.3f}",
                "pitch_deg": f"{msg.pitch_deg:.3f}",
            }
        )
        self.csv_file.flush()

    def run(self) -> None:
        period = 1.0 / self.config.rate_hz
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            mode = self.command_mode_from_rc()
            self.publish_command(mode)
            self.log_status(mode)
            if self.status_fresh():
                self.write_csv(mode)
            time.sleep(period)

    def close(self) -> None:
        if self.csv_file is not None:
            self.csv_file.close()


def parse_args() -> HandheldHlsConfig:
    parser = argparse.ArgumentParser(description="Handheld HLS force-grasp reaction test.")
    parser.add_argument("--rc-topic", default="/mavros/rc/in")
    parser.add_argument("--status-topic", default="/hls_gripper/status")
    parser.add_argument("--command-topic", default="/gripper/command")
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--rc-timeout-s", type=float, default=0.8)
    parser.add_argument("--status-timeout-s", type=float, default=2.0)
    parser.add_argument("--ch10-index", type=int, default=9)
    parser.add_argument("--ch10-open-pwm", type=int, default=1300)
    parser.add_argument("--ch10-close-pwm", type=int, default=1700)
    parser.add_argument("--open-command", type=float, default=100.0)
    parser.add_argument("--close-command", type=float, default=0.0)
    parser.add_argument("--publish-period-s", type=float, default=0.5)
    parser.add_argument("--grasp-mode-stable-s", type=float, default=0.3)
    parser.add_argument("--open-mode-stable-s", type=float, default=0.8)
    parser.add_argument("--hold-mode-timeout-s", type=float, default=1.5)
    parser.add_argument("--rc-stale-mode", choices=("hold", "open"), default="hold")
    parser.add_argument("--csv-path", default="")
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    if args.rate_hz <= 0.0:
        raise ValueError("--rate-hz must be positive.")
    if args.ch10_index < 0:
        raise ValueError("--ch10-index must be non-negative.")
    if args.grasp_mode_stable_s < 0.0:
        raise ValueError("--grasp-mode-stable-s must be non-negative.")
    if args.open_mode_stable_s < 0.0:
        raise ValueError("--open-mode-stable-s must be non-negative.")
    if args.hold_mode_timeout_s < 0.0:
        raise ValueError("--hold-mode-timeout-s must be non-negative.")
    return HandheldHlsConfig(**vars(args))


def main() -> None:
    config = parse_args()
    rclpy.init()
    node = HandheldHlsGraspTest(config)
    try:
        node.run()
    except KeyboardInterrupt:
        try:
            if rclpy.ok():
                node.publish_command("open", force=True)
        except Exception as exc:
            node.get_logger().warn(f"failed to publish open command during shutdown: {exc}")
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
