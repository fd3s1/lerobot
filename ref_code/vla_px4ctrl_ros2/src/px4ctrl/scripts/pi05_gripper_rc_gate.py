#!/usr/bin/env python3

from __future__ import annotations

import argparse
import time

import rclpy
from mavros_msgs.msg import RCIn
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64


def rc_qos(reliability: ReliabilityPolicy) -> QoSProfile:
    return QoSProfile(
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class Pi05GripperRcGate(Node):
    """CH10 safety gate for pi0.5 inference.

    CH10 low forces the gripper open. CH10 high/middle publishes nothing, so
    pi0.5/HLS remains the only source of close/grasp commands during inference.
    """

    def __init__(self, args: argparse.Namespace):
        super().__init__("pi05_gripper_rc_gate")
        self.args = args
        self.command_pub = self.create_publisher(Float64, args.command_topic, 10)
        self.create_subscription(RCIn, args.rc_topic, self.rc_cb, rc_qos(ReliabilityPolicy.RELIABLE))
        self.create_subscription(RCIn, args.rc_topic, self.rc_cb, rc_qos(ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(1.0 / max(1e-3, args.rate_hz), self.timer_cb)

        self.rc_msg: RCIn | None = None
        self.rc_received_s = 0.0
        self.last_publish_s = 0.0
        self.last_log_s = 0.0
        self.last_pwm: int | None = None
        self.last_mode = "unknown"

        self.get_logger().info(
            f"pi0.5 gripper RC gate started: CH{args.ch10_index + 1} "
            f"low<={args.open_pwm} forces open {args.open_command:.1f} on {args.command_topic}; "
            "high/middle allows policy/HLS control."
        )

    def rc_cb(self, msg: RCIn) -> None:
        self.rc_msg = msg
        self.rc_received_s = time.monotonic()

    def rc_fresh(self) -> bool:
        return self.rc_msg is not None and time.monotonic() - self.rc_received_s <= self.args.rc_timeout_s

    def current_mode(self) -> str:
        if not self.rc_fresh() or self.rc_msg is None:
            self.last_pwm = None
            return self.args.rc_stale_action
        if self.args.ch10_index < 0 or self.args.ch10_index >= len(self.rc_msg.channels):
            self.last_pwm = None
            return self.args.rc_stale_action
        pwm = int(self.rc_msg.channels[self.args.ch10_index])
        self.last_pwm = pwm
        if pwm <= self.args.open_pwm:
            return "force_open"
        return "allow_control"

    def publish_open(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_publish_s < self.args.publish_period_s:
            return
        msg = Float64()
        msg.data = float(self.args.open_command)
        self.command_pub.publish(msg)
        self.last_publish_s = now

    def timer_cb(self) -> None:
        mode = self.current_mode()
        if mode == "force_open":
            self.publish_open()
        elif mode == "open":
            self.publish_open()

        now = time.monotonic()
        if mode != self.last_mode or now - self.last_log_s >= self.args.log_period_s:
            self.last_mode = mode
            self.last_log_s = now
            self.get_logger().info(f"CH10 gate mode={mode} pwm={self.last_pwm}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rc-topic", default="/mavros/rc/in")
    parser.add_argument("--command-topic", default="/gripper/command")
    parser.add_argument("--ch10-index", type=int, default=9)
    parser.add_argument("--open-pwm", type=int, default=1300)
    parser.add_argument("--open-command", type=float, default=100.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--publish-period-s", type=float, default=0.10)
    parser.add_argument("--rc-timeout-s", type=float, default=0.5)
    parser.add_argument("--rc-stale-action", choices=("hold", "open"), default="hold")
    parser.add_argument("--log-period-s", type=float, default=1.0)
    return parser.parse_args(remove_ros_args()[1:])


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = Pi05GripperRcGate(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
