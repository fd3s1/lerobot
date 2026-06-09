#!/usr/bin/env python3

from __future__ import annotations

import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


HELP = """VRPN to MAVROS vision pose bridge.

Subscribes to a VRPN PoseStamped topic with BEST_EFFORT QoS and republishes it
to MAVROS vision_pose with RELIABLE QoS.

Examples:
  ros2 run px4ctrl vrpn_to_mavros_vision_bridge.py
  ros2 run px4ctrl vrpn_to_mavros_vision_bridge.py --ros-args -p source_topic:=/vla_drone1/pose
  ros2 run px4ctrl vrpn_to_mavros_vision_bridge.py --ros-args -p frame_id_override:=map
  ros2 run px4ctrl vrpn_to_mavros_vision_bridge.py --ros-args -p restamp:=true

Parameters:
  source_topic       default /vla_drone1/pose
  target_topic       default /mavros/vision_pose/pose
  frame_id_override  default ""  (preserve input frame_id)
  restamp            default false  (preserve input header.stamp)
  stamp_age_warn_s   default 0.2
  status_period_s    default 10.0
"""


def make_qos(reliability: ReliabilityPolicy) -> QoSProfile:
    return QoSProfile(
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class VrpnToMavrosVisionBridge(Node):
    def __init__(self) -> None:
        super().__init__("vrpn_to_mavros_vision_bridge")

        self.declare_parameter("source_topic", "/vla_drone1/pose")
        self.declare_parameter("target_topic", "/mavros/vision_pose/pose")
        self.declare_parameter("frame_id_override", "")
        self.declare_parameter("restamp", False)
        self.declare_parameter("stamp_age_warn_s", 0.2)
        self.declare_parameter("status_period_s", 10.0)

        self.source_topic = self.get_parameter("source_topic").value
        self.target_topic = self.get_parameter("target_topic").value
        self.frame_id_override = self.get_parameter("frame_id_override").value
        self.restamp = bool(self.get_parameter("restamp").value)
        self.stamp_age_warn_s = float(self.get_parameter("stamp_age_warn_s").value)
        self.status_period_s = float(self.get_parameter("status_period_s").value)

        self.publisher = self.create_publisher(
            PoseStamped,
            self.target_topic,
            make_qos(ReliabilityPolicy.RELIABLE),
        )
        self.subscription = self.create_subscription(
            PoseStamped,
            self.source_topic,
            self._pose_callback,
            make_qos(ReliabilityPolicy.BEST_EFFORT),
        )
        self.timer = self.create_timer(self.status_period_s, self._status_callback)

        self.message_count = 0
        self.last_message_time_s: float | None = None
        self.last_header_stamp_s: float | None = None
        self.start_time_s = time.monotonic()

        self.get_logger().info(
            f"Bridging {self.source_topic} BEST_EFFORT -> {self.target_topic} RELIABLE"
        )
        if self.frame_id_override:
            self.get_logger().info(f"Overriding frame_id with '{self.frame_id_override}'")
        if self.restamp:
            self.get_logger().info("Restamping outgoing poses with this node clock")

    def _pose_callback(self, msg: PoseStamped) -> None:
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose

        self.last_header_stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if self.restamp:
            out.header.stamp = self.get_clock().now().to_msg()
        if self.frame_id_override:
            out.header.frame_id = self.frame_id_override

        self.publisher.publish(out)
        self.message_count += 1
        self.last_message_time_s = time.monotonic()

    def _status_callback(self) -> None:
        now_s = time.monotonic()
        elapsed_s = max(now_s - self.start_time_s, 1e-6)
        average_hz = self.message_count / elapsed_s

        if self.last_message_time_s is None:
            self.get_logger().warn(f"No messages received yet from {self.source_topic}")
            return

        age_s = now_s - self.last_message_time_s
        stamp_age_s: float | None = None
        if self.last_header_stamp_s is not None and self.last_header_stamp_s > 0.0:
            stamp_now = self.get_clock().now().nanoseconds * 1e-9
            stamp_age_s = stamp_now - self.last_header_stamp_s

        self.get_logger().info(
            f"Forwarded {self.message_count} poses, average {average_hz:.1f} Hz, "
            f"last age {age_s:.3f} s"
        )
        if stamp_age_s is None:
            self.get_logger().warn(
                "Input header.stamp is zero; use restamp:=true only as a temporary test."
            )
        elif stamp_age_s > self.stamp_age_warn_s:
            self.get_logger().warn(
                f"Input header.stamp age is {stamp_age_s:.3f} s "
                f"> stamp_age_warn_s {self.stamp_age_warn_s:.3f} s"
            )


def main() -> None:
    if "--help" in sys.argv or "-h" in sys.argv:
        print(HELP)
        return

    rclpy.init()
    node = VrpnToMavrosVisionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
