#!/usr/bin/env python

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class DronePose:
    x: float
    y: float
    z: float
    yaw: float
    timestamp_s: float


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class ROS2PoseBridge:
    """ROS2 bridge for Nokov PoseStamped input and MAVROS PoseStamped setpoints."""

    def __init__(self, node_name: str, pose_topic: str, setpoint_topic: str, frame_id: str):
        self.node_name = node_name
        self.pose_topic = pose_topic
        self.setpoint_topic = setpoint_topic
        self.frame_id = frame_id
        self._latest_pose: DronePose | None = None
        self._lock = threading.Lock()
        self._connected = False

        self._rclpy = None
        self._node = None
        self._executor = None
        self._publisher = None
        self._pose_msg_cls = None
        self._spin_thread: threading.Thread | None = None

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        self._rclpy = rclpy
        self._pose_msg_cls = PoseStamped

        if not rclpy.ok():
            rclpy.init(args=None)

        self._node = rclpy.create_node(self.node_name)
        self._publisher = self._node.create_publisher(PoseStamped, self.setpoint_topic, 10)
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._node.create_subscription(PoseStamped, self.pose_topic, self._pose_callback, pose_qos)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        self._connected = True

    def _pose_callback(self, msg) -> None:
        position = msg.pose.position
        orientation = msg.pose.orientation
        pose = DronePose(
            x=float(position.x),
            y=float(position.y),
            z=float(position.z),
            yaw=yaw_from_quaternion(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
            timestamp_s=time.monotonic(),
        )
        with self._lock:
            self._latest_pose = pose

    def get_latest_pose(self, max_age_s: float) -> DronePose | None:
        deadline_s = time.monotonic() + max(max_age_s, 0.0)
        while True:
            with self._lock:
                pose = self._latest_pose

            now_s = time.monotonic()
            if pose is not None and now_s - pose.timestamp_s <= max_age_s:
                return pose

            if now_s >= deadline_s:
                return None

            time.sleep(0.005)

    def publish_setpoint(self, x: float, y: float, z: float, yaw: float) -> None:
        if not self._connected or self._node is None or self._publisher is None or self._pose_msg_cls is None:
            raise RuntimeError("ROS2PoseBridge is not connected.")

        msg = self._pose_msg_cls()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._publisher.publish(msg)

    def disconnect(self) -> None:
        self._connected = False
        if self._executor is not None:
            self._executor.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
        if self._node is not None:
            self._node.destroy_node()
        # Do not call global rclpy.shutdown() here. LeRobot may run another ROS2
        # device, such as the ros_expert_pose teleoperator, in the same process.
        # Destroying this node is enough; the process exit will release rclpy.
