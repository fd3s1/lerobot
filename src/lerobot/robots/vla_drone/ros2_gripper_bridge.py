#!/usr/bin/env python

from __future__ import annotations

import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class GripperFeedbackSample:
    left_pos: float
    right_pos: float
    left_load: float
    right_load: float
    left_current: float
    right_current: float
    left_position_error: float
    right_position_error: float
    left_goal_pos: float
    right_goal_pos: float
    timestamp_s: float


class ROS2GripperBridge:
    """ROS2 bridge for an external gripper manager.

    This lets LeRobot record observations/actions without opening the Feetech
    serial port. The manager owns /dev/ttyACM1 and publishes feedback.
    """

    def __init__(self, node_name: str, command_topic: str, feedback_topic: str):
        self.node_name = node_name
        self.command_topic = command_topic
        self.feedback_topic = feedback_topic
        self._latest_feedback: GripperFeedbackSample | None = None
        self._lock = threading.Lock()
        self._connected = False

        self._rclpy = None
        self._node = None
        self._executor = None
        self._publisher = None
        self._command_msg_cls = None
        self._spin_thread: threading.Thread | None = None

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        import rclpy
        from quadrotor_msgs.msg import GripperCommandPair, GripperFeedback
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        self._rclpy = rclpy
        self._command_msg_cls = GripperCommandPair

        if not rclpy.ok():
            rclpy.init(args=None)

        self._node = rclpy.create_node(self.node_name)
        self._publisher = self._node.create_publisher(GripperCommandPair, self.command_topic, 10)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._node.create_subscription(GripperFeedback, self.feedback_topic, self._feedback_cb, reliable_qos)
        self._node.create_subscription(GripperFeedback, self.feedback_topic, self._feedback_cb, best_effort_qos)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        self._connected = True

    def _feedback_cb(self, msg) -> None:
        feedback = GripperFeedbackSample(
            left_pos=float(msg.left_pos),
            right_pos=float(msg.right_pos),
            left_load=float(msg.left_load),
            right_load=float(msg.right_load),
            left_current=float(msg.left_current),
            right_current=float(msg.right_current),
            left_position_error=float(msg.left_position_error),
            right_position_error=float(msg.right_position_error),
            left_goal_pos=float(msg.left_goal_pos),
            right_goal_pos=float(msg.right_goal_pos),
            timestamp_s=time.monotonic(),
        )
        with self._lock:
            self._latest_feedback = feedback

    def get_latest_feedback(self, max_age_s: float) -> GripperFeedbackSample | None:
        deadline_s = time.monotonic() + max(max_age_s, 0.0)
        while True:
            with self._lock:
                feedback = self._latest_feedback

            now_s = time.monotonic()
            if feedback is not None and now_s - feedback.timestamp_s <= max_age_s:
                return feedback

            if now_s >= deadline_s:
                return None

            time.sleep(0.005)

    def publish_pair(self, left: float, right: float) -> None:
        if not self._connected or self._node is None or self._publisher is None or self._command_msg_cls is None:
            raise RuntimeError("ROS2GripperBridge is not connected.")

        msg = self._command_msg_cls()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "gripper"
        msg.left_pos = float(left)
        msg.right_pos = float(right)
        self._publisher.publish(msg)

    def disconnect(self) -> None:
        self._connected = False
        if self._executor is not None:
            self._executor.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
        if self._node is not None:
            self._node.destroy_node()
        # Keep the global rclpy context alive; the pose bridge and teleop may share it.
