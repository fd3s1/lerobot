#!/usr/bin/env python

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any

from lerobot.processor import RobotAction
from lerobot.teleoperators.teleoperator import Teleoperator

from .config_ros_expert_pose import ROSExpertPoseTeleopConfig


ACTION_X = "x"
ACTION_Y = "y"
ACTION_Z = "z"
ACTION_YAW = "yaw"
GRIPPER_LEFT_POS = "gripper_left.pos"
GRIPPER_RIGHT_POS = "gripper_right.pos"


@dataclass(frozen=True)
class TimedExpertPose:
    x: float
    y: float
    z: float
    yaw: float
    received_at_s: float
    header_stamp_s: float


@dataclass(frozen=True)
class TimedGripperCommand:
    value: float
    received_at_s: float


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


class ROSExpertPoseTeleop(Teleoperator):
    """Read expert drone actions from px4ctrl ROS2 topics.

    The pose action is sampled from /px4ctrl/expert_pose, which px4ctrl stamps
    with the same timestamp used for its MAVROS setpoint publication. The
    gripper action is the last commanded /gripper/command value, which is a held
    target rather than a continuously published stream.
    """

    config_class = ROSExpertPoseTeleopConfig
    name = "ros_expert_pose"

    def __init__(self, config: ROSExpertPoseTeleopConfig):
        super().__init__(config)
        self.config = config
        self._latest_pose: TimedExpertPose | None = None
        self._latest_gripper: TimedGripperCommand | None = None
        self._lock = threading.Lock()
        self._connected = False

        self._rclpy = None
        self._node = None
        self._executor = None
        self._spin_thread: threading.Thread | None = None

    @property
    def action_features(self) -> dict[str, type]:
        return {
            ACTION_X: float,
            ACTION_Y: float,
            ACTION_Z: float,
            ACTION_YAW: float,
            GRIPPER_LEFT_POS: float,
            GRIPPER_RIGHT_POS: float,
        }

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float64

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=None)

        self._node = rclpy.create_node(self.config.ros_node_name)
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
        self._node.create_subscription(PoseStamped, self.config.expert_pose_topic, self._pose_cb, reliable_qos)
        self._node.create_subscription(PoseStamped, self.config.expert_pose_topic, self._pose_cb, best_effort_qos)
        self._node.create_subscription(Float64, self.config.gripper_topic, self._gripper_cb, reliable_qos)
        self._node.create_subscription(Float64, self.config.gripper_topic, self._gripper_cb, best_effort_qos)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        self._connected = True

    def configure(self) -> None:
        return

    def calibrate(self) -> None:
        return

    def _pose_cb(self, msg) -> None:
        position = msg.pose.position
        orientation = msg.pose.orientation
        pose = TimedExpertPose(
            x=float(position.x),
            y=float(position.y),
            z=float(position.z),
            yaw=yaw_from_quaternion(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
            received_at_s=time.monotonic(),
            header_stamp_s=stamp_to_seconds(msg.header.stamp),
        )
        with self._lock:
            self._latest_pose = pose

    def _gripper_cb(self, msg) -> None:
        gripper = TimedGripperCommand(
            value=clamp(float(msg.data), 0.0, 100.0),
            received_at_s=time.monotonic(),
        )
        with self._lock:
            self._latest_gripper = gripper

    def get_action(self) -> RobotAction:
        if not self._connected:
            raise RuntimeError("ROS expert pose teleoperator is not connected.")

        deadline_s = time.monotonic() + max(self.config.startup_timeout_s, 0.0)
        while True:
            now_s = time.monotonic()
            with self._lock:
                pose = self._latest_pose
                gripper = self._latest_gripper

            if pose is not None or now_s >= deadline_s:
                break

            time.sleep(0.005)

        if pose is None:
            raise RuntimeError(
                f"No expert pose received on {self.config.expert_pose_topic} "
                f"within {self.config.startup_timeout_s:.3f}s."
            )

        pose_age_s = now_s - pose.received_at_s
        if pose_age_s > self.config.max_pose_age_s:
            raise RuntimeError(
                f"Expert pose on {self.config.expert_pose_topic} is stale: "
                f"{pose_age_s:.3f}s > {self.config.max_pose_age_s:.3f}s."
            )

        if gripper is None:
            if self.config.require_gripper_command:
                raise RuntimeError(f"No gripper command received on {self.config.gripper_topic}.")
            gripper_value = clamp(self.config.default_gripper_pos, 0.0, 100.0)
        else:
            gripper_age_s = now_s - gripper.received_at_s
            if self.config.max_gripper_age_s > 0.0 and gripper_age_s > self.config.max_gripper_age_s:
                raise RuntimeError(
                    f"Gripper command on {self.config.gripper_topic} is stale: "
                    f"{gripper_age_s:.3f}s > {self.config.max_gripper_age_s:.3f}s."
                )
            gripper_value = gripper.value

        return {
            ACTION_X: pose.x,
            ACTION_Y: pose.y,
            ACTION_Z: pose.z,
            ACTION_YAW: pose.yaw,
            GRIPPER_LEFT_POS: gripper_value,
            GRIPPER_RIGHT_POS: gripper_value,
        }

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return

    def disconnect(self) -> None:
        self._connected = False
        if self._executor is not None:
            self._executor.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
        if self._node is not None:
            self._node.destroy_node()
        # Keep the global rclpy context alive because the robot ROS2 bridge may
        # share it in the same LeRobot process.


__all__ = ["ROSExpertPoseTeleop", "ROSExpertPoseTeleopConfig"]
