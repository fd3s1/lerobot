#!/usr/bin/python3

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Literal

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import RCIn
from quadrotor_msgs.msg import GripperCommandPair, GripperFeedback
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args

from gripper_soft_grasp_lib import GripperFeedbackState, SoftGraspConfig, SoftGraspController, clamp


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return normalize_angle(math.atan2(siny_cosp, cosy_cosp))


@dataclass(frozen=True)
class PoseSample:
    x: float
    y: float
    z: float
    yaw: float
    received_s: float
    frame_id: str


@dataclass(frozen=True)
class HandheldConfig:
    drone_pose_topic: str
    target_pose_topic: str
    box_pose_topic: str
    rc_topic: str
    gripper_command_pair_topic: str
    gripper_feedback_topic: str
    rate_hz: float
    pose_timeout_s: float
    feedback_timeout_s: float
    ch10_index: int
    ch10_threshold: int
    gripper_open: float
    gripper_z_offset_m: float
    target_height_m: float
    target_grasp_height_m: float
    target_pose_z_reference: Literal["base", "center", "top", "grasp"]
    target_hover_clearance_m: float
    box_height_m: float
    box_hover_gripper_clearance_m: float
    box_place_bottom_clearance_m: float
    target_offset_x: float
    target_offset_y: float
    target_offset_z: float
    box_offset_x: float
    box_offset_y: float
    box_offset_z: float
    grasp_step_size: float
    grasp_step_settle_s: float
    grasp_close_min: float
    grasp_contact_current_delta: float
    grasp_contact_load_delta: float
    grasp_position_error_threshold: float
    grasp_angle_contact_delta: float
    grasp_stall_delta: float
    grasp_contact_confirm_steps: int
    grasp_balance_load_diff: float
    grasp_balance_step: float
    grasp_max_balance_steps: int
    grasp_angle_balance_diff: float


class HandheldGraspTune(Node):
    def __init__(self, config: HandheldConfig) -> None:
        super().__init__("handheld_grasp_tune")
        self.config = config
        self.poses: dict[str, PoseSample] = {}
        self.rc: RCIn | None = None
        self.rc_received_s: float | None = None
        self.feedback: GripperFeedbackState | None = None
        self.last_left = config.gripper_open
        self.last_right = config.gripper_open
        self.state = "IDLE"
        self.last_open_publish_s = 0.0
        self.last_status_s = 0.0

        self._create_pose_subscription("drone", config.drone_pose_topic)
        self._create_pose_subscription("target", config.target_pose_topic)
        self._create_pose_subscription("box", config.box_pose_topic)

        rc_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(RCIn, config.rc_topic, self._rc_cb, rc_qos)
        self.create_subscription(GripperFeedback, config.gripper_feedback_topic, self._feedback_cb, 10)
        self.gripper_pair_pub = self.create_publisher(GripperCommandPair, config.gripper_command_pair_topic, 10)

        self.get_logger().info(
            "Handheld tune started. CH10 high runs soft grasp; CH10 low opens and resets. "
            "This node does not publish position commands, takeoff/land, or record data."
        )

    def _create_pose_subscription(self, key: str, topic: str) -> None:
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(PoseStamped, topic, self._pose_cb(key), qos)

    def _pose_cb(self, key: str):
        def callback(msg: PoseStamped) -> None:
            self.poses[key] = PoseSample(
                x=float(msg.pose.position.x),
                y=float(msg.pose.position.y),
                z=float(msg.pose.position.z),
                yaw=quaternion_to_yaw(msg),
                received_s=time.monotonic(),
                frame_id=str(msg.header.frame_id),
            )

        return callback

    def _rc_cb(self, msg: RCIn) -> None:
        self.rc = msg
        self.rc_received_s = time.monotonic()

    def _feedback_cb(self, msg: GripperFeedback) -> None:
        self.feedback = GripperFeedbackState(
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
            received_s=time.monotonic(),
        )

    def pose_fresh(self, key: str) -> bool:
        pose = self.poses.get(key)
        return pose is not None and time.monotonic() - pose.received_s <= self.config.pose_timeout_s

    def feedback_fresh(self) -> bool:
        return self.feedback is not None and time.monotonic() - self.feedback.received_s <= self.config.feedback_timeout_s

    def ch10_high(self) -> bool:
        if self.rc is None or self.rc_received_s is None:
            return False
        if time.monotonic() - self.rc_received_s > self.config.pose_timeout_s:
            return False
        if self.config.ch10_index < 0 or self.config.ch10_index >= len(self.rc.channels):
            return False
        return int(self.rc.channels[self.config.ch10_index]) >= self.config.ch10_threshold

    def publish_pair(self, left: float, right: float) -> None:
        left = clamp(left, 0.0, 100.0)
        right = clamp(right, 0.0, 100.0)
        self.last_left = left
        self.last_right = right
        msg = GripperCommandPair()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gripper"
        msg.left_pos = left
        msg.right_pos = right
        self.gripper_pair_pub.publish(msg)

    def open_gripper(self, repeats: int = 3) -> None:
        for _ in range(max(1, repeats)):
            self.publish_pair(self.config.gripper_open, self.config.gripper_open)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.03)

    def wait_feedback(self) -> GripperFeedbackState:
        deadline = time.monotonic() + max(0.5, self.config.feedback_timeout_s * 4.0)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.feedback_fresh() and self.feedback is not None:
                return self.feedback
        raise RuntimeError(f"No fresh gripper feedback on {self.config.gripper_feedback_topic}.")

    def hold_pair_step(self, left: float, right: float, duration_s: float) -> None:
        period = 1.0 / self.config.rate_hz
        deadline = time.monotonic() + max(0.0, duration_s)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            if not self.ch10_high():
                raise RuntimeError("CH10 went low during soft grasp.")
            self.publish_pair(left, right)
            time.sleep(period)

    def adjusted_pose(self, key: Literal["target", "box"]) -> PoseSample | None:
        pose = self.poses.get(key)
        if pose is None:
            return None
        if key == "target":
            return PoseSample(
                pose.x + self.config.target_offset_x,
                pose.y + self.config.target_offset_y,
                pose.z + self.config.target_offset_z,
                pose.yaw,
                pose.received_s,
                pose.frame_id,
            )
        return PoseSample(
            pose.x + self.config.box_offset_x,
            pose.y + self.config.box_offset_y,
            pose.z + self.config.box_offset_z,
            pose.yaw,
            pose.received_s,
            pose.frame_id,
        )

    def target_base_z(self, target: PoseSample) -> float:
        if self.config.target_pose_z_reference == "base":
            return target.z
        if self.config.target_pose_z_reference == "center":
            return target.z - 0.5 * self.config.target_height_m
        if self.config.target_pose_z_reference == "top":
            return target.z - self.config.target_height_m
        if self.config.target_pose_z_reference == "grasp":
            return target.z - self.config.target_grasp_height_m
        raise ValueError(f"Unknown target pose z reference: {self.config.target_pose_z_reference}")

    def target_grasp_drone_z(self, target: PoseSample) -> float:
        return self.target_base_z(target) + self.config.target_grasp_height_m + self.config.gripper_z_offset_m

    def target_hover_drone_z(self, target: PoseSample) -> float:
        return self.target_grasp_drone_z(target) + self.config.target_hover_clearance_m

    def box_place_drone_z(self, box: PoseSample) -> float:
        box_bottom_z = box.z - self.config.box_height_m
        gripper_z = box_bottom_z + self.config.target_grasp_height_m + self.config.box_place_bottom_clearance_m
        return gripper_z + self.config.gripper_z_offset_m

    def box_hover_drone_z(self, box: PoseSample) -> float:
        return box.z + self.config.box_hover_gripper_clearance_m + self.config.gripper_z_offset_m

    def log_status(self) -> None:
        now_s = time.monotonic()
        if now_s - self.last_status_s < 0.5:
            return
        self.last_status_s = now_s

        drone = self.poses.get("drone")
        target = self.adjusted_pose("target")
        box = self.adjusted_pose("box")
        parts = []
        for key in ("drone", "target", "box"):
            pose = self.poses.get(key)
            if pose is None:
                parts.append(f"{key}=missing")
            else:
                parts.append(f"{key}_age={now_s - pose.received_s:.2f}s")

        rc_state = "HIGH" if self.ch10_high() else "LOW"
        line = f"CH10={rc_state} state={self.state} " + " ".join(parts)

        if drone is not None and target is not None:
            grasp_z = self.target_grasp_drone_z(target)
            hover_z = self.target_hover_drone_z(target)
            line += (
                f" | target grasp err dx={drone.x - target.x:+.3f} "
                f"dy={drone.y - target.y:+.3f} dz={drone.z - grasp_z:+.3f} "
                f"(grasp_z={grasp_z:.3f}, hover_z={hover_z:.3f})"
            )
        if drone is not None and box is not None:
            place_z = self.box_place_drone_z(box)
            hover_z = self.box_hover_drone_z(box)
            line += (
                f" | box place err dx={drone.x - box.x:+.3f} "
                f"dy={drone.y - box.y:+.3f} dz={drone.z - place_z:+.3f} "
                f"(place_z={place_z:.3f}, hover_z={hover_z:.3f})"
            )
        if self.feedback is not None:
            line += (
                f" | grip L={self.feedback.left_pos:.1f}/{self.feedback.left_goal_pos:.1f} "
                f"R={self.feedback.right_pos:.1f}/{self.feedback.right_goal_pos:.1f} "
                f"load=({self.feedback.left_load:.0f},{self.feedback.right_load:.0f}) "
                f"cur=({self.feedback.left_current:.0f},{self.feedback.right_current:.0f})"
            )

        self.get_logger().info(line)

    def run_soft_grasp_once(self) -> None:
        controller = SoftGraspController(
            config=SoftGraspConfig(
                gripper_open=self.config.gripper_open,
                grasp_step_size=self.config.grasp_step_size,
                grasp_step_settle_s=self.config.grasp_step_settle_s,
                grasp_close_min=self.config.grasp_close_min,
                grasp_contact_current_delta=self.config.grasp_contact_current_delta,
                grasp_contact_load_delta=self.config.grasp_contact_load_delta,
                grasp_position_error_threshold=self.config.grasp_position_error_threshold,
                grasp_angle_contact_delta=self.config.grasp_angle_contact_delta,
                grasp_stall_delta=self.config.grasp_stall_delta,
                grasp_contact_confirm_steps=self.config.grasp_contact_confirm_steps,
                grasp_balance_load_diff=self.config.grasp_balance_load_diff,
                grasp_balance_step=self.config.grasp_balance_step,
                grasp_max_balance_steps=self.config.grasp_max_balance_steps,
                grasp_angle_balance_diff=self.config.grasp_angle_balance_diff,
            ),
            log=self.get_logger().info,
            publish_pair=self.publish_pair,
            open_gripper=self.open_gripper,
            wait_feedback=self.wait_feedback,
            hold_pair_step=self.hold_pair_step,
        )
        result = controller.run()
        self.last_left = result.left_goal
        self.last_right = result.right_goal
        self.state = "HOLDING"

    def run(self) -> None:
        period = 1.0 / self.config.rate_hz
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            self.log_status()

            if not self.ch10_high():
                if self.state != "IDLE":
                    self.get_logger().info("CH10 low: opening gripper and resetting soft grasp.")
                self.state = "IDLE"
                if time.monotonic() - self.last_open_publish_s >= 0.2:
                    self.last_open_publish_s = time.monotonic()
                    self.open_gripper(1)
                time.sleep(period)
                continue

            if self.state == "IDLE":
                self.state = "GRASPING"
                try:
                    self.run_soft_grasp_once()
                except Exception as exc:
                    self.get_logger().warn(f"Soft grasp stopped: {exc}")
                    self.open_gripper(5)
                    self.state = "IDLE"
                continue

            if self.state == "HOLDING":
                self.publish_pair(self.last_left, self.last_right)

            time.sleep(period)


def parse_args() -> HandheldConfig:
    parser = argparse.ArgumentParser(description="Handheld soft-grasp tuning without flight or recording.")
    parser.add_argument("--drone-pose-topic", default="/mavros/vision_pose/pose")
    parser.add_argument("--target-pose-topic", default="/strawberry_bear/pose")
    parser.add_argument("--box-pose-topic", default="/box1/pose")
    parser.add_argument("--rc-topic", default="/mavros/rc/in")
    parser.add_argument("--gripper-command-pair-topic", default="/gripper/command_pair")
    parser.add_argument("--gripper-feedback-topic", default="/gripper/feedback")
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--pose-timeout-s", type=float, default=0.8)
    parser.add_argument("--feedback-timeout-s", type=float, default=0.8)
    parser.add_argument("--ch10-index", type=int, default=9)
    parser.add_argument("--ch10-threshold", type=int, default=1500)
    parser.add_argument("--gripper-open", type=float, default=100.0)
    parser.add_argument("--gripper-z-offset-m", type=float, default=0.25)
    parser.add_argument("--target-height-m", type=float, default=0.30)
    parser.add_argument("--target-grasp-height-m", type=float, default=0.17)
    parser.add_argument("--target-pose-z-reference", choices=("base", "center", "top", "grasp"), default="center")
    parser.add_argument("--target-hover-clearance-m", type=float, default=0.45)
    parser.add_argument("--box-height-m", type=float, default=0.14)
    parser.add_argument("--box-hover-gripper-clearance-m", type=float, default=0.55)
    parser.add_argument("--box-place-bottom-clearance-m", type=float, default=0.03)
    parser.add_argument("--target-offset-x", type=float, default=0.0)
    parser.add_argument("--target-offset-y", type=float, default=0.0)
    parser.add_argument("--target-offset-z", type=float, default=0.0)
    parser.add_argument("--box-offset-x", type=float, default=0.0)
    parser.add_argument("--box-offset-y", type=float, default=0.0)
    parser.add_argument("--box-offset-z", type=float, default=0.0)
    parser.add_argument("--grasp-step-size", type=float, default=3.0)
    parser.add_argument("--grasp-step-settle-s", type=float, default=0.10)
    parser.add_argument("--grasp-close-min", type=float, default=15.0)
    parser.add_argument("--grasp-contact-current-delta", type=float, default=100.0)
    parser.add_argument("--grasp-contact-load-delta", type=float, default=60.0)
    parser.add_argument("--grasp-position-error-threshold", type=float, default=3.0)
    parser.add_argument("--grasp-angle-contact-delta", type=float, default=3.0)
    parser.add_argument("--grasp-stall-delta", type=float, default=0.8)
    parser.add_argument("--grasp-contact-confirm-steps", type=int, default=2)
    parser.add_argument("--grasp-balance-load-diff", type=float, default=60.0)
    parser.add_argument("--grasp-balance-step", type=float, default=1.5)
    parser.add_argument("--grasp-max-balance-steps", type=int, default=8)
    parser.add_argument("--grasp-angle-balance-diff", type=float, default=5.0)

    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    if args.rate_hz <= 0.0:
        raise ValueError("--rate-hz must be positive.")
    if args.ch10_index < 0:
        raise ValueError("--ch10-index must be non-negative.")
    if args.grasp_step_size <= 0.0 or args.grasp_step_settle_s <= 0.0:
        raise ValueError("--grasp-step-size and --grasp-step-settle-s must be positive.")
    if not 0.0 <= args.grasp_close_min <= args.gripper_open:
        raise ValueError("--grasp-close-min must be within [0, --gripper-open].")
    if args.grasp_contact_confirm_steps < 1:
        raise ValueError("--grasp-contact-confirm-steps must be >= 1.")
    if args.grasp_max_balance_steps < 0:
        raise ValueError("--grasp-max-balance-steps must be >= 0.")

    return HandheldConfig(**vars(args))


def main() -> None:
    config = parse_args()
    rclpy.init()
    node = HandheldGraspTune(config)
    try:
        node.run()
    except KeyboardInterrupt:
        node.open_gripper(5)
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
