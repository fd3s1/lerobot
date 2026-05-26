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
from quadrotor_msgs.msg import GripperCommandPair, GripperFeedback, TakeoffLand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64, String

from gripper_soft_grasp_lib import GripperFeedbackState, SoftGraspConfig, SoftGraspController


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return normalize_angle(math.atan2(siny_cosp, cosy_cosp))


def quaternion_to_roll_pitch_yaw(msg: PoseStamped) -> tuple[float, float, float]:
    q = msg.pose.orientation
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    yaw = quaternion_to_yaw(msg)
    return roll, pitch, yaw


@dataclass
class PoseSample:
    x: float
    y: float
    z: float
    yaw: float
    received_s: float
    frame_id: str
    roll: float = 0.0
    pitch: float = 0.0


@dataclass(frozen=True)
class AutoConfig:
    drone_pose_topic: str
    target_pose_topic: str
    box_pose_topic: str
    cmd_topic: str
    gripper_topic: str
    gripper_command_pair_topic: str
    gripper_feedback_topic: str
    takeoff_land_topic: str
    px4ctrl_state_topic: str
    record_status_topic: str
    record_gate_topic: str
    record_gate_value: str
    frame_id: str
    rate_hz: float
    max_speed: float
    approach_speed: float
    lift_speed: float
    payload_lift_speed: float
    payload_transfer_speed: float
    post_grasp_settle_s: float
    post_lift_settle_s: float
    smooth_trajectory: bool
    takeoff_forward_comp_m: float
    payload_lift_forward_comp_m: float
    takeoff_comp_x: float
    takeoff_comp_y: float
    takeoff_comp_z: float
    payload_lift_comp_x: float
    payload_lift_comp_y: float
    payload_lift_comp_z: float
    gripper_z_offset_m: float
    target_height_m: float
    target_grasp_height_m: float
    target_pose_z_reference: Literal["base", "center", "top", "grasp"]
    target_hover_clearance_m: float
    box_length_m: float
    box_width_m: float
    box_height_m: float
    box_hover_gripper_clearance_m: float
    box_place_bottom_clearance_m: float
    target_offset_x: float
    target_offset_y: float
    target_offset_z: float
    box_offset_x: float
    box_offset_y: float
    box_offset_z: float
    target_hover_z_offset: float | None
    target_grasp_z_offset: float | None
    box_hover_z_offset: float | None
    box_place_z_offset: float | None
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float
    pose_timeout_s: float
    state_timeout_s: float
    stable_duration_s: float
    stable_pos_tolerance_m: float
    takeoff_timeout_s: float
    cmd_ctrl_timeout_s: float
    record_ready_timeout_s: float
    record_duration_s: float
    record_start_hold_s: float
    grasp_mode: Literal["soft", "continuous_center"]
    gripper_open: float
    gripper_closed: float
    gripper_close_duration_s: float
    gripper_open_duration_s: float
    grasp_step_size: float
    grasp_step_settle_s: float
    grasp_open_timeout_s: float
    grasp_open_tolerance: float
    grasp_step_timeout_s: float
    grasp_goal_tolerance: float
    grasp_close_min: float
    grasp_contact_current_delta: float
    grasp_contact_load_delta: float
    grasp_position_error_threshold: float
    grasp_angle_contact_delta: float
    grasp_stall_delta: float
    grasp_contact_min_close_delta: float
    grasp_contact_confirm_steps: int
    grasp_balance_load_diff: float
    grasp_balance_step: float
    grasp_max_balance_steps: int
    grasp_angle_balance_diff: float
    grasp_compliance_radius_m: float
    grasp_compliance_elastic_gain: float
    grasp_abort_drift_m: float
    grasp_attitude_soft_rad: float
    grasp_abort_attitude_rad: float
    release_retreat_up_m: float
    release_retreat_forward_m: float
    retreat_speed: float
    landing_mode: Literal["cmd", "auto", "none"]
    cmd_land_speed: float
    cmd_land_z: float | None
    cmd_land_z_offset_m: float
    command_stop_before_land_s: float
    no_land: bool


class AutoGraspPlaceDataset(Node):
    def __init__(self, config: AutoConfig) -> None:
        super().__init__("auto_grasp_place_dataset")
        self.config = config
        self.poses: dict[str, PoseSample] = {}
        self.latched_poses: dict[str, PoseSample] = {}
        self.last_latch_warn_s: dict[str, float] = {}
        self.gripper_feedback: GripperFeedbackState | None = None
        self.last_gripper_left = config.gripper_open
        self.last_gripper_right = config.gripper_open
        self.px4ctrl_state: str | None = None
        self.record_status: str | None = None
        self.pose_subscriptions = []

        self._create_pose_subscription("drone", config.drone_pose_topic)
        self._create_pose_subscription("target", config.target_pose_topic)
        self._create_pose_subscription("box", config.box_pose_topic)
        self.create_subscription(String, config.px4ctrl_state_topic, self._state_cb, 10)
        self.create_subscription(GripperFeedback, config.gripper_feedback_topic, self._gripper_feedback_cb, 10)

        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        if config.record_status_topic:
            self.create_subscription(String, config.record_status_topic, self._record_status_cb, status_qos)

        self.cmd_pub = self.create_publisher(PoseStamped, config.cmd_topic, 10)
        self.gripper_pub = self.create_publisher(Float64, config.gripper_topic, 10)
        self.gripper_pair_pub = self.create_publisher(GripperCommandPair, config.gripper_command_pair_topic, 10)
        self.takeoff_land_pub = self.create_publisher(TakeoffLand, config.takeoff_land_topic, 10)
        self.record_gate_pub = self.create_publisher(String, config.record_gate_topic, 10)

    def _create_pose_subscription(self, key: str, topic: str) -> None:
        # A BEST_EFFORT subscription is compatible with both VRPN BEST_EFFORT
        # publishers and MAVROS RELIABLE publishers, and avoids noisy QoS warnings.
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        callback = self._pose_cb(key)
        self.pose_subscriptions.append(self.create_subscription(PoseStamped, topic, callback, best_effort_qos))

    def _pose_cb(self, key: str):
        def callback(msg: PoseStamped) -> None:
            roll, pitch, yaw = quaternion_to_roll_pitch_yaw(msg)
            self.poses[key] = PoseSample(
                x=float(msg.pose.position.x),
                y=float(msg.pose.position.y),
                z=float(msg.pose.position.z),
                yaw=yaw,
                received_s=time.monotonic(),
                frame_id=str(msg.header.frame_id),
                roll=roll,
                pitch=pitch,
            )

        return callback

    def _state_cb(self, msg: String) -> None:
        self.px4ctrl_state = str(msg.data)

    def _record_status_cb(self, msg: String) -> None:
        self.record_status = str(msg.data)

    def _gripper_feedback_cb(self, msg: GripperFeedback) -> None:
        self.gripper_feedback = GripperFeedbackState(
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

    def spin_sleep(self, seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, seconds)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def pose_fresh(self, key: str) -> bool:
        pose = self.poses.get(key)
        return pose is not None and time.monotonic() - pose.received_s <= self.config.pose_timeout_s

    def wait_for_record_ready(self) -> None:
        if not self.config.record_status_topic:
            return

        self.get_logger().info(
            f"Waiting for record status WAITING_GATE on {self.config.record_status_topic} before takeoff."
        )
        deadline = time.monotonic() + self.config.record_ready_timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.record_status == "WAITING_GATE":
                self.get_logger().info("Record is waiting at gate; automatic takeoff can start.")
                return

        raise RuntimeError(
            f"Timed out waiting for {self.config.record_status_topic}=WAITING_GATE. "
            "Record was not confirmed ready, so takeoff was not triggered."
        )

    def wait_for_state(self, desired: str, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.px4ctrl_state == desired:
                self.get_logger().info(f"px4ctrl state reached {desired}.")
                return
        raise RuntimeError(f"Timed out waiting for px4ctrl state {desired}; latest={self.px4ctrl_state!r}.")

    def wait_for_fresh_poses(self) -> None:
        self.get_logger().info(
            "Waiting for fresh poses: "
            f"drone={self.config.drone_pose_topic}, "
            f"target={self.config.target_pose_topic}, "
            f"box={self.config.box_pose_topic}."
        )
        last_log_s = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(self.pose_fresh(key) for key in ("drone", "target", "box")):
                return
            now_s = time.monotonic()
            if now_s - last_log_s >= 2.0:
                last_log_s = now_s
                status_parts = []
                for key, topic in (
                    ("drone", self.config.drone_pose_topic),
                    ("target", self.config.target_pose_topic),
                    ("box", self.config.box_pose_topic),
                ):
                    pose = self.poses.get(key)
                    if pose is None:
                        status_parts.append(f"{key}({topic})=missing")
                    else:
                        status_parts.append(f"{key}({topic}) age={now_s - pose.received_s:.2f}s")
                self.get_logger().info("Still waiting for fresh poses: " + ", ".join(status_parts))

    def wait_for_stable_pose(self, key: str) -> PoseSample:
        reference: PoseSample | None = None
        stable_since: float | None = None
        deadline = time.monotonic() + self.config.state_timeout_s

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            pose = self.poses.get(key)
            if pose is None or not self.pose_fresh(key):
                reference = None
                stable_since = None
                continue

            if reference is None:
                reference = pose
                stable_since = time.monotonic()
                continue

            delta = math.dist((pose.x, pose.y, pose.z), (reference.x, reference.y, reference.z))
            if delta > self.config.stable_pos_tolerance_m:
                reference = pose
                stable_since = time.monotonic()
                continue

            if stable_since is not None and time.monotonic() - stable_since >= self.config.stable_duration_s:
                return pose

        raise RuntimeError(f"Pose {key!r} did not become stable before timeout.")

    def publish_cmd(self, pose: PoseSample) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.config.frame_id
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        msg.pose.position.z = pose.z
        qx, qy, qz, qw = yaw_to_quaternion(pose.yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.cmd_pub.publish(msg)

    def publish_gripper(self, target: float, repeats: int = 1, interval_s: float = 0.05) -> None:
        msg = Float64()
        msg.data = clamp(target, 0.0, 100.0)
        for _ in range(max(1, repeats)):
            self.gripper_pub.publish(msg)
            self.publish_gripper_pair(msg.data, msg.data, publish_scalar=False)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(max(0.0, interval_s))

    def publish_gripper_pair(
        self,
        left: float,
        right: float,
        *,
        publish_scalar: bool = True,
    ) -> None:
        left = clamp(left, 0.0, 100.0)
        right = clamp(right, 0.0, 100.0)
        self.last_gripper_left = left
        self.last_gripper_right = right
        msg = GripperCommandPair()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gripper"
        msg.left_pos = left
        msg.right_pos = right
        self.gripper_pair_pub.publish(msg)
        if publish_scalar:
            scalar = Float64()
            scalar.data = 0.5 * (left + right)
            self.gripper_pub.publish(scalar)

    def gripper_feedback_fresh(self) -> bool:
        feedback = self.gripper_feedback
        return feedback is not None and time.monotonic() - feedback.received_s <= self.config.pose_timeout_s

    def wait_for_gripper_feedback(self) -> GripperFeedbackState:
        deadline = time.monotonic() + self.config.state_timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.gripper_feedback_fresh() and self.gripper_feedback is not None:
                return self.gripper_feedback
        raise RuntimeError(f"No fresh gripper feedback on {self.config.gripper_feedback_topic}.")

    def publish_gripper_ramp(
        self,
        start: float,
        end: float,
        duration_s: float,
        hold_pose: PoseSample | None = None,
    ) -> None:
        steps = max(2, int(math.ceil(duration_s * self.config.rate_hz)))
        period = 1.0 / self.config.rate_hz
        next_tick = time.monotonic()
        for i in range(steps + 1):
            alpha = i / steps
            if hold_pose is not None:
                self.publish_cmd(hold_pose)
            self.publish_gripper(start + (end - start) * alpha, repeats=1, interval_s=0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))

            if hold_pose is not None and self.px4ctrl_state != "CMD_CTRL":
                raise RuntimeError(
                    f"px4ctrl left CMD_CTRL during gripper ramp; latest={self.px4ctrl_state!r}."
                )

    def publish_takeoff(self) -> None:
        msg = TakeoffLand()
        msg.takeoff_land_cmd = TakeoffLand.TAKEOFF
        self.get_logger().info("Publishing TAKEOFF.")
        for _ in range(5):
            self.takeoff_land_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.2)

    def publish_land(self) -> None:
        msg = TakeoffLand()
        msg.takeoff_land_cmd = TakeoffLand.LAND
        self.get_logger().info("Publishing LAND with gripper open.")
        for _ in range(5):
            self.publish_gripper(self.config.gripper_open, repeats=1, interval_s=0.0)
            self.takeoff_land_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.2)

    def publish_record_gate(self) -> None:
        msg = String()
        msg.data = self.config.record_gate_value
        self.get_logger().info(
            f"Publishing record gate {self.config.record_gate_topic}={self.config.record_gate_value!r}."
        )
        for _ in range(5):
            self.record_gate_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

    def checked_pose(self, x: float, y: float, z: float, yaw: float) -> PoseSample:
        if not (self.config.x_min <= x <= self.config.x_max):
            raise RuntimeError(f"Waypoint x={x:.3f} outside [{self.config.x_min}, {self.config.x_max}].")
        if not (self.config.y_min <= y <= self.config.y_max):
            raise RuntimeError(f"Waypoint y={y:.3f} outside [{self.config.y_min}, {self.config.y_max}].")
        if not (self.config.z_min <= z <= self.config.z_max):
            raise RuntimeError(f"Waypoint z={z:.3f} outside [{self.config.z_min}, {self.config.z_max}].")
        return PoseSample(x=x, y=y, z=z, yaw=yaw, received_s=time.monotonic(), frame_id=self.config.frame_id)

    def offset_pose(self, pose: PoseSample, dx: float, dy: float, dz: float) -> PoseSample:
        return self.checked_pose(
            pose.x + dx,
            pose.y + dy,
            pose.z + dz,
            pose.yaw,
        )

    def adjusted_object_pose(self, key: Literal["target", "box"], require_fresh: bool = True) -> PoseSample:
        pose = self.poses.get(key)
        if pose is None:
            raise RuntimeError(f"No {key} pose has been received.")
        if require_fresh and not self.pose_fresh(key):
            raise RuntimeError(f"{key} pose is stale during live waypoint update.")

        if key == "target":
            adjusted = self.offset_pose(
                pose,
                self.config.target_offset_x,
                self.config.target_offset_y,
                self.config.target_offset_z,
            )
        else:
            adjusted = self.offset_pose(
                pose,
                self.config.box_offset_x,
                self.config.box_offset_y,
                self.config.box_offset_z,
            )
        self.latched_poses[key] = adjusted
        return adjusted

    def latched_or_fresh_object_pose(self, key: Literal["target", "box"]) -> PoseSample:
        if self.pose_fresh(key):
            return self.adjusted_object_pose(key, require_fresh=True)
        pose = self.latched_poses.get(key)
        if pose is None:
            raise RuntimeError(f"No latched {key} pose is available while the topic is occluded/stale.")
        now_s = time.monotonic()
        if now_s - self.last_latch_warn_s.get(key, 0.0) >= 2.0:
            self.last_latch_warn_s[key] = now_s
            self.get_logger().warn(
                f"Using latched {key} pose because latest topic data is not fresh. "
                "This is expected during temporary occlusion."
            )
        return pose

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
        if self.config.target_grasp_z_offset is not None:
            return target.z + self.config.target_grasp_z_offset
        gripper_z = self.target_base_z(target) + self.config.target_grasp_height_m
        return gripper_z + self.config.gripper_z_offset_m

    def target_hover_drone_z(self, target: PoseSample) -> float:
        if self.config.target_hover_z_offset is not None:
            return target.z + self.config.target_hover_z_offset
        return self.target_grasp_drone_z(target) + self.config.target_hover_clearance_m

    def box_place_drone_z(self, box: PoseSample) -> float:
        if self.config.box_place_z_offset is not None:
            return box.z + self.config.box_place_z_offset
        box_bottom_z = box.z - self.config.box_height_m
        gripper_z = box_bottom_z + self.config.target_grasp_height_m + self.config.box_place_bottom_clearance_m
        return gripper_z + self.config.gripper_z_offset_m

    def box_hover_drone_z(self, box: PoseSample) -> float:
        if self.config.box_hover_z_offset is not None:
            return box.z + self.config.box_hover_z_offset
        gripper_z = box.z + self.config.box_hover_gripper_clearance_m
        return gripper_z + self.config.gripper_z_offset_m

    def validate_box_fit(self) -> None:
        margin_x = 0.5 * (self.config.box_length_m - self.config.target_height_m)
        margin_y = 0.5 * (self.config.box_width_m - self.config.target_height_m)
        if margin_x < 0.0 or margin_y < 0.0:
            raise RuntimeError(
                "Target height is larger than the box footprint assumption. "
                f"target_height={self.config.target_height_m:.3f}m, "
                f"box_length={self.config.box_length_m:.3f}m, box_width={self.config.box_width_m:.3f}m."
            )
        self.get_logger().info(
            "Box center placement clearance estimate: "
            f"x_margin={margin_x:.3f}m, y_margin={margin_y:.3f}m."
        )

    def pose_at_z(self, obj: PoseSample, drone_z: float, yaw: float) -> PoseSample:
        return self.checked_pose(obj.x, obj.y, drone_z, yaw)

    def pose_relative_forward(self, pose: PoseSample, forward_m: float, up_m: float = 0.0) -> PoseSample:
        return self.checked_pose(
            x=pose.x + forward_m * math.cos(pose.yaw),
            y=pose.y + forward_m * math.sin(pose.yaw),
            z=pose.z + up_m,
            yaw=pose.yaw,
        )

    def fly_segment(
        self,
        start: PoseSample,
        end: PoseSample,
        speed: float,
        label: str,
        keep_gripper_open: bool = False,
    ) -> PoseSample:
        distance = math.dist((start.x, start.y, start.z), (end.x, end.y, end.z))
        duration_scale = 1.5 if self.config.smooth_trajectory else 1.0
        duration = max(distance / max(speed, 1e-3) * duration_scale, 1.0 / self.config.rate_hz)
        steps = max(1, int(math.ceil(duration * self.config.rate_hz)))
        period = 1.0 / self.config.rate_hz
        self.get_logger().info(
            f"{label}: distance={distance:.3f}m speed_limit={speed:.3f}m/s duration={duration:.2f}s."
        )

        next_tick = time.monotonic()
        for i in range(steps + 1):
            t = i / steps
            alpha = (10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5) if self.config.smooth_trajectory else t
            yaw_delta = normalize_angle(end.yaw - start.yaw)
            pose = self.checked_pose(
                x=start.x + (end.x - start.x) * alpha,
                y=start.y + (end.y - start.y) * alpha,
                z=start.z + (end.z - start.z) * alpha,
                yaw=normalize_angle(start.yaw + yaw_delta * alpha),
            )
            self.publish_cmd(pose)
            if keep_gripper_open:
                self.publish_gripper(self.config.gripper_open, repeats=1, interval_s=0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))

            if self.px4ctrl_state != "CMD_CTRL":
                raise RuntimeError(f"px4ctrl left CMD_CTRL during {label}; latest={self.px4ctrl_state!r}.")

        return end

    def fly_to_live_waypoint(
        self,
        start: PoseSample,
        waypoint_fn,
        speed: float,
        label: str,
        arrival_tolerance_m: float = 0.04,
        settle_s: float = 0.3,
    ) -> PoseSample:
        current = start
        initial_end = waypoint_fn()
        initial_distance = math.dist(
            (current.x, current.y, current.z),
            (initial_end.x, initial_end.y, initial_end.z),
        )
        max_duration = max(initial_distance / max(speed, 1e-3) * 3.0 + 5.0, 5.0)
        period = 1.0 / self.config.rate_hz
        step_max = speed * period
        deadline = time.monotonic() + max_duration
        arrived_since: float | None = None
        last_log_s = 0.0

        self.get_logger().info(
            f"{label}: live waypoint tracking, initial_distance={initial_distance:.3f}m, "
            f"speed_limit={speed:.3f}m/s, timeout={max_duration:.2f}s."
        )

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            end = waypoint_fn()
            dx = end.x - current.x
            dy = end.y - current.y
            dz = end.z - current.z
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)

            if distance <= arrival_tolerance_m:
                current = self.checked_pose(end.x, end.y, end.z, end.yaw)
                if arrived_since is None:
                    arrived_since = time.monotonic()
                elif time.monotonic() - arrived_since >= settle_s:
                    self.publish_cmd(current)
                    return current
            else:
                arrived_since = None
                alpha = min(1.0, step_max / max(distance, 1e-6))
                yaw_delta = normalize_angle(end.yaw - current.yaw)
                current = self.checked_pose(
                    x=current.x + dx * alpha,
                    y=current.y + dy * alpha,
                    z=current.z + dz * alpha,
                    yaw=normalize_angle(current.yaw + yaw_delta * alpha),
                )

            self.publish_cmd(current)
            now_s = time.monotonic()
            if now_s - last_log_s >= 2.0:
                last_log_s = now_s
                self.get_logger().info(
                    f"{label}: tracking distance={distance:.3f}m toward "
                    f"x={end.x:.3f}, y={end.y:.3f}, z={end.z:.3f}."
                )
            time.sleep(period)

            if self.px4ctrl_state != "CMD_CTRL":
                raise RuntimeError(f"px4ctrl left CMD_CTRL during {label}; latest={self.px4ctrl_state!r}.")

        raise RuntimeError(f"Timed out during {label}; latest command x={current.x:.3f}, y={current.y:.3f}, z={current.z:.3f}.")

    def hold_cmd(self, pose: PoseSample, duration_s: float, keep_gripper_open: bool = False) -> None:
        period = 1.0 / self.config.rate_hz
        deadline = time.monotonic() + max(0.0, duration_s)
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_cmd(pose)
            if keep_gripper_open:
                self.publish_gripper(self.config.gripper_open, repeats=1, interval_s=0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def compliant_pose(self, reference: PoseSample) -> PoseSample:
        drone = self.poses.get("drone")
        if drone is None or not self.pose_fresh("drone"):
            return reference

        attitude = max(abs(drone.roll), abs(drone.pitch))
        if attitude > self.config.grasp_abort_attitude_rad:
            self.publish_gripper(self.config.gripper_open, repeats=5)
            raise RuntimeError(
                f"Drone attitude too large during compliant grasp: "
                f"roll={drone.roll:.3f}rad pitch={drone.pitch:.3f}rad, "
                f"threshold={self.config.grasp_abort_attitude_rad:.3f}rad."
            )

        dx = drone.x - reference.x
        dy = drone.y - reference.y
        drift_xy = math.hypot(dx, dy)
        if drift_xy > self.config.grasp_abort_drift_m:
            self.publish_gripper(self.config.gripper_open, repeats=5)
            raise RuntimeError(
                f"Drone drifted {drift_xy:.3f}m during compliant grasp; "
                f"abort threshold is {self.config.grasp_abort_drift_m:.3f}m."
            )

        elastic_gain = self.config.grasp_compliance_elastic_gain
        if attitude > self.config.grasp_attitude_soft_rad:
            attitude_span = max(
                self.config.grasp_abort_attitude_rad - self.config.grasp_attitude_soft_rad,
                1e-6,
            )
            attitude_ratio = clamp(
                (attitude - self.config.grasp_attitude_soft_rad) / attitude_span,
                0.0,
                1.0,
            )
            # As attitude grows, leave more XY error for the position controller
            # so the vehicle is encouraged to level instead of following the load.
            elastic_gain = min(1.0, elastic_gain + (1.0 - elastic_gain) * attitude_ratio)

        free_radius = self.config.grasp_compliance_radius_m
        if drift_xy <= free_radius:
            allowed = (1.0 - elastic_gain) * drift_xy
        else:
            overflow = drift_xy - free_radius
            allowed = min(
                drift_xy,
                (1.0 - elastic_gain) * free_radius
                + (1.0 - 0.5 * elastic_gain) * overflow,
            )
        if drift_xy > 1e-6:
            scale = allowed / drift_xy
            x = reference.x + dx * scale
            y = reference.y + dy * scale
        else:
            x = reference.x
            y = reference.y
        return self.checked_pose(x, y, reference.z, reference.yaw)

    def hold_cmd_compliant(self, reference: PoseSample, duration_s: float) -> PoseSample:
        period = 1.0 / self.config.rate_hz
        deadline = time.monotonic() + max(0.0, duration_s)
        latest = reference
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            latest = self.compliant_pose(reference)
            self.publish_cmd(latest)
            time.sleep(period)
            if self.px4ctrl_state != "CMD_CTRL":
                raise RuntimeError(f"px4ctrl left CMD_CTRL during compliant hold; latest={self.px4ctrl_state!r}.")
        return latest

    def _hold_pair_step(
        self,
        reference: PoseSample,
        left_goal: float,
        right_goal: float,
        duration_s: float,
    ) -> PoseSample:
        period = 1.0 / self.config.rate_hz
        deadline = time.monotonic() + max(0.0, duration_s)
        latest = reference
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            latest = self.compliant_pose(reference)
            self.publish_cmd(latest)
            self.publish_gripper_pair(left_goal, right_goal)
            time.sleep(period)
            if self.px4ctrl_state != "CMD_CTRL":
                raise RuntimeError(f"px4ctrl left CMD_CTRL during soft grasp; latest={self.px4ctrl_state!r}.")
        return latest

    def soft_grasp(self, reference: PoseSample) -> PoseSample:
        latest: PoseSample = self.hold_cmd_compliant(reference, 0.3)

        if self.config.grasp_mode == "continuous_center":
            return self.continuous_center_grasp(latest)

        def hold_pair(left_goal: float, right_goal: float, duration_s: float) -> PoseSample:
            nonlocal latest
            latest = self._hold_pair_step(reference, left_goal, right_goal, duration_s)
            return latest

        controller = SoftGraspController(
            config=SoftGraspConfig(
                gripper_open=self.config.gripper_open,
                grasp_open_timeout_s=self.config.grasp_open_timeout_s,
                grasp_open_tolerance=self.config.grasp_open_tolerance,
                grasp_step_size=self.config.grasp_step_size,
                grasp_step_settle_s=self.config.grasp_step_settle_s,
                grasp_step_timeout_s=self.config.grasp_step_timeout_s,
                grasp_goal_tolerance=self.config.grasp_goal_tolerance,
                grasp_close_min=self.config.grasp_close_min,
                grasp_contact_current_delta=self.config.grasp_contact_current_delta,
                grasp_contact_load_delta=self.config.grasp_contact_load_delta,
                grasp_position_error_threshold=self.config.grasp_position_error_threshold,
                grasp_angle_contact_delta=self.config.grasp_angle_contact_delta,
                grasp_stall_delta=self.config.grasp_stall_delta,
                grasp_contact_min_close_delta=self.config.grasp_contact_min_close_delta,
                grasp_contact_confirm_steps=self.config.grasp_contact_confirm_steps,
                grasp_balance_load_diff=self.config.grasp_balance_load_diff,
                grasp_balance_step=self.config.grasp_balance_step,
                grasp_max_balance_steps=self.config.grasp_max_balance_steps,
                grasp_angle_balance_diff=self.config.grasp_angle_balance_diff,
            ),
            log=self.get_logger().info,
            publish_pair=lambda left, right: self.publish_gripper_pair(left, right),
            open_gripper=lambda repeats: self.publish_gripper(self.config.gripper_open, repeats=repeats),
            wait_feedback=self.wait_for_gripper_feedback,
            hold_pair_step=hold_pair,
        )
        result = controller.run()
        if isinstance(result.context, PoseSample):
            latest = result.context
        return latest

    def continuous_center_grasp(self, reference: PoseSample) -> PoseSample:
        cfg = self.config
        duration_s = max(0.0, cfg.gripper_close_duration_s)
        period = 1.0 / cfg.rate_hz
        steps = max(2, int(math.ceil(duration_s * cfg.rate_hz)))
        latest = reference
        self.get_logger().info(
            "Continuous center grasp: closing symmetrically "
            f"{cfg.gripper_open:.1f}->{cfg.gripper_closed:.1f} over {duration_s:.2f}s "
            "while XY remains compliant."
        )
        next_tick = time.monotonic()
        for i in range(steps + 1):
            rclpy.spin_once(self, timeout_sec=0.0)
            latest = self.compliant_pose(reference)
            t = i / steps
            alpha = 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5
            target = cfg.gripper_open + (cfg.gripper_closed - cfg.gripper_open) * alpha
            self.publish_cmd(latest)
            self.publish_gripper_pair(target, target)
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))
            if self.px4ctrl_state != "CMD_CTRL":
                raise RuntimeError(
                    f"px4ctrl left CMD_CTRL during continuous center grasp; latest={self.px4ctrl_state!r}."
                )
        return latest

    def finish_with_cmd_landing(self, current: PoseSample, pre_takeoff_drone: PoseSample, gate_s: float) -> None:
        landing_z = (
            self.config.cmd_land_z
            if self.config.cmd_land_z is not None
            else pre_takeoff_drone.z + self.config.cmd_land_z_offset_m
        )
        landing_pose = self.checked_pose(current.x, current.y, landing_z, current.yaw)
        self.get_logger().info(
            "CMD_CTRL landing: "
            f"z={landing_pose.z:.3f}m speed_limit={self.config.cmd_land_speed:.3f}m/s; "
            "gripper will stay open."
        )
        current = self.fly_segment(
            current,
            landing_pose,
            self.config.cmd_land_speed,
            "CMD_CTRL descent landing",
            keep_gripper_open=True,
        )

        elapsed_record_s = time.monotonic() - gate_s
        remaining_record_s = self.config.record_duration_s + 0.5 - elapsed_record_s
        if remaining_record_s > 0.0:
            self.get_logger().info(
                f"Holding CMD landing pose for {remaining_record_s:.1f}s until record episode is expected to finish."
            )
            self.hold_cmd(current, remaining_record_s, keep_gripper_open=True)

        self.get_logger().warn(
            "CMD_CTRL landing sequence finished. Auto LAND/disarm was not sent; disarm or switch mode manually if needed."
        )

    def run_sequence(self) -> None:
        self.wait_for_record_ready()
        self.wait_for_fresh_poses()
        target = self.wait_for_stable_pose("target")
        box = self.wait_for_stable_pose("box")
        drone = self.wait_for_stable_pose("drone")
        raw_target = target
        raw_box = box
        target = self.adjusted_object_pose("target", require_fresh=True)
        box = self.adjusted_object_pose("box", require_fresh=True)

        self.get_logger().info(
            f"Raw target pose: x={raw_target.x:.3f}, y={raw_target.y:.3f}, z={raw_target.z:.3f}; "
            f"raw box pose: x={raw_box.x:.3f}, y={raw_box.y:.3f}, z={raw_box.z:.3f}."
        )
        self.get_logger().info(
            f"Adjusted target pose: x={target.x:.3f}, y={target.y:.3f}, z={target.z:.3f}; "
            f"adjusted box pose: x={box.x:.3f}, y={box.y:.3f}, z={box.z:.3f}."
        )
        self.get_logger().info(
            "Planning offsets in mocap/map frame: "
            f"target=({self.config.target_offset_x:.3f}, {self.config.target_offset_y:.3f}, "
            f"{self.config.target_offset_z:.3f})m, "
            f"box=({self.config.box_offset_x:.3f}, {self.config.box_offset_y:.3f}, "
            f"{self.config.box_offset_z:.3f})m."
        )
        self.get_logger().info(
            "Geometry: "
            f"gripper_z_offset={self.config.gripper_z_offset_m:.3f}m, "
            f"target_height={self.config.target_height_m:.3f}m, "
            f"target_grasp_height={self.config.target_grasp_height_m:.3f}m, "
            f"target_z_ref={self.config.target_pose_z_reference}, "
            f"box_lwh=({self.config.box_length_m:.3f}, {self.config.box_width_m:.3f}, "
            f"{self.config.box_height_m:.3f})m."
        )
        self.validate_box_fit()

        self.publish_gripper(self.config.gripper_open, repeats=5)
        self.publish_takeoff()
        self.wait_for_state("AUTO_HOVER", self.config.takeoff_timeout_s)

        current_drone = self.poses.get("drone", drone)
        hold = self.checked_pose(current_drone.x, current_drone.y, current_drone.z, current_drone.yaw)
        self.get_logger().info("Publishing hold /position_cmd to enter CMD_CTRL.")
        hold_deadline = time.monotonic() + self.config.cmd_ctrl_timeout_s
        while rclpy.ok() and time.monotonic() < hold_deadline:
            self.publish_cmd(hold)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.px4ctrl_state == "CMD_CTRL":
                break
            time.sleep(1.0 / self.config.rate_hz)
        if self.px4ctrl_state != "CMD_CTRL":
            raise RuntimeError(f"Failed to enter CMD_CTRL; latest px4ctrl state={self.px4ctrl_state!r}.")

        if abs(self.config.takeoff_forward_comp_m) > 1e-6:
            recentered = self.pose_relative_forward(
                hold,
                forward_m=self.config.takeoff_forward_comp_m,
                up_m=0.0,
            )
            hold = self.fly_segment(
                hold,
                recentered,
                self.config.approach_speed,
                "Recenter forward after takeoff",
            )
        if any(abs(value) > 1e-6 for value in (
            self.config.takeoff_comp_x,
            self.config.takeoff_comp_y,
            self.config.takeoff_comp_z,
        )):
            compensated = self.offset_pose(
                hold,
                self.config.takeoff_comp_x,
                self.config.takeoff_comp_y,
                self.config.takeoff_comp_z,
            )
            self.get_logger().info(
                "Recenter in mocap/map frame after takeoff: "
                f"dx={self.config.takeoff_comp_x:.3f}, "
                f"dy={self.config.takeoff_comp_y:.3f}, "
                f"dz={self.config.takeoff_comp_z:.3f}."
            )
            hold = self.fly_segment(
                hold,
                compensated,
                self.config.approach_speed,
                "Recenter map-frame after takeoff",
            )

        self.publish_record_gate()
        gate_s = time.monotonic()
        self.hold_cmd(hold, self.config.record_start_hold_s)

        yaw = hold.yaw
        self.get_logger().info(
            "Object waypoints will be refreshed during approach and latched before occlusion-prone descent/grasp/place."
        )

        def target_hover_waypoint() -> PoseSample:
            live_target = self.latched_or_fresh_object_pose("target")
            return self.pose_at_z(live_target, self.target_hover_drone_z(live_target), yaw)

        def box_hover_waypoint() -> PoseSample:
            live_box = self.latched_or_fresh_object_pose("box")
            return self.pose_at_z(live_box, self.box_hover_drone_z(live_box), yaw)

        current = hold
        current = self.fly_to_live_waypoint(
            current,
            target_hover_waypoint,
            self.config.max_speed,
            "Fly to target hover with live target update",
        )

        target = self.latched_or_fresh_object_pose("target")
        target_above = self.pose_at_z(target, self.target_hover_drone_z(target), yaw)
        target_grasp = self.pose_at_z(target, self.target_grasp_drone_z(target), yaw)
        target_lift = self.pose_relative_forward(
            target_above,
            forward_m=self.config.payload_lift_forward_comp_m,
            up_m=0.0,
        )
        if any(abs(value) > 1e-6 for value in (
            self.config.payload_lift_comp_x,
            self.config.payload_lift_comp_y,
            self.config.payload_lift_comp_z,
        )):
            target_lift = self.offset_pose(
                target_lift,
                self.config.payload_lift_comp_x,
                self.config.payload_lift_comp_y,
                self.config.payload_lift_comp_z,
            )
            self.get_logger().info(
                "Payload lift compensation in mocap/map frame: "
                f"dx={self.config.payload_lift_comp_x:.3f}, "
                f"dy={self.config.payload_lift_comp_y:.3f}, "
                f"dz={self.config.payload_lift_comp_z:.3f}."
            )
        self.get_logger().info(
            "Latched target drone-center waypoints: "
            f"target_hover_z={target_above.z:.3f}, target_grasp_z={target_grasp.z:.3f}."
        )

        current = self.fly_segment(current, target_grasp, self.config.approach_speed, "Descend to grasp")
        current = self.soft_grasp(current)
        if self.config.post_grasp_settle_s > 0.0:
            self.get_logger().info(f"Holding after grasp for {self.config.post_grasp_settle_s:.2f}s to let payload settle.")
            current = self.hold_cmd_compliant(current, self.config.post_grasp_settle_s)
        current = self.fly_segment(current, target_lift, self.config.payload_lift_speed, "Lift object")
        if self.config.post_lift_settle_s > 0.0:
            self.get_logger().info(f"Holding after lift for {self.config.post_lift_settle_s:.2f}s to damp payload swing.")
            self.hold_cmd(current, self.config.post_lift_settle_s)
        current = self.fly_to_live_waypoint(
            current,
            box_hover_waypoint,
            self.config.payload_transfer_speed,
            "Fly to box hover with live box update",
        )

        box = self.latched_or_fresh_object_pose("box")
        box_above = self.pose_at_z(box, self.box_hover_drone_z(box), yaw)
        box_place = self.pose_at_z(box, self.box_place_drone_z(box), yaw)
        self.get_logger().info(
            "Latched box drone-center waypoints: "
            f"box_hover_z={box_above.z:.3f}, box_place_z={box_place.z:.3f}."
        )
        current = self.fly_segment(current, box_place, self.config.approach_speed, "Descend to box")
        self.get_logger().info("Opening gripper to release.")
        self.publish_gripper_ramp(
            0.5 * (self.last_gripper_left + self.last_gripper_right),
            self.config.gripper_open,
            self.config.gripper_open_duration_s,
            hold_pose=current,
        )
        self.publish_gripper(self.config.gripper_open, repeats=5)

        release_up = self.pose_relative_forward(current, forward_m=0.0, up_m=self.config.release_retreat_up_m)
        release_forward = self.pose_relative_forward(
            release_up,
            forward_m=self.config.release_retreat_forward_m,
            up_m=0.0,
        )
        current = self.fly_segment(current, release_up, self.config.lift_speed, "Retreat upward after release")
        current = self.fly_segment(
            current,
            release_forward,
            self.config.retreat_speed,
            "Retreat forward after release",
        )

        landing_mode = "none" if self.config.no_land else self.config.landing_mode
        if landing_mode == "none":
            self.get_logger().warn("Skipping landing because landing mode is 'none'.")
            return

        if landing_mode == "cmd":
            self.get_logger().info("Landing will be performed by CMD_CTRL position descent and included in the dataset.")
            self.finish_with_cmd_landing(current, drone, gate_s)
            return

        elapsed_record_s = time.monotonic() - gate_s
        self.get_logger().info(
            f"Task motion finished {elapsed_record_s:.1f}s after record start; AUTO_LAND will be included in the dataset."
        )
        self.get_logger().info(
            f"Stopping position commands for {self.config.command_stop_before_land_s:.1f}s before LAND."
        )
        self.spin_sleep(self.config.command_stop_before_land_s)
        self.publish_gripper(self.config.gripper_open, repeats=5)
        self.publish_land()

    def emergency_open_and_land(self) -> None:
        self.get_logger().warn("Emergency cleanup: opening gripper.")
        self.publish_gripper(self.config.gripper_open, repeats=5)
        if not self.config.no_land:
            self.publish_land()


def parse_args() -> AutoConfig:
    parser = argparse.ArgumentParser(description="Automatic grasp/place data collection driver.")
    parser.add_argument("--drone-pose-topic", default="/mavros/vision_pose/pose")
    parser.add_argument("--target-pose-topic", default="/strawberry_bear/pose")
    parser.add_argument("--box-pose-topic", default="/box1/pose")
    parser.add_argument("--cmd-topic", default="/position_cmd")
    parser.add_argument("--gripper-topic", default="/gripper/command")
    parser.add_argument("--gripper-command-pair-topic", default="/gripper/command_pair")
    parser.add_argument("--gripper-feedback-topic", default="/gripper/feedback")
    parser.add_argument("--takeoff-land-topic", default="/px4ctrl/takeoff_land")
    parser.add_argument("--px4ctrl-state-topic", default="/px4ctrl/state")
    parser.add_argument("--record-status-topic", default="/lerobot_record/status")
    parser.add_argument("--record-gate-topic", default="/auto_grasp_dataset/record_gate")
    parser.add_argument("--record-gate-value", default="START")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--max-speed", type=float, default=0.6)
    parser.add_argument("--approach-speed", type=float, default=0.3)
    parser.add_argument("--lift-speed", type=float, default=0.4)
    parser.add_argument("--payload-lift-speed", type=float, default=0.10)
    parser.add_argument("--payload-transfer-speed", type=float, default=0.16)
    parser.add_argument("--post-grasp-settle-s", type=float, default=1.5)
    parser.add_argument("--post-lift-settle-s", type=float, default=1.5)
    parser.add_argument("--smooth-trajectory", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--takeoff-forward-comp-m",
        type=float,
        default=0.0,
        help="Body-forward correction after AUTO_TAKEOFF and before recording starts.",
    )
    parser.add_argument(
        "--payload-lift-forward-comp-m",
        type=float,
        default=0.0,
        help="Body-forward correction applied while lifting the grasped payload.",
    )
    parser.add_argument("--takeoff-comp-x", type=float, default=0.0)
    parser.add_argument("--takeoff-comp-y", type=float, default=0.0)
    parser.add_argument("--takeoff-comp-z", type=float, default=0.0)
    parser.add_argument("--payload-lift-comp-x", type=float, default=0.0)
    parser.add_argument("--payload-lift-comp-y", type=float, default=0.0)
    parser.add_argument("--payload-lift-comp-z", type=float, default=0.0)
    parser.add_argument(
        "--gripper-z-offset-m",
        type=float,
        default=0.25,
        help="Vertical distance from drone rigid-body center down to the gripper contact center.",
    )
    parser.add_argument("--target-height-m", type=float, default=0.30)
    parser.add_argument(
        "--target-grasp-height-m",
        type=float,
        default=0.17,
        help="Desired gripper contact height above the target object's base.",
    )
    parser.add_argument(
        "--target-pose-z-reference",
        choices=("base", "center", "top", "grasp"),
        default="center",
        help="Meaning of target pose z: object base, geometric center, top, or desired grasp point.",
    )
    parser.add_argument("--target-hover-clearance-m", type=float, default=0.45)
    parser.add_argument("--box-length-m", type=float, default=0.65)
    parser.add_argument("--box-width-m", type=float, default=0.41)
    parser.add_argument(
        "--box-height-m",
        type=float,
        default=0.14,
        help="Box height. Box pose z is assumed to be at the top rim plane.",
    )
    parser.add_argument(
        "--box-hover-gripper-clearance-m",
        type=float,
        default=0.55,
        help="Gripper contact-center clearance above box top rim during transfer hover.",
    )
    parser.add_argument(
        "--box-place-bottom-clearance-m",
        type=float,
        default=0.03,
        help="Clearance between target base and box bottom when releasing.",
    )
    parser.add_argument("--target-offset-x", type=float, default=0.0)
    parser.add_argument("--target-offset-y", type=float, default=0.0)
    parser.add_argument("--target-offset-z", type=float, default=0.0)
    parser.add_argument("--box-offset-x", type=float, default=0.0)
    parser.add_argument("--box-offset-y", type=float, default=0.0)
    parser.add_argument("--box-offset-z", type=float, default=0.0)
    parser.add_argument("--target-hover-z-offset", type=float, default=None)
    parser.add_argument("--target-grasp-z-offset", type=float, default=None)
    parser.add_argument("--box-hover-z-offset", type=float, default=None)
    parser.add_argument("--box-place-z-offset", type=float, default=None)
    parser.add_argument("--x-min", type=float, default=-11.0)
    parser.add_argument("--x-max", type=float, default=11.0)
    parser.add_argument("--y-min", type=float, default=-4.1)
    parser.add_argument("--y-max", type=float, default=4.1)
    parser.add_argument("--z-min", type=float, default=-0.3)
    parser.add_argument("--z-max", type=float, default=2.5)
    parser.add_argument("--pose-timeout-s", type=float, default=0.5)
    parser.add_argument("--state-timeout-s", type=float, default=10.0)
    parser.add_argument("--stable-duration-s", type=float, default=0.5)
    parser.add_argument("--stable-pos-tolerance-m", type=float, default=0.03)
    parser.add_argument("--takeoff-timeout-s", type=float, default=30.0)
    parser.add_argument("--cmd-ctrl-timeout-s", type=float, default=10.0)
    parser.add_argument("--record-ready-timeout-s", type=float, default=60.0)
    parser.add_argument("--record-duration-s", type=float, default=30.0)
    parser.add_argument("--record-start-hold-s", type=float, default=0.06)
    parser.add_argument("--grasp-mode", choices=("soft", "continuous_center"), default="continuous_center")
    parser.add_argument("--gripper-open", type=float, default=100.0)
    parser.add_argument("--gripper-closed", type=float, default=0.0)
    parser.add_argument("--gripper-close-duration-s", type=float, default=1.5)
    parser.add_argument("--gripper-open-duration-s", type=float, default=0.4)
    parser.add_argument("--grasp-step-size", type=float, default=3.0)
    parser.add_argument("--grasp-step-settle-s", type=float, default=0.10)
    parser.add_argument("--grasp-open-timeout-s", type=float, default=2.0)
    parser.add_argument("--grasp-open-tolerance", type=float, default=5.0)
    parser.add_argument("--grasp-step-timeout-s", type=float, default=0.80)
    parser.add_argument("--grasp-goal-tolerance", type=float, default=3.0)
    parser.add_argument("--grasp-close-min", type=float, default=15.0)
    parser.add_argument("--grasp-contact-current-delta", type=float, default=250.0)
    parser.add_argument("--grasp-contact-load-delta", type=float, default=800.0)
    parser.add_argument("--grasp-position-error-threshold", type=float, default=20.0)
    parser.add_argument("--grasp-angle-contact-delta", type=float, default=20.0)
    parser.add_argument("--grasp-stall-delta", type=float, default=0.25)
    parser.add_argument("--grasp-contact-min-close-delta", type=float, default=15.0)
    parser.add_argument("--grasp-contact-confirm-steps", type=int, default=2)
    parser.add_argument("--grasp-balance-load-diff", type=float, default=60.0)
    parser.add_argument("--grasp-balance-step", type=float, default=1.5)
    parser.add_argument("--grasp-max-balance-steps", type=int, default=8)
    parser.add_argument("--grasp-angle-balance-diff", type=float, default=5.0)
    parser.add_argument("--grasp-compliance-radius-m", type=float, default=0.35)
    parser.add_argument("--grasp-compliance-elastic-gain", type=float, default=0.15)
    parser.add_argument("--grasp-abort-drift-m", type=float, default=0.80)
    parser.add_argument("--grasp-attitude-soft-rad", type=float, default=0.30)
    parser.add_argument("--grasp-abort-attitude-rad", type=float, default=0.65)
    parser.add_argument(
        "--release-retreat-up-m",
        type=float,
        default=0.3,
        help="After release, climb this much before moving forward.",
    )
    parser.add_argument(
        "--release-retreat-forward-m",
        type=float,
        default=2.0,
        help="After release and climb, fly this far forward in current yaw direction before landing.",
    )
    parser.add_argument("--retreat-speed", type=float, default=0.6)
    parser.add_argument(
        "--landing-mode",
        choices=("cmd", "auto", "none"),
        default="cmd",
        help="Normal end-of-task landing mode. 'cmd' descends with /position_cmd; 'auto' publishes px4ctrl LAND.",
    )
    parser.add_argument("--cmd-land-speed", type=float, default=0.25)
    parser.add_argument(
        "--cmd-land-z",
        type=float,
        default=-0.3,
        help="Absolute drone-center z for CMD_CTRL landing.",
    )
    parser.add_argument("--cmd-land-z-offset-m", type=float, default=0.0)
    parser.add_argument("--command-stop-before-land-s", type=float, default=1.2)
    parser.add_argument("--no-land", action="store_true")

    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    if not 0.5 <= args.max_speed <= 1.0:
        raise ValueError("--max-speed must be in [0.5, 1.0] m/s.")
    if args.approach_speed <= 0.0 or args.lift_speed <= 0.0:
        raise ValueError("--approach-speed and --lift-speed must be positive.")
    if args.payload_lift_speed <= 0.0 or args.payload_transfer_speed <= 0.0:
        raise ValueError("--payload-lift-speed and --payload-transfer-speed must be positive.")
    if args.post_grasp_settle_s < 0.0 or args.post_lift_settle_s < 0.0:
        raise ValueError("--post-grasp-settle-s and --post-lift-settle-s must be non-negative.")
    if args.retreat_speed <= 0.0:
        raise ValueError("--retreat-speed must be positive.")
    if args.cmd_land_speed <= 0.0:
        raise ValueError("--cmd-land-speed must be positive.")
    if args.rate_hz <= 0.0:
        raise ValueError("--rate-hz must be positive.")
    if args.record_duration_s <= 0.0:
        raise ValueError("--record-duration-s must be positive.")
    if args.gripper_z_offset_m < 0.0:
        raise ValueError("--gripper-z-offset-m must be non-negative.")
    if args.target_height_m <= 0.0:
        raise ValueError("--target-height-m must be positive.")
    if not 0.0 < args.target_grasp_height_m <= args.target_height_m:
        raise ValueError("--target-grasp-height-m must be in (0, --target-height-m].")
    if args.box_length_m <= 0.0 or args.box_width_m <= 0.0 or args.box_height_m <= 0.0:
        raise ValueError("--box-length-m, --box-width-m, and --box-height-m must be positive.")
    if args.release_retreat_up_m < 0.0 or args.release_retreat_forward_m < 0.0:
        raise ValueError("--release-retreat-up-m and --release-retreat-forward-m must be non-negative.")
    if not 0.0 <= args.gripper_closed <= args.gripper_open:
        raise ValueError("--gripper-closed must be within [0, --gripper-open].")
    if args.gripper_close_duration_s <= 0.0 or args.gripper_open_duration_s <= 0.0:
        raise ValueError("--gripper-close-duration-s and --gripper-open-duration-s must be positive.")
    if (
        args.grasp_step_size <= 0.0
        or args.grasp_step_settle_s <= 0.0
        or args.grasp_step_timeout_s <= 0.0
        or args.grasp_open_timeout_s <= 0.0
    ):
        raise ValueError(
            "--grasp-step-size, --grasp-step-settle-s, --grasp-step-timeout-s, "
            "and --grasp-open-timeout-s must be positive."
        )
    if args.grasp_goal_tolerance < 0.0 or args.grasp_open_tolerance < 0.0:
        raise ValueError("--grasp-goal-tolerance and --grasp-open-tolerance must be non-negative.")
    if not 0.0 <= args.grasp_close_min <= args.gripper_open:
        raise ValueError("--grasp-close-min must be within [0, --gripper-open].")
    if args.grasp_angle_contact_delta < 0.0 or args.grasp_stall_delta < 0.0:
        raise ValueError("--grasp-angle-contact-delta and --grasp-stall-delta must be non-negative.")
    if args.grasp_contact_min_close_delta < 0.0:
        raise ValueError("--grasp-contact-min-close-delta must be non-negative.")
    if args.grasp_contact_confirm_steps < 1:
        raise ValueError("--grasp-contact-confirm-steps must be >= 1.")
    if args.grasp_max_balance_steps < 0:
        raise ValueError("--grasp-max-balance-steps must be >= 0.")
    if args.grasp_angle_balance_diff < 0.0:
        raise ValueError("--grasp-angle-balance-diff must be non-negative.")
    if (
        args.grasp_compliance_radius_m < 0.0
        or not 0.0 <= args.grasp_compliance_elastic_gain <= 1.0
        or args.grasp_abort_drift_m <= 0.0
        or args.grasp_attitude_soft_rad < 0.0
        or args.grasp_abort_attitude_rad <= 0.0
        or args.grasp_attitude_soft_rad >= args.grasp_abort_attitude_rad
    ):
        raise ValueError(
            "--grasp-compliance-radius-m must be non-negative, "
            "--grasp-compliance-elastic-gain must be in [0, 1], "
            "and attitude/drift thresholds must be positive and ordered."
        )

    return AutoConfig(**vars(args))


def main() -> None:
    config = parse_args()
    rclpy.init()
    node = AutoGraspPlaceDataset(config)
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        node.emergency_open_and_land()
        raise
    except Exception as exc:
        node.get_logger().error(f"Automatic sequence failed: {exc}")
        node.emergency_open_and_land()
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
