#!/usr/bin/python3

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, replace

from mavros_msgs.msg import RCIn
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool, Float64, String

from auto_grasp_place_dataset import AutoConfig, AutoGraspPlaceDataset, PoseSample, parse_args as parse_auto_args


HLS_SINGLE_CONTACT_STATES = {"LEFT_CONTACT", "RIGHT_CONTACT"}
HLS_BODY_Y_CENTERING_STATES = {"BOTH_CONTACT", "CENTERING", "CENTERED", "FINAL_GRIP"}
HLS_OFFSET_LIMIT_GRACE_S = 0.8


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def sign_nonzero(value: float) -> float:
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


@dataclass(frozen=True)
class HlsTaskConfig:
    hls_status_topic: str
    hls_status_timeout_s: float
    hls_grasp_timeout_s: float
    rc_topic: str
    rc_timeout_s: float
    rc_stale_action: str
    ch10_index: int
    ch10_open_pwm: int
    ch10_close_pwm: int
    force_open_below_z: float
    open_command: float
    close_command: float
    center_deadband_m: float
    center_kp: float
    center_vmax_mps: float
    center_offset_max_m: float
    center_command_sign: float
    single_contact_vmax_mps: float
    single_contact_offset_max_m: float
    single_contact_body_y_sign: float
    abort_rise_m: float
    abort_rise_speed: float


@dataclass
class StandardHlsStatus:
    state: str = "UNKNOWN"
    center_error_m: float = 0.0
    centering_offset_m: float = 0.0
    single_contact_offset_m: float = 0.0
    safe_to_lift: bool = False
    fault: bool = False
    fault_reason: str = ""
    left_contact: bool = False
    right_contact: bool = False
    single_contact_need_motion: bool = False
    single_contact_direction: float = 0.0
    left_at_close_limit: bool = False
    right_at_close_limit: bool = False


class AutoHlsSafetyAbort(RuntimeError):
    pass


def status_prefix_from_topic(topic: str) -> str:
    topic = topic.rstrip("/")
    if topic.endswith("/status"):
        return topic[: -len("/status")]
    return topic


class AutoHlsGraspPlace(AutoGraspPlaceDataset):
    def __init__(self, config: AutoConfig, hls_config: HlsTaskConfig) -> None:
        super().__init__(config)
        self.hls_config = hls_config
        self.status_prefix = status_prefix_from_topic(hls_config.hls_status_topic)
        self.hls_status = StandardHlsStatus()
        self.hls_status_stamps: dict[str, float] = {}
        self.rc: RCIn | None = None
        self.rc_received_s: float | None = None
        self.last_ch10_pwm: int | None = None
        self.mission_safety_armed = False
        self.safety_abort_active = False
        self.hls_close_allowed = False
        self.hls_payload_attached = False
        self.last_pregrasp_open_s = 0.0
        self.last_safety_open_s = 0.0
        self.last_rc_stale_warn_s = 0.0
        self.create_subscription(String, f"{self.status_prefix}/state", self._string_cb("state"), 10)
        self.create_subscription(String, f"{self.status_prefix}/fault_reason", self._string_cb("fault_reason"), 10)
        self.create_subscription(Float64, f"{self.status_prefix}/center_error_m", self._float_cb("center_error_m"), 10)
        self.create_subscription(Float64, f"{self.status_prefix}/centering_offset_m", self._float_cb("centering_offset_m"), 10)
        self.create_subscription(
            Float64,
            f"{self.status_prefix}/single_contact_offset_m",
            self._float_cb("single_contact_offset_m"),
            10,
        )
        self.create_subscription(
            Float64,
            f"{self.status_prefix}/single_contact_direction",
            self._float_cb("single_contact_direction"),
            10,
        )
        for name in (
            "safe_to_lift",
            "fault",
            "left_contact",
            "right_contact",
            "single_contact_need_motion",
            "left_at_close_limit",
            "right_at_close_limit",
        ):
            self.create_subscription(Bool, f"{self.status_prefix}/{name}", self._bool_cb(name), 10)
        rc_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(RCIn, hls_config.rc_topic, self._rc_cb, rc_qos)
        self.get_logger().info(
            "Auto HLS grasp/place started without dataset recording. "
            f"status_prefix={self.status_prefix} rc_topic={hls_config.rc_topic} "
            f"ch10_index={hls_config.ch10_index}"
        )

    def _string_cb(self, name: str):
        def callback(msg: String) -> None:
            setattr(self.hls_status, name, str(msg.data))
            self.hls_status_stamps[name] = time.monotonic()

        return callback

    def _float_cb(self, name: str):
        def callback(msg: Float64) -> None:
            setattr(self.hls_status, name, float(msg.data))
            self.hls_status_stamps[name] = time.monotonic()

        return callback

    def _bool_cb(self, name: str):
        def callback(msg: Bool) -> None:
            setattr(self.hls_status, name, bool(msg.data))
            self.hls_status_stamps[name] = time.monotonic()

        return callback

    def _rc_cb(self, msg: RCIn) -> None:
        self.rc = msg
        self.rc_received_s = time.monotonic()
        if 0 <= self.hls_config.ch10_index < len(msg.channels):
            self.last_ch10_pwm = int(msg.channels[self.hls_config.ch10_index])
        else:
            self.last_ch10_pwm = None

    def wait_for_record_ready(self) -> None:
        return

    def publish_record_gate(self) -> None:
        return

    def hls_status_fresh(self) -> bool:
        now = time.monotonic()
        for name in (
            "state",
            "centering_offset_m",
            "single_contact_offset_m",
            "safe_to_lift",
            "fault",
            "left_contact",
            "right_contact",
            "single_contact_need_motion",
            "single_contact_direction",
        ):
            stamp = self.hls_status_stamps.get(name)
            if stamp is None or now - stamp > self.hls_config.hls_status_timeout_s:
                return False
        return True

    def latest_hls_status_or_raise(self) -> StandardHlsStatus:
        if self.hls_status_fresh():
            return self.hls_status
        raise RuntimeError(f"No fresh HLS gripper standard status under {self.status_prefix}.")

    def wait_for_hls_status(self) -> StandardHlsStatus:
        deadline = time.monotonic() + max(0.5, self.config.state_timeout_s)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.hls_status_fresh():
                return self.hls_status
        raise RuntimeError(f"Timed out waiting for HLS gripper standard status under {self.status_prefix}.")

    def rc_fresh(self) -> bool:
        return (
            self.rc is not None
            and self.rc_received_s is not None
            and time.monotonic() - self.rc_received_s <= self.hls_config.rc_timeout_s
        )

    def ch10_open_requested(self) -> bool:
        return self.last_ch10_pwm is None or self.last_ch10_pwm <= self.hls_config.ch10_open_pwm

    def wait_for_rc_safety_ready(self) -> None:
        deadline = time.monotonic() + max(1.0, self.config.state_timeout_s)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.rc_fresh():
                if self.ch10_open_requested():
                    raise AutoHlsSafetyAbort(
                        f"CH10 is low/open before automatic takeoff; pwm={self.last_ch10_pwm}."
                    )
                self.get_logger().info(
                    f"CH10 safety ready: pwm={self.last_ch10_pwm}, "
                    f"open<={self.hls_config.ch10_open_pwm}, close>={self.hls_config.ch10_close_pwm}."
                )
                return
        raise AutoHlsSafetyAbort(f"No fresh RC input on {self.hls_config.rc_topic} before automatic takeoff.")

    def force_open_for_safety(self, reason: str) -> None:
        now = time.monotonic()
        self.hls_close_allowed = False
        self.hls_payload_attached = False
        if now - self.last_safety_open_s >= 0.2:
            self.get_logger().warn(f"HLS safety open: {reason}")
            self.publish_gripper(self.hls_config.open_command, repeats=3, interval_s=0.0)
            self.last_safety_open_s = now

    def raise_if_safety_abort(self, phase: str) -> None:
        if self.safety_abort_active or not self.mission_safety_armed:
            return
        if not self.rc_fresh():
            reason = f"RC stale during {phase}"
            if self.hls_config.rc_stale_action == "abort":
                self.force_open_for_safety(reason)
                raise AutoHlsSafetyAbort(reason)
            now = time.monotonic()
            if now - self.last_rc_stale_warn_s >= 1.0:
                self.get_logger().warn(
                    f"{reason}; continuing with last CH10 pwm={self.last_ch10_pwm}. "
                    "Set --rc-stale-action abort to make RC timeout abort the mission."
                )
                self.last_rc_stale_warn_s = now
        elif self.ch10_open_requested():
            reason = f"CH10 low/open during {phase}; pwm={self.last_ch10_pwm}"
            self.force_open_for_safety(reason)
            raise AutoHlsSafetyAbort(reason)
        if self.hls_close_allowed or self.hls_payload_attached:
            if self.px4ctrl_state != "CMD_CTRL":
                reason = f"px4ctrl left CMD_CTRL during {phase}; latest={self.px4ctrl_state!r}"
                self.force_open_for_safety(reason)
                raise AutoHlsSafetyAbort(reason)
            drone = self.poses.get("drone")
            if (
                drone is not None
                and self.pose_fresh("drone")
                and drone.z <= self.hls_config.force_open_below_z
            ):
                reason = (
                    f"drone below gripper safety height during {phase}: "
                    f"z={drone.z:.3f} <= {self.hls_config.force_open_below_z:.3f}"
                )
                self.force_open_for_safety(reason)
                raise AutoHlsSafetyAbort(reason)

    def keep_open_before_close_if_needed(self) -> None:
        if self.safety_abort_active or self.hls_close_allowed or self.hls_payload_attached:
            return
        now = time.monotonic()
        if now - self.last_pregrasp_open_s >= 0.2:
            self.publish_gripper(self.hls_config.open_command, repeats=1, interval_s=0.0)
            self.last_pregrasp_open_s = now

    def publish_cmd(self, pose: PoseSample) -> None:
        self.raise_if_safety_abort("position command")
        super().publish_cmd(pose)
        self.keep_open_before_close_if_needed()

    def publish_gripper_ramp(
        self,
        start: float,
        end: float,
        duration_s: float,
        hold_pose: PoseSample | None = None,
    ) -> None:
        if end >= self.hls_config.open_command - 1e-6:
            self.hls_close_allowed = False
            self.hls_payload_attached = False
        super().publish_gripper_ramp(start, end, duration_s, hold_pose)

    def run_sequence(self) -> None:
        self.wait_for_rc_safety_ready()
        self.mission_safety_armed = True
        try:
            super().run_sequence()
        finally:
            self.hls_close_allowed = False
            self.hls_payload_attached = False

    def pose_with_body_y_offset(self, reference: PoseSample, body_y_offset_m: float) -> PoseSample:
        return self.checked_pose(
            x=reference.x - math.sin(reference.yaw) * body_y_offset_m,
            y=reference.y + math.cos(reference.yaw) * body_y_offset_m,
            z=reference.z,
            yaw=reference.yaw,
        )

    def abort_before_lift(self, current: PoseSample, reference: PoseSample, reason: str) -> PoseSample:
        self.get_logger().error(f"HLS grasp failed before lift: {reason}")
        self.hls_close_allowed = False
        self.hls_payload_attached = False
        self.publish_gripper(self.hls_config.open_command, repeats=5)
        abort_z = min(self.config.z_max, max(current.z, reference.z + self.hls_config.abort_rise_m))
        abort_pose = self.checked_pose(current.x, current.y, abort_z, current.yaw)
        self.get_logger().warn(
            f"Opening gripper and rising away from target: z={abort_pose.z:.3f} speed={self.hls_config.abort_rise_speed:.3f}."
        )
        return self.fly_segment(current, abort_pose, self.hls_config.abort_rise_speed, "Abort rise after failed HLS grasp")

    def body_y_assist_command(
        self,
        status: StandardHlsStatus,
    ) -> tuple[str, bool, float, float, float, float]:
        """Return phase, active flag, raw target, clamped target, limit, and vmax."""
        cfg = self.hls_config
        if status.state in HLS_SINGLE_CONTACT_STATES and status.left_contact != status.right_contact:
            offset_limit_m = cfg.single_contact_offset_max_m
            if not status.single_contact_need_motion:
                return "single_hold", False, 0.0, 0.0, offset_limit_m, cfg.single_contact_vmax_mps

            direction = sign_nonzero(status.single_contact_direction)
            raw_magnitude_m = abs(float(status.single_contact_offset_m))
            if direction == 0.0 or raw_magnitude_m <= cfg.center_deadband_m:
                return "single_hold", False, 0.0, 0.0, offset_limit_m, cfg.single_contact_vmax_mps

            raw_target_m = cfg.single_contact_body_y_sign * direction * raw_magnitude_m
            target_m = clamp(raw_target_m, -offset_limit_m, offset_limit_m)
            return "single", True, raw_target_m, target_m, offset_limit_m, cfg.single_contact_vmax_mps

        if status.state in HLS_BODY_Y_CENTERING_STATES:
            offset_limit_m = cfg.center_offset_max_m
            raw_target_m = cfg.center_command_sign * float(status.centering_offset_m)
            target_m = clamp(raw_target_m, -offset_limit_m, offset_limit_m)
            return "center", True, raw_target_m, target_m, offset_limit_m, cfg.center_vmax_mps

        return "hold", False, 0.0, 0.0, cfg.center_offset_max_m, cfg.center_vmax_mps

    def soft_grasp(self, reference: PoseSample) -> PoseSample:
        self.wait_for_hls_status()
        self.hold_cmd(reference, 0.3)
        self.raise_if_safety_abort("pre-HLS grasp")
        self.hls_close_allowed = True
        self.hls_payload_attached = False
        self.publish_gripper(self.hls_config.close_command, repeats=5)

        period = 1.0 / self.config.rate_hz
        deadline = time.monotonic() + self.hls_config.hls_grasp_timeout_s
        latest = reference
        body_y_offset_m = 0.0
        last_t = time.monotonic()
        last_log_s = 0.0
        offset_limit_hit_since_s: float | None = None

        self.get_logger().info(
            "HLS force grasp requested. Holding during search, using staged body-y assist for single-contact "
            "motion requests and slower centering corrections while continuously commanding close."
        )

        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            dt = max(1e-3, now - last_t)
            last_t = now
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.px4ctrl_state != "CMD_CTRL":
                raise RuntimeError(f"px4ctrl left CMD_CTRL during HLS grasp; latest={self.px4ctrl_state!r}.")
            try:
                self.raise_if_safety_abort("HLS grasp")
            except AutoHlsSafetyAbort as exc:
                latest = self.abort_before_lift(latest, reference, str(exc))
                raise

            try:
                status = self.latest_hls_status_or_raise()
            except RuntimeError as exc:
                latest = self.abort_before_lift(latest, reference, str(exc))
                raise

            if status.fault:
                latest = self.abort_before_lift(latest, reference, status.fault_reason or "HLS fault")
                raise RuntimeError(f"HLS gripper fault: {status.fault_reason}")

            if status.safe_to_lift or status.state == "LIFT_READY":
                self.get_logger().info("HLS reports safe_to_lift; automatic lift may start.")
                self.hls_payload_attached = True
                return latest

            phase, assist_active, raw_target_offset_m, target_offset_m, offset_limit_m, vmax_mps = (
                self.body_y_assist_command(status)
            )

            if assist_active:
                offset_error_m = target_offset_m - body_y_offset_m
                if abs(offset_error_m) > self.hls_config.center_deadband_m:
                    vy = self.hls_config.center_kp * offset_error_m
                    vy = clamp(vy, -vmax_mps, vmax_mps)
                    step = vy * dt
                    if abs(step) > abs(offset_error_m):
                        step = offset_error_m
                    body_y_offset_m += step
                    body_y_offset_m = clamp(body_y_offset_m, -offset_limit_m, offset_limit_m)
            else:
                offset_limit_hit_since_s = None

            limit_hit = (
                assist_active
                and (status.left_contact or status.right_contact)
                and abs(body_y_offset_m) >= max(0.0, offset_limit_m - self.hls_config.center_deadband_m)
                and abs(raw_target_offset_m) >= max(0.0, offset_limit_m - self.hls_config.center_deadband_m)
            )
            if limit_hit:
                if offset_limit_hit_since_s is None:
                    offset_limit_hit_since_s = now
                    self.get_logger().warn(
                        f"HLS body-y assist reached {phase} offset limit; holding close command for "
                        f"{HLS_OFFSET_LIMIT_GRACE_S:.1f}s before abort if not safe_to_lift."
                    )
                elif now - offset_limit_hit_since_s >= HLS_OFFSET_LIMIT_GRACE_S:
                    latest = self.pose_with_body_y_offset(reference, body_y_offset_m)
                    latest = self.abort_before_lift(latest, reference, "HLS body-y assist offset limit")
                    raise RuntimeError("HLS body-y assist stayed at offset limit before safe_to_lift.")
            elif assist_active:
                offset_limit_hit_since_s = None

            latest = self.pose_with_body_y_offset(reference, body_y_offset_m)

            self.publish_cmd(latest)
            self.publish_gripper(self.hls_config.close_command, repeats=1, interval_s=0.0)

            if now - last_log_s >= 0.5:
                last_log_s = now
                self.get_logger().info(
                    f"HLS state={status.state} phase={phase} center={status.center_error_m:+.4f}m "
                    f"target_offset={target_offset_m:+.4f}m raw={raw_target_offset_m:+.4f}m "
                    f"body_y_offset={body_y_offset_m:+.4f}m vmax={vmax_mps:.3f} "
                    f"single_est={status.single_contact_offset_m:+.4f}m "
                    f"motion={int(status.single_contact_need_motion)} dir={status.single_contact_direction:+.0f} "
                    f"contact=({int(status.left_contact)},{int(status.right_contact)})"
                )
            time.sleep(period)

        latest = self.abort_before_lift(latest, reference, "HLS grasp timeout")
        raise RuntimeError("HLS grasp timed out before safe_to_lift.")

    def emergency_open_and_land(self) -> None:
        self.safety_abort_active = True
        self.hls_close_allowed = False
        self.hls_payload_attached = False
        self.get_logger().warn("Emergency cleanup: opening HLS gripper.")
        self.publish_gripper(self.hls_config.open_command, repeats=5)
        if self.config.no_land:
            return
        if self.px4ctrl_state != "CMD_CTRL" or not self.pose_fresh("drone"):
            self.get_logger().warn(
                "Emergency cleanup will not command-land because CMD_CTRL or fresh drone pose is unavailable. "
                f"px4ctrl_state={self.px4ctrl_state!r} pose_fresh={self.pose_fresh('drone')}."
            )
            return
        drone = self.poses["drone"]
        current = self.checked_pose(drone.x, drone.y, drone.z, drone.yaw)
        landing_z = (
            self.config.cmd_land_z
            if self.config.cmd_land_z is not None
            else drone.z + self.config.cmd_land_z_offset_m
        )
        landing_pose = self.checked_pose(current.x, current.y, landing_z, current.yaw)
        self.get_logger().warn(
            f"Emergency CMD_CTRL descent with gripper open: z={landing_pose.z:.3f} "
            f"speed={self.config.cmd_land_speed:.3f}."
        )
        self.fly_segment(
            current,
            landing_pose,
            self.config.cmd_land_speed,
            "Emergency CMD_CTRL descent landing",
            keep_gripper_open=True,
        )


def parse_configs() -> tuple[AutoConfig, HlsTaskConfig]:
    raw_args = remove_ros_args(args=sys.argv)[1:]
    hls_parser = argparse.ArgumentParser(add_help=False)
    hls_parser.add_argument("--hls-status-topic", default="/hls_gripper/status")
    hls_parser.add_argument("--hls-status-timeout-s", type=float, default=0.8)
    hls_parser.add_argument("--hls-grasp-timeout-s", type=float, default=12.0)
    hls_parser.add_argument("--rc-topic", default="/mavros/rc/in")
    hls_parser.add_argument("--rc-timeout-s", type=float, default=0.5)
    hls_parser.add_argument("--rc-stale-action", choices=("warn", "abort"), default="warn")
    hls_parser.add_argument("--ch10-index", type=int, default=9)
    hls_parser.add_argument("--ch10-open-pwm", type=int, default=1300)
    hls_parser.add_argument("--ch10-close-pwm", type=int, default=1700)
    hls_parser.add_argument("--force-open-below-z", type=float, default=0.20)
    hls_parser.add_argument("--open-command", type=float, default=100.0)
    hls_parser.add_argument("--close-command", type=float, default=0.0)
    hls_parser.add_argument("--center-deadband-m", type=float, default=0.005)
    hls_parser.add_argument("--center-kp", type=float, default=0.8)
    hls_parser.add_argument("--center-vmax-mps", type=float, default=0.03)
    hls_parser.add_argument("--center-offset-max-m", type=float, default=0.08)
    hls_parser.add_argument("--center-command-sign", type=float, default=1.0)
    hls_parser.add_argument("--single-contact-vmax-mps", type=float, default=0.015)
    hls_parser.add_argument("--single-contact-offset-max-m", type=float, default=0.10)
    hls_parser.add_argument("--single-contact-body-y-sign", type=float, default=1.0)
    hls_parser.add_argument("--abort-rise-m", type=float, default=0.25)
    hls_parser.add_argument("--abort-rise-speed", type=float, default=0.12)
    hls_args, remaining = hls_parser.parse_known_args(raw_args)

    old_argv = sys.argv
    try:
        sys.argv = [old_argv[0], *remaining]
        auto_config = parse_auto_args()
    finally:
        sys.argv = old_argv

    auto_config = replace(
        auto_config,
        record_status_topic="",
        record_gate_topic="/auto_hls_grasp_place/unused_record_gate",
        record_duration_s=0.1,
        record_start_hold_s=0.0,
    )
    hls_config = HlsTaskConfig(**vars(hls_args))
    if hls_config.hls_status_timeout_s <= 0.0:
        raise ValueError("--hls-status-timeout-s must be positive.")
    if hls_config.hls_grasp_timeout_s <= 0.0:
        raise ValueError("--hls-grasp-timeout-s must be positive.")
    if hls_config.rc_timeout_s <= 0.0:
        raise ValueError("--rc-timeout-s must be positive.")
    if hls_config.ch10_index < 0:
        raise ValueError("--ch10-index must be non-negative.")
    if hls_config.ch10_open_pwm >= hls_config.ch10_close_pwm:
        raise ValueError("--ch10-open-pwm must be less than --ch10-close-pwm.")
    if hls_config.force_open_below_z < auto_config.z_min or hls_config.force_open_below_z > auto_config.z_max:
        raise ValueError("--force-open-below-z must be inside configured flight z limits.")
    if hls_config.center_kp < 0.0 or hls_config.center_vmax_mps <= 0.0 or hls_config.center_offset_max_m <= 0.0:
        raise ValueError("--center-kp must be non-negative; center speed/offset limits must be positive.")
    if hls_config.single_contact_vmax_mps <= 0.0 or hls_config.single_contact_offset_max_m <= 0.0:
        raise ValueError("--single-contact-vmax-mps and --single-contact-offset-max-m must be positive.")
    if hls_config.single_contact_body_y_sign == 0.0:
        raise ValueError("--single-contact-body-y-sign must be non-zero.")
    if hls_config.abort_rise_m < 0.0 or hls_config.abort_rise_speed <= 0.0:
        raise ValueError("--abort-rise-m must be non-negative and --abort-rise-speed must be positive.")
    return auto_config, hls_config


def main() -> None:
    auto_config, hls_config = parse_configs()
    rclpy.init()
    node = AutoHlsGraspPlace(auto_config, hls_config)
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        node.emergency_open_and_land()
        raise
    except Exception as exc:
        node.get_logger().error(f"Automatic HLS sequence failed: {exc}")
        node.emergency_open_and_land()
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
