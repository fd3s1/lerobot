#!/usr/bin/python3

from __future__ import annotations

import argparse
import copy
from datetime import datetime
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import TakeoffLand
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu
from std_msgs.msg import String


MAYBE_AIRBORNE_EXIT_CODE = 20

DEFAULT_BAG_TOPICS = (
    "/mavros/tunnel/in",
    "/mavros/setpoint_raw/attitude",
    "/mavros/setpoint_raw/target_attitude",
    "/mavros/local_position/odom",
    "/mavros/imu/data",
    "/mavros/vision_pose/pose",
    "/vla_drone1/twist",
    "/mavros/battery",
    "/mavros/state",
    "/mavros/extended_state",
    "/mavros/rc/in",
    "/position_cmd",
    "/px4ctrl/takeoff_land",
    "/px4ctrl/state",
    "/px4ctrl/simulink/attitude_target",
    "/px4ctrl/simulink/reference_state",
    "/px4ctrl/simulink/actual_state",
    "/px4ctrl/simulink/tracking_error",
    "/px4ctrl/simulink/ude_debug",
    "/px4ctrl/simulink/yaw_debug",
    "/px4ctrl/ude_tune_status",
    "/px4ctrl/ude_tune_status_text",
    "/px4ctrl/mocap_state_status",
)


def mavros_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def latched_state_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return normalize_angle(math.atan2(siny_cosp, cosy_cosp))


class TakeoffHoverTest(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(getattr(args, "node_name", "ude_takeoff_hover_test"))
        self.args = args

        self.last_seen: dict[str, float] = {}
        self.fsm_state: str | None = None
        self.mavros_state: State | None = None
        self.odom: Odometry | None = None
        self.imu: Imu | None = None
        self.vision_pose: PoseStamped | None = None
        self.mocap_twist: TwistStamped | None = None
        self.mocap_state_status: str | None = None
        self.attitude_sp: AttitudeTarget | None = None
        self.takeoff_sent = False
        self.rng = random.Random(args.wp_random_seed)
        self.bag_process: subprocess.Popen[str] | None = None
        self.bag_output: Path | None = None
        self.bag_log_handle = None
        self.bag_log_path: Path | None = None

        self.takeoff_pub = self.create_publisher(TakeoffLand, args.takeoff_land_topic, 10)
        self.cmd_pub = self.create_publisher(PoseStamped, args.cmd_topic, 10)

        self.create_subscription(String, args.state_topic, self._fsm_state_cb, latched_state_qos())
        self.create_subscription(State, args.mavros_state_topic, self._mavros_state_cb, 10)
        self.create_subscription(Odometry, args.odom_topic, self._odom_cb, mavros_qos())
        self.create_subscription(Imu, args.imu_topic, self._imu_cb, mavros_qos())
        self.create_subscription(PoseStamped, args.vision_topic, self._vision_pose_cb, mavros_qos())
        self.create_subscription(TwistStamped, args.mocap_twist_topic, self._mocap_twist_cb, mavros_qos())
        self.create_subscription(
            String,
            args.mocap_state_status_topic,
            self._mocap_state_status_cb,
            10,
        )
        self.create_subscription(
            AttitudeTarget,
            args.attitude_setpoint_topic,
            self._attitude_setpoint_cb,
            mavros_qos(),
        )

    def _touch(self, name: str) -> None:
        self.last_seen[name] = time.monotonic()

    def _fsm_state_cb(self, msg: String) -> None:
        self.fsm_state = msg.data
        self._touch("px4ctrl_state")

    def _mavros_state_cb(self, msg: State) -> None:
        self.mavros_state = msg
        self._touch("mavros_state")

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg
        self._touch("odom")

    def _imu_cb(self, msg: Imu) -> None:
        self.imu = msg
        self._touch("imu")

    def _vision_pose_cb(self, msg: PoseStamped) -> None:
        self.vision_pose = msg
        self._touch("vision_pose")

    def _mocap_twist_cb(self, msg: TwistStamped) -> None:
        self.mocap_twist = msg
        self._touch("mocap_twist")

    def _mocap_state_status_cb(self, msg: String) -> None:
        self.mocap_state_status = msg.data
        self._touch("mocap_state_status")

    def _attitude_setpoint_cb(self, msg: AttitudeTarget) -> None:
        self.attitude_sp = msg
        self._touch("attitude_setpoint")

    def age(self, name: str) -> float | None:
        stamp = self.last_seen.get(name)
        if stamp is None:
            return None
        return time.monotonic() - stamp

    def is_fresh(self, name: str, max_age: float) -> bool:
        age = self.age(name)
        return age is not None and age <= max_age

    def required_inputs_ready(self) -> tuple[bool, list[str]]:
        required = [
            "px4ctrl_state",
            "mavros_state",
            "odom",
            "imu",
        ]
        if self.args.require_vision_pose:
            required.append("vision_pose")
        if self.args.require_composed_state:
            required.extend(("vision_pose", "mocap_twist", "mocap_state_status"))
        if not self.args.skip_setpoint_check:
            required.append("attitude_setpoint")

        missing = [
            name
            for name in required
            if not self.is_fresh(name, self.args.fresh_timeout)
        ]

        if self.args.require_composed_state and self.is_fresh(
            "mocap_state_status", self.args.fresh_timeout
        ):
            status = self.mocap_state_status or ""
            if not (
                "p_source=mocap" in status
                and "v_source=mocap" in status
                and "reason='mocap_synced'" in status
            ):
                missing.append("mocap_state_sync")

        if (
            not self.args.skip_mavros_connected_check
            and self.mavros_state is not None
            and not self.mavros_state.connected
        ):
            missing.append("mavros_connected")

        return len(missing) == 0, missing

    def wait_for_startup(self) -> bool:
        deadline = time.monotonic() + self.args.startup_timeout
        last_report = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            ready, missing = self.required_inputs_ready()
            if ready:
                return True

            now = time.monotonic()
            if now - last_report >= 1.0:
                self.get_logger().info(
                    "Waiting for startup inputs: " + ", ".join(missing)
                )
                last_report = now
            time.sleep(0.05)

        ready, missing = self.required_inputs_ready()
        if not ready:
            self.get_logger().error(
                "Startup inputs are not ready: " + ", ".join(missing)
            )
        return ready

    def set_px4ctrl_param_if_requested(
        self,
        *,
        param_name: str,
        value: str | None,
        timeout_s: float,
    ) -> bool:
        if value is None:
            return True

        deadline = time.monotonic() + max(0.1, timeout_s)
        last_error = ""
        self.get_logger().info(f"Setting /px4ctrl {param_name}={value} before TAKEOFF.")

        while rclpy.ok() and time.monotonic() < deadline:
            try:
                result = subprocess.run(
                    ["ros2", "param", "set", "/px4ctrl", param_name, value],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=2.0,
                    check=False,
                )
            except subprocess.TimeoutExpired:
                last_error = "ros2 param set timed out"
            except OSError as exc:
                last_error = str(exc)
            else:
                if result.returncode == 0:
                    output = result.stdout.strip()
                    if output:
                        self.get_logger().info(output)
                    self.get_logger().info(f"/px4ctrl {param_name} is now {value}.")
                    return True
                last_error = (result.stderr or result.stdout).strip()

            time.sleep(0.5)

        self.get_logger().error(
            f"Failed to set /px4ctrl {param_name}={value} within "
            f"{timeout_s:.1f}s: {last_error}"
        )
        return False

    def set_ude_enable_if_requested(self) -> bool:
        return self.set_px4ctrl_param_if_requested(
            param_name="ude.enable",
            value=self.args.ude_enable,
            timeout_s=self.args.ude_param_timeout_s,
        )

    def set_td_enable_if_requested(self) -> bool:
        return self.set_px4ctrl_param_if_requested(
            param_name="td.enable",
            value=self.args.td_enable,
            timeout_s=self.args.td_param_timeout_s,
        )

    def set_attitude_feedback_mode_if_requested(self) -> bool:
        if self.args.attitude_feedback_mode is None:
            return True
        return self.set_px4ctrl_param_if_requested(
            param_name="attitude.feedback_mode",
            value=self.args.attitude_feedback_mode,
            timeout_s=self.args.attitude_param_timeout_s,
        )

    def publish_takeoff(self) -> None:
        msg = TakeoffLand()
        msg.takeoff_land_cmd = getattr(TakeoffLand, "TAKEOFF", 1)
        for _ in range(max(1, self.args.command_repeats)):
            self.takeoff_pub.publish(msg)
            time.sleep(1.0 / max(1.0, self.args.command_rate_hz))
        self.takeoff_sent = True

    def publish_land(self) -> None:
        msg = TakeoffLand()
        msg.takeoff_land_cmd = getattr(TakeoffLand, "LAND", 2)
        for _ in range(max(1, self.args.command_repeats)):
            self.takeoff_pub.publish(msg)
            time.sleep(1.0 / max(1.0, self.args.command_rate_hz))

    def wait_for_auto_hover(self) -> bool:
        deadline = time.monotonic() + self.args.takeoff_timeout
        saw_takeoff = False
        last_report = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            if self.fsm_state == "AUTO_TAKEOFF":
                saw_takeoff = True
            if self.fsm_state == "AUTO_HOVER":
                if saw_takeoff:
                    self.get_logger().info("AUTO_TAKEOFF -> AUTO_HOVER reached.")
                else:
                    self.get_logger().info("AUTO_HOVER reached.")
                return True

            now = time.monotonic()
            if now - last_report >= 1.0:
                self.get_logger().info(
                    f"Waiting for AUTO_HOVER, current px4ctrl state: {self.fsm_state}"
                )
                last_report = now
            time.sleep(0.05)
        return False

    def start_bag_recording(self) -> bool:
        if not self.args.record_bag:
            return True
        if self.bag_process is not None:
            return self.bag_process.poll() is None

        bag_root = Path(self.args.bag_root).expanduser().resolve()
        bag_root.mkdir(parents=True, exist_ok=True)
        bag_prefix = getattr(self.args, "bag_prefix", "ude_physical_wls")
        bag_name = f"{bag_prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.bag_output = bag_root / bag_name
        self.bag_log_path = bag_root / f"{bag_name}.record.log"
        self.bag_log_handle = self.bag_log_path.open("w", encoding="utf-8")

        command = [
            "ros2",
            "bag",
            "record",
            "--storage",
            self.args.bag_storage,
            "--output",
            str(self.bag_output),
            *self.args.bag_topic,
        ]
        self.get_logger().info(
            f"Starting rosbag after AUTO_HOVER: {self.bag_output}"
        )
        try:
            self.bag_process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=self.bag_log_handle,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        except OSError as exc:
            self.get_logger().error(f"Failed to start rosbag: {exc}")
            self.bag_log_handle.close()
            self.bag_log_handle = None
            return False

        time.sleep(max(0.2, self.args.bag_startup_wait_s))
        if self.bag_process.poll() is not None:
            return_code = self.bag_process.returncode
            self.get_logger().error(
                f"rosbag exited during startup with code {return_code}; "
                f"see {self.bag_log_path}"
            )
            self.bag_log_handle.close()
            self.bag_log_handle = None
            return False

        self.get_logger().info(
            f"rosbag is recording {len(self.args.bag_topic)} topics."
        )
        return True

    def stop_bag_recording(self) -> None:
        process = self.bag_process
        if process is None:
            return

        if process.poll() is None:
            self.get_logger().info("Stopping rosbag and flushing metadata.")
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=self.args.bag_stop_timeout_s)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if process.poll() is None:
                    self.get_logger().warn("rosbag did not stop on SIGINT; sending SIGTERM.")
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    try:
                        process.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        self.get_logger().warn("rosbag did not stop on SIGTERM; sending SIGKILL.")
                        try:
                            os.killpg(process.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                        process.wait(timeout=3.0)

        if self.bag_log_handle is not None:
            self.bag_log_handle.close()
            self.bag_log_handle = None
        self.get_logger().info(
            f"rosbag stopped with code {process.returncode}; output={self.bag_output}"
        )
        self.bag_process = None

    def analyze_recorded_bag(self) -> bool:
        if not self.args.analyze_bag or self.bag_output is None:
            return True
        if not self.bag_output.exists():
            self.get_logger().error(f"Cannot analyze missing bag: {self.bag_output}")
            return False

        analyzer = Path(__file__).resolve().with_name("analyze_physical_wls_bag.py")
        if not analyzer.exists():
            self.get_logger().error(f"Bag analyzer not found: {analyzer}")
            return False

        self.get_logger().info(f"Analyzing recorded bag: {self.bag_output}")
        result = subprocess.run(
            [
                sys.executable,
                str(analyzer),
                str(self.bag_output),
                "--storage-id",
                self.args.bag_storage,
            ],
            check=False,
        )
        if result.returncode != 0:
            self.get_logger().error(
                f"Bag analysis reported a failure (code {result.returncode})."
            )
            return False
        self.get_logger().info("Bag analysis passed its critical checks.")
        return True

    def wait_for_state(self, target: str, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if self.fsm_state == target:
                return True
            time.sleep(0.05)
        return False

    def pose_cmd_from_pose(self, pose) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom.header.frame_id if self.odom is not None and self.odom.header.frame_id else self.args.frame_id
        msg.pose = copy.deepcopy(pose)
        return msg

    def current_odom_pose_cmd(self) -> PoseStamped:
        if self.odom is None:
            raise RuntimeError("No odom message available for command.")
        return self.pose_cmd_from_pose(self.odom.pose.pose)

    def publish_current_pose_cmd(self, duration_s: float) -> None:
        if self.odom is None:
            raise RuntimeError("No odom message available for hold command.")

        deadline = time.monotonic() + duration_s
        rate = max(1.0, self.args.cmd_rate_hz)
        while rclpy.ok() and time.monotonic() < deadline:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.odom.header.frame_id or self.args.frame_id
            msg.pose = self.odom.pose.pose
            self.cmd_pub.publish(msg)
            time.sleep(1.0 / rate)

    def cmd_inputs_ready(self) -> tuple[bool, list[str]]:
        required = [
            "px4ctrl_state",
            "mavros_state",
            "odom",
            "imu",
        ]
        missing = [
            name
            for name in required
            if not self.is_fresh(name, self.args.fresh_timeout)
        ]
        if (
            not self.args.skip_mavros_connected_check
            and self.mavros_state is not None
            and not self.mavros_state.connected
        ):
            missing.append("mavros_connected")
        return len(missing) == 0, missing

    def global_axis_bounds(self) -> list[tuple[float, float]]:
        return [
            (self.args.limit_x_min, self.args.limit_x_max),
            (self.args.limit_y_min, self.args.limit_y_max),
            (self.args.limit_z_min, self.args.limit_z_max),
        ]

    def offset_in_limits(self, origin_axis_value: float, axis_index: int, offset: float) -> bool:
        if abs(offset) > self.args.wp_axis_limit_m + 1e-9:
            return False
        low, high = self.global_axis_bounds()[axis_index]
        value = origin_axis_value + offset
        return low <= value <= high

    def sample_next_axis_offset(
        self,
        origin_axis_value: float,
        axis_index: int,
        current_offset: float,
    ) -> float:
        step_min = min(self.args.wp_step_min_m, self.args.wp_step_max_m)
        step_max = max(self.args.wp_step_min_m, self.args.wp_step_max_m)
        for _ in range(max(1, self.args.wp_resample_attempts)):
            step = self.rng.uniform(step_min, step_max)
            direction = -1.0 if self.rng.random() < 0.5 else 1.0
            candidate = current_offset + direction * step
            if self.offset_in_limits(origin_axis_value, axis_index, candidate):
                return candidate

        low, high = self.global_axis_bounds()[axis_index]
        safe_low = max(-self.args.wp_axis_limit_m, low - origin_axis_value)
        safe_high = min(self.args.wp_axis_limit_m, high - origin_axis_value)
        if safe_low <= 0.0 <= safe_high:
            return 0.0
        return min(max(current_offset, safe_low), safe_high)

    def axis_toggle_values(self, axis_index: int) -> tuple[float, float]:
        pairs = [
            (self.args.wp_x_low, self.args.wp_x_high),
            (self.args.wp_y_low, self.args.wp_y_high),
            (self.args.wp_z_low, self.args.wp_z_high),
        ]
        low, high = pairs[axis_index]
        return (min(low, high), max(low, high))

    def set_axis_value(self, pose, axis_index: int, value: float) -> None:
        if axis_index == 0:
            pose.position.x = value
        elif axis_index == 1:
            pose.position.y = value
        else:
            pose.position.z = value

    def publish_until_cmd_ctrl(self) -> bool:
        self.get_logger().info(
            "Publishing current odom pose until CMD_CTRL. RC gate uses CH5/CH6 only; CH10 is ignored here."
        )
        rate = max(1.0, self.args.wp_rate_hz)
        deadline = time.monotonic() + self.args.cmd_state_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            ready, missing = self.cmd_inputs_ready()
            if not ready:
                self.get_logger().warn("CMD entry inputs stale: " + ", ".join(missing))
                return False
            if self.fsm_state == "CMD_CTRL":
                return True
            if self.fsm_state not in ("AUTO_HOVER", "CMD_CTRL"):
                self.get_logger().warn(f"Cannot enter CMD_CTRL from px4ctrl state {self.fsm_state}.")
                return False
            self.cmd_pub.publish(self.current_odom_pose_cmd())
            time.sleep(1.0 / rate)
        return self.fsm_state == "CMD_CTRL"

    def run_axis_waypoints(self) -> None:
        axis_map = {"x": 0, "y": 1, "z": 2}
        axis_index = axis_map[self.args.test_axis]
        axis_name = self.args.test_axis

        if not self.publish_until_cmd_ctrl():
            self.get_logger().warn(
                "CMD_CTRL was not reached. Check CH5/CH6 command gate and px4ctrl logs."
            )
            return

        if self.odom is None:
            self.get_logger().warn("No odom available when CMD_CTRL was reached.")
            return

        origin_pose = copy.deepcopy(self.odom.pose.pose)
        origin = [
            origin_pose.position.x,
            origin_pose.position.y,
            origin_pose.position.z,
        ]
        bounds = self.global_axis_bounds()
        for i, value in enumerate(origin):
            low, high = bounds[i]
            if not (low <= value <= high):
                self.get_logger().warn(
                    f"Waypoint origin axis {i}={value:.3f} is outside limits [{low:.3f},{high:.3f}]."
                )
                return

        self.get_logger().info(
            f"CMD_CTRL reached. Waypoint origin locked at "
            f"({origin[0]:.3f}, {origin[1]:.3f}, {origin[2]:.3f}); "
            f"testing only {axis_name}-axis with mode={self.args.wp_mode}."
        )

        current_offset = 0.0
        rate = max(1.0, self.args.wp_rate_hz)
        waypoint_index = 0
        previous_axis_value = origin[axis_index]
        toggle_values = self.axis_toggle_values(axis_index)
        if self.args.wp_mode == "toggle":
            low, high = bounds[axis_index]
            for value in toggle_values:
                if not (low <= value <= high):
                    self.get_logger().warn(
                        f"Toggle waypoint {axis_name}={value:.3f} is outside limits "
                        f"[{low:.3f},{high:.3f}]."
                    )
                    return
            self.get_logger().info(
                f"Toggle waypoints: {axis_name}={toggle_values[0]:.3f} <-> "
                f"{toggle_values[1]:.3f}; non-test axes hold origin."
            )

        while rclpy.ok():
            ready, missing = self.cmd_inputs_ready()
            if not ready:
                self.get_logger().warn("Stopping waypoint publishing; inputs stale: " + ", ".join(missing))
                return
            if self.fsm_state != "CMD_CTRL":
                self.get_logger().info(
                    f"px4ctrl left CMD_CTRL ({self.fsm_state}); stopping waypoints."
                )
                return

            if self.args.wp_mode == "random":
                target_offset = self.sample_next_axis_offset(
                    origin[axis_index],
                    axis_index,
                    current_offset,
                )
                target_value = origin[axis_index] + target_offset
            else:
                target_value = toggle_values[waypoint_index % 2]
                target_offset = target_value - origin[axis_index]

            target_pose = copy.deepcopy(origin_pose)
            self.set_axis_value(target_pose, axis_index, target_value)

            waypoint_index += 1
            self.get_logger().info(
                f"waypoint #{waypoint_index}: mode={self.args.wp_mode} axis={axis_name} "
                f"value={target_value:.3f} offset={target_offset:+.3f}m "
                f"step={target_value - previous_axis_value:+.3f}m"
            )

            hold_deadline = time.monotonic() + self.args.wp_hold_s
            while rclpy.ok() and time.monotonic() < hold_deadline:
                ready, missing = self.cmd_inputs_ready()
                if not ready:
                    self.get_logger().warn(
                        "Stopping waypoint publishing; inputs stale: " + ", ".join(missing)
                    )
                    return
                if self.fsm_state != "CMD_CTRL":
                    self.get_logger().info(
                        f"px4ctrl left CMD_CTRL ({self.fsm_state}); stopping waypoints."
                    )
                    return
                self.cmd_pub.publish(self.pose_cmd_from_pose(target_pose))
                time.sleep(1.0 / rate)

            current_offset = target_offset
            previous_axis_value = target_value

    def run_optional_cmd_ctrl_test(self) -> None:
        self.get_logger().info(
            "Publishing current-pose /position_cmd to test CMD_CTRL. "
            "RC must be in command mode for px4ctrl to switch."
        )
        self.publish_current_pose_cmd(self.args.cmd_hold_duration)
        entered_cmd = self.wait_for_state("CMD_CTRL", self.args.cmd_state_timeout)
        if entered_cmd:
            self.get_logger().info("CMD_CTRL reached. Stopping /position_cmd to return to AUTO_HOVER.")
            returned = self.wait_for_state("AUTO_HOVER", self.args.cmd_return_timeout)
            if returned:
                self.get_logger().info("CMD_CTRL -> AUTO_HOVER reached after command timeout.")
            else:
                self.get_logger().warn(
                    f"CMD_CTRL did not return to AUTO_HOVER within {self.args.cmd_return_timeout:.1f}s."
                )
        else:
            self.get_logger().warn(
                "CMD_CTRL was not reached. This is expected if RC command mode is not active."
            )

    def safe_to_clean_stack(self) -> bool:
        armed = self.mavros_state.armed if self.mavros_state is not None else True
        return not armed

    def print_status(self) -> None:
        mavros = self.mavros_state
        connected = mavros.connected if mavros is not None else None
        armed = mavros.armed if mavros is not None else None
        mode = mavros.mode if mavros is not None else None

        if self.odom is not None:
            p = self.odom.pose.pose.position
            yaw = yaw_from_quaternion(self.odom)
            odom_text = f"p=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}), yaw={yaw:.2f}"
        else:
            odom_text = "p=n/a"

        if self.attitude_sp is not None:
            br = self.attitude_sp.body_rate
            sp_text = (
                f"bodyrate=({br.x:.3f}, {br.y:.3f}, {br.z:.3f}), "
                f"thrust={self.attitude_sp.thrust:.3f}, mask={self.attitude_sp.type_mask}"
            )
        else:
            sp_text = "bodyrate/thrust=n/a"

        self.get_logger().info(
            f"px4ctrl={self.fsm_state}, mavros connected={connected}, "
            f"armed={armed}, mode={mode}, {odom_text}, {sp_text}"
        )

    def wait_for_safe_landing(self) -> bool:
        deadline = time.monotonic() + self.args.land_timeout
        last_report = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            if self.safe_to_clean_stack():
                return True

            now = time.monotonic()
            if now - last_report >= 1.0:
                self.print_status()
                last_report = now
            time.sleep(0.05)
        return False

    def interactive_after_hover(self) -> int:
        while rclpy.ok():
            self.print_status()
            print(
                "\nAUTO_HOVER is active. Type STATUS, LAND, or EXIT.\n"
                "Use EXIT only after the vehicle is already safe by another method."
            )
            try:
                choice = input("> ").strip().upper()
            except EOFError:
                if self.safe_to_clean_stack():
                    return 0
                self.get_logger().warn(
                    "Interactive stdin closed while the vehicle is armed. Continuing monitoring and "
                    "bag recording until the vehicle is disarmed; land with the RC."
                )
                while rclpy.ok() and not self.safe_to_clean_stack():
                    self.print_status()
                    time.sleep(1.0)
                return 0 if self.safe_to_clean_stack() else MAYBE_AIRBORNE_EXIT_CODE

            if choice == "STATUS":
                continue
            if choice == "LAND":
                self.publish_land()
                if self.wait_for_safe_landing():
                    self.get_logger().info("Landing completed or vehicle is disarmed/manual.")
                    return 0
                self.get_logger().warn("Landing was requested, but safe landed state was not confirmed.")
                return MAYBE_AIRBORNE_EXIT_CODE
            if choice == "EXIT":
                if self.safe_to_clean_stack():
                    return 0
                self.get_logger().warn("Exiting while the vehicle may still be airborne.")
                return MAYBE_AIRBORNE_EXIT_CODE

            print("Unknown command. Use STATUS, LAND, or EXIT.")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Independent UDE/bodyrate AUTO_HOVER/CMD_CTRL waypoint test helper."
    )
    parser.add_argument("--state-topic", default="/px4ctrl/state")
    parser.add_argument("--takeoff-land-topic", default="/px4ctrl/takeoff_land")
    parser.add_argument("--cmd-topic", default="/position_cmd")
    parser.add_argument("--mavros-state-topic", default="/mavros/state")
    parser.add_argument("--odom-topic", default="/mavros/local_position/odom")
    parser.add_argument("--imu-topic", default="/mavros/imu/data")
    parser.add_argument("--vision-topic", default="/mavros/vision_pose/pose")
    parser.add_argument("--mocap-twist-topic", default="/vla_drone1/twist")
    parser.add_argument("--mocap-state-status-topic", default="/px4ctrl/mocap_state_status")
    parser.add_argument("--attitude-setpoint-topic", default="/mavros/setpoint_raw/attitude")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    parser.add_argument("--fresh-timeout", type=float, default=2.0)
    parser.add_argument("--takeoff-timeout", type=float, default=30.0)
    parser.add_argument("--land-timeout", type=float, default=30.0)
    parser.add_argument("--command-repeats", type=int, default=8)
    parser.add_argument("--command-rate-hz", type=float, default=10.0)
    parser.add_argument("--cmd-rate-hz", type=float, default=20.0)
    parser.add_argument("--cmd-hold-duration", type=float, default=5.0)
    parser.add_argument("--cmd-state-timeout", type=float, default=5.0)
    parser.add_argument("--cmd-return-timeout", type=float, default=3.0)
    parser.add_argument("--enter-cmd", action="store_true")
    parser.add_argument("--run-waypoints", action="store_true")
    parser.add_argument("--test-axis", choices=("x", "y", "z"), default="x")
    parser.add_argument("--wp-mode", choices=("toggle", "random"), default="toggle")
    parser.add_argument("--wp-x-low", type=float, default=-1.0)
    parser.add_argument("--wp-x-high", type=float, default=1.0)
    parser.add_argument("--wp-y-low", type=float, default=-1.0)
    parser.add_argument("--wp-y-high", type=float, default=1.0)
    parser.add_argument("--wp-z-low", type=float, default=0.6)
    parser.add_argument("--wp-z-high", type=float, default=1.2)
    parser.add_argument("--wp-step-min-m", type=float, default=0.05)
    parser.add_argument("--wp-step-max-m", type=float, default=1.0)
    parser.add_argument("--wp-axis-limit-m", type=float, default=1.0)
    parser.add_argument("--wp-hold-s", type=float, default=4.0)
    parser.add_argument("--wp-rate-hz", type=float, default=20.0)
    parser.add_argument("--wp-resample-attempts", type=int, default=50)
    parser.add_argument("--wp-random-seed", type=int, default=None)
    parser.add_argument("--limit-x-min", type=float, default=-7.0)
    parser.add_argument("--limit-x-max", type=float, default=14.0)
    parser.add_argument("--limit-y-min", type=float, default=-2.5)
    parser.add_argument("--limit-y-max", type=float, default=2.5)
    parser.add_argument("--limit-z-min", type=float, default=-0.3)
    parser.add_argument("--limit-z-max", type=float, default=2.5)
    parser.add_argument(
        "--publish-takeoff",
        action="store_true",
        help=(
            "Publish /px4ctrl/takeoff_land TAKEOFF from this helper. "
            "By default the helper waits for manual takeoff and AUTO_HOVER."
        ),
    )
    parser.add_argument(
        "--auto-confirm",
        action="store_true",
        help="Only used with --publish-takeoff; publish TAKEOFF without typing TAKEOFF.",
    )
    parser.add_argument("--skip-mavros-connected-check", action="store_true")
    parser.add_argument("--skip-setpoint-check", action="store_true")
    parser.add_argument("--require-vision-pose", action="store_true")
    parser.add_argument(
        "--require-composed-state",
        action="store_true",
        help="Require synchronized mocap p/v and FCU odom q/w before TAKEOFF.",
    )
    parser.add_argument("--ude-enable", choices=("true", "false"), default=None)
    parser.add_argument("--ude-param-timeout-s", type=float, default=10.0)
    parser.add_argument("--td-enable", choices=("true", "false"), default=None)
    parser.add_argument("--td-param-timeout-s", type=float, default=10.0)
    parser.add_argument(
        "--attitude-feedback-mode",
        choices=("full_quaternion", "reduced_attitude"),
        default=None,
    )
    parser.add_argument("--attitude-param-timeout-s", type=float, default=10.0)
    parser.add_argument(
        "--record-bag",
        action="store_true",
        help="Start ros2 bag recording only after AUTO_HOVER is reached.",
    )
    parser.add_argument(
        "--analyze-bag",
        action="store_true",
        help="Analyze the recorded ROS bag after it is stopped.",
    )
    parser.add_argument("--bag-root", default="bags")
    parser.add_argument("--bag-storage", default="sqlite3")
    parser.add_argument("--bag-startup-wait-s", type=float, default=1.0)
    parser.add_argument("--bag-stop-timeout-s", type=float, default=15.0)
    parser.add_argument(
        "--bag-topic",
        action="append",
        dest="bag_topic",
        default=list(DEFAULT_BAG_TOPICS),
        help="Additional topic to record; may be repeated.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv if argv is None else argv
    parser = build_arg_parser()
    args = parser.parse_args(remove_ros_args(args=argv)[1:])

    rclpy.init(args=argv)
    node = TakeoffHoverTest(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    exit_code = 0
    try:
        if not node.wait_for_startup():
            return 1

        if not node.set_ude_enable_if_requested():
            return 1

        if not node.set_td_enable_if_requested():
            return 1

        if not node.set_attitude_feedback_mode_if_requested():
            return 1

        node.print_status()

        if args.publish_takeoff:
            if node.fsm_state != "MANUAL_CTRL":
                node.get_logger().error(
                    "Refusing to publish TAKEOFF because px4ctrl is already in "
                    f"{node.fsm_state}. This usually means an old test stack is still "
                    "running. Stop that stack first so UDE/TD/controller state starts clean."
                )
                return 1

            print(
                "\nBefore takeoff: keep RC in hover+command mode, sticks centered, "
                "and verify the flight area is clear. Stack output is quiet here."
            )
            if args.auto_confirm:
                print("--auto-confirm is set; publishing TAKEOFF.")
            else:
                input("Press Enter once to publish /px4ctrl/takeoff_land TAKEOFF: ")

            node.publish_takeoff()
        else:
            if args.auto_confirm:
                node.get_logger().warn(
                    "--auto-confirm is ignored because --publish-takeoff is not set."
                )
            print(
                "\nManual takeoff mode: this helper will not publish "
                "/px4ctrl/takeoff_land. Take off manually with the normal UDE "
                "procedure, then the helper will wait for px4ctrl AUTO_HOVER."
            )

        if not node.wait_for_auto_hover():
            node.get_logger().error(
                "AUTO_HOVER was not reached. Check px4ctrl logs for takeoff rejection conditions."
            )
            return 1 if node.safe_to_clean_stack() else MAYBE_AIRBORNE_EXIT_CODE

        if not node.start_bag_recording():
            node.get_logger().error(
                "Bag recording was requested but did not start. Skipping waypoint "
                "commands; use LAND in the interactive prompt."
            )
            return node.interactive_after_hover()

        if args.run_waypoints:
            node.run_axis_waypoints()
        elif args.enter_cmd:
            node.run_optional_cmd_ctrl_test()

        exit_code = node.interactive_after_hover()
        return exit_code
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted.")
        if node.safe_to_clean_stack():
            return 0
        return MAYBE_AIRBORNE_EXIT_CODE
    finally:
        node.stop_bag_recording()
        node.analyze_recorded_bag()
        executor.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
