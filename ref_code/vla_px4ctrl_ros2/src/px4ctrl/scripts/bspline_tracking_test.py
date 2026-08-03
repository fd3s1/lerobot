#!/usr/bin/python3

from __future__ import annotations

import argparse
from collections import deque
import math
from pathlib import Path
import subprocess
import sys
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from bspline_trajectory import (
    EntryPoint,
    FigureEightPath,
    MotionLimits,
    ReferencePoint,
    distance_to_polygon_edges,
    flat_body_rate_and_acceleration,
    generate_multi_lap_reference,
    generate_transfer_reference,
    norm,
    normalize_angle,
    point_in_convex_polygon,
    smootherstep9_kinematics,
    validate_points_in_flight_area,
)
from ude_takeoff_hover_test import (
    MAYBE_AIRBORNE_EXIT_CODE,
    TakeoffHoverTest,
    build_arg_parser as build_base_arg_parser,
    yaw_from_quaternion,
)


DEFAULT_BOUNDARY = (
    (15.1, 2.1),
    (15.1, -2.1),
    (-2.0, -2.1),
    (-2.0, 2.1),
)


class FlightAbort(RuntimeError):
    pass


def parse_lap_speeds(value: str) -> tuple[float, ...]:
    try:
        speeds = tuple(float(item.strip()) for item in value.split(",") if item.strip())
    except ValueError as exc:
        raise argparse.ArgumentTypeError("lap speeds must be comma-separated numbers") from exc
    if not speeds or any(not math.isfinite(speed) or speed <= 0.0 for speed in speeds):
        raise argparse.ArgumentTypeError("lap speeds must contain positive finite values")
    return speeds


def parse_boundary_point(value: str) -> tuple[float, float]:
    try:
        x_text, y_text = value.split(",", 1)
        point = (float(x_text), float(y_text))
    except (ValueError, TypeError) as exc:
        raise argparse.ArgumentTypeError("boundary points must use x,y in metres") from exc
    if not all(math.isfinite(component) for component in point):
        raise argparse.ArgumentTypeError("boundary points must be finite")
    return point


def quaternion_yaw(msg: Odometry) -> float:
    return yaw_from_quaternion(msg)


class BsplineTrackingTest(TakeoffHoverTest):
    def __init__(self, args: argparse.Namespace) -> None:
        self._odom_arrivals: deque[float] = deque()
        self._imu_arrivals: deque[float] = deque()
        super().__init__(args)
        self.actual_state: Odometry | None = None
        self.tracking_error: Odometry | None = None
        self.battery_state: BatteryState | None = None
        self.active_reference: ReferencePoint | None = None
        self.active_phase = "STARTUP"
        self.trajectory_id = 0
        self.flight_polygon = tuple(args.boundary_point)
        self.xyz_limits = (
            args.limit_x_min,
            args.limit_x_max,
            args.limit_y_min,
            args.limit_y_max,
            args.limit_z_min,
            args.limit_z_max,
        )
        self.runtime_controller_max_angle_deg = 0.0

        self.traj_pub = self.create_publisher(PositionCommand, args.traj_topic, 30)
        self.phase_pub = self.create_publisher(String, args.phase_topic, 10)
        self.runtime_param_client = self.create_client(
            GetParameters,
            "/px4ctrl/get_parameters",
        )
        self.create_subscription(Odometry, args.actual_topic, self._actual_state_cb, 30)
        self.create_subscription(Odometry, args.tracking_error_topic, self._tracking_error_cb, 30)
        self.create_subscription(
            BatteryState,
            args.battery_topic,
            self._battery_state_cb,
            qos_profile_sensor_data,
        )

    def _record_feedback_arrival(self, history: deque[float]) -> None:
        now = time.monotonic()
        history.append(now)
        cutoff = now - self.args.feedback_rate_window_s
        while history and history[0] < cutoff:
            history.popleft()

    def _odom_cb(self, msg: Odometry) -> None:
        super()._odom_cb(msg)
        self._record_feedback_arrival(self._odom_arrivals)

    def _imu_cb(self, msg) -> None:
        super()._imu_cb(msg)
        self._record_feedback_arrival(self._imu_arrivals)

    @staticmethod
    def _arrival_rate_hz(history: deque[float]) -> float:
        if len(history) < 2:
            return 0.0
        span = history[-1] - history[0]
        return 0.0 if span <= 1e-6 else (len(history) - 1) / span

    def _actual_state_cb(self, msg: Odometry) -> None:
        self.actual_state = msg
        self._touch("actual_state")

    def _tracking_error_cb(self, msg: Odometry) -> None:
        self.tracking_error = msg
        self._touch("tracking_error")

    def _battery_state_cb(self, msg: BatteryState) -> None:
        self.battery_state = msg
        self._touch("battery")

    def physical_battery_error(self) -> str | None:
        if not self.is_fresh("battery", self.args.battery_timeout_s):
            return "physical battery telemetry is missing or stale"
        if self.battery_state is None:
            return "physical battery telemetry is unavailable"
        voltage = float(self.battery_state.voltage)
        if not self.battery_state.present:
            return f"PX4 reports battery disconnected (voltage={voltage:.3f} V)"
        if not math.isfinite(voltage):
            return "PX4 battery voltage is not finite"
        if not self.args.battery_min_voltage_v <= voltage <= self.args.battery_max_voltage_v:
            return (
                f"PX4 battery voltage {voltage:.3f} V is outside the allowed 6S range "
                f"[{self.args.battery_min_voltage_v:.3f}, "
                f"{self.args.battery_max_voltage_v:.3f}] V"
            )
        return None

    def required_inputs_ready(self) -> tuple[bool, list[str]]:
        ready, missing = super().required_inputs_ready()
        if not self.is_fresh("actual_state", self.args.fresh_timeout):
            missing.append("actual_state")
        elif self.actual_state is not None and self.actual_state.child_frame_id != self.args.expected_actual_frame:
            missing.append("actual_state_not_mocap_composed")
        battery_error = self.physical_battery_error()
        if battery_error is not None:
            missing.append(f"physical_battery({battery_error})")
        for label, history in (
            ("odom", self._odom_arrivals),
            ("imu", self._imu_arrivals),
        ):
            observed_rate = self._arrival_rate_hz(history)
            minimum_samples = max(
                2,
                int(0.75 * self.args.minimum_feedback_rate_hz * self.args.feedback_rate_window_s),
            )
            if len(history) < minimum_samples or observed_rate < self.args.minimum_feedback_rate_hz:
                missing.append(
                    f"{label}_rate({observed_rate:.1f}Hz < "
                    f"{self.args.minimum_feedback_rate_hz:.1f}Hz)"
                )
        return ready and not missing, missing

    def actual_position(self) -> tuple[float, float, float]:
        if self.actual_state is None:
            raise FlightAbort("combined actual state is unavailable")
        point = self.actual_state.pose.pose.position
        return (point.x, point.y, point.z)

    def actual_yaw(self) -> float:
        if self.actual_state is None:
            raise FlightAbort("combined actual state is unavailable")
        return quaternion_yaw(self.actual_state)

    def set_phase(self, phase: str) -> None:
        if phase != self.active_phase:
            self.get_logger().info(f"B-spline phase: {self.active_phase} -> {phase}")
            self.active_phase = phase
        msg = String()
        msg.data = phase
        self.phase_pub.publish(msg)

    @staticmethod
    def parameter_value(value: ParameterValue) -> bool | int | float | str | list[float]:
        if value.type == ParameterType.PARAMETER_BOOL:
            return value.bool_value
        if value.type == ParameterType.PARAMETER_INTEGER:
            return value.integer_value
        if value.type == ParameterType.PARAMETER_DOUBLE:
            return value.double_value
        if value.type == ParameterType.PARAMETER_STRING:
            return value.string_value
        if value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(value.double_array_value)
        raise RuntimeError(f"unsupported ROS parameter type: {value.type}")

    def get_runtime_parameters(
        self,
        names: tuple[str, ...],
        timeout_s: float = 5.0,
    ) -> dict[str, bool | int | float | str | list[float]]:
        if not self.runtime_param_client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError("/px4ctrl/get_parameters service is unavailable")

        request = GetParameters.Request()
        request.names = list(names)
        future = self.runtime_param_client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            future.cancel()
            raise RuntimeError("timed out reading px4ctrl parameters")
        if future.exception() is not None:
            raise RuntimeError(f"px4ctrl parameter request failed: {future.exception()}")

        response = future.result()
        if response is None or len(response.values) != len(names):
            raise RuntimeError("px4ctrl returned an incomplete parameter response")
        return {
            name: self.parameter_value(value)
            for name, value in zip(names, response.values)
        }

    def load_and_verify_runtime_parameters(self) -> bool:
        limit_names = (
            "limits.x_min",
            "limits.x_max",
            "limits.y_min",
            "limits.y_max",
            "limits.z_min",
            "limits.z_max",
        )
        required = {
            "physical_control.enable": True,
            "physical_control.body_rate_feedforward_scale":
                self.args.body_rate_ff_scale,
            "physical_control.angular_acceleration_feedforward_scale":
                self.args.angular_accel_ff_scale,
            "use_bodyrate_ctrl": False,
            "attitude.feedback_mode": "full_quaternion",
            "cmd_feedforward.enable": True,
            "td.enable": False,
        }
        capability_names = (
            "controller.max_angle_deg",
            "controller.max_bodyrate_x",
            "controller.max_bodyrate_y",
            "controller.max_bodyrate_z",
            "cmd_feedforward.max_velocity",
            "cmd_feedforward.max_acceleration",
            "cmd_feedforward.max_jerk",
            "cmd_feedforward.max_snap",
            "ude.max_f_hat",
            "ude.max_u_acc",
        )
        parameter_names = limit_names + tuple(required) + capability_names
        try:
            parameters = self.get_runtime_parameters(parameter_names)
        except RuntimeError as exc:
            self.get_logger().error(
                f"Cannot verify px4ctrl runtime parameters; refusing takeoff: {exc}"
            )
            return False

        values = tuple(float(parameters[name]) for name in limit_names)
        if not (values[0] < values[1] and values[2] < values[3] and values[4] < values[5]):
            self.get_logger().error(f"Invalid px4ctrl flight limits: {values}")
            return False
        polygon_x = [point[0] for point in self.flight_polygon]
        polygon_y = [point[1] for point in self.flight_polygon]
        if (
            values[0] > min(polygon_x)
            or values[1] < max(polygon_x)
            or values[2] > min(polygon_y)
            or values[3] < max(polygon_y)
        ):
            self.get_logger().error(
                "px4ctrl XY limits do not cover the full flight polygon: "
                f"limits={values[:4]}, polygon_bounds="
                f"({min(polygon_x)}, {max(polygon_x)}, {min(polygon_y)}, {max(polygon_y)})"
            )
            return False
        self.xyz_limits = values

        mismatches = []
        for name, expected in required.items():
            actual = parameters[name]
            if actual != expected:
                mismatches.append(f"{name}={actual!r}, expected {expected!r}")
        if mismatches:
            self.get_logger().error("Unsafe px4ctrl runtime configuration: " + "; ".join(mismatches))
            return False

        scalar_capability_names = capability_names[:8]
        capabilities = {name: float(parameters[name]) for name in scalar_capability_names}
        minimum_capabilities = {
            "controller.max_angle_deg": self.args.controller_max_angle_deg,
            "controller.max_bodyrate_x": self.args.controller_max_bodyrates[0],
            "controller.max_bodyrate_y": self.args.controller_max_bodyrates[1],
            "controller.max_bodyrate_z": self.args.controller_max_bodyrates[2],
            "cmd_feedforward.max_velocity": max(self.args.lap_speeds),
            "cmd_feedforward.max_acceleration": max(
                self.args.max_long_accel_mps2,
                self.args.max_decel_mps2,
                self.args.max_lateral_accel_mps2,
            ),
            "cmd_feedforward.max_jerk": self.args.max_jerk_mps3,
            "cmd_feedforward.max_snap": self.args.max_snap_mps4,
        }
        insufficient = [
            f"{name}={capabilities[name]:.3f}, needs >= {minimum:.3f}"
            for name, minimum in minimum_capabilities.items()
            if not math.isfinite(capabilities[name]) or capabilities[name] + 1e-6 < minimum
        ]
        if insufficient:
            self.get_logger().error(
                "px4ctrl trajectory capability is below this plan: " + "; ".join(insufficient)
            )
            return False
        for name, minimum in (
            ("ude.max_f_hat", self.args.ude_max_f_hat),
            ("ude.max_u_acc", self.args.ude_max_u_acc),
        ):
            actual = parameters[name]
            if not isinstance(actual, list) or len(actual) != 3:
                self.get_logger().error(f"px4ctrl parameter {name} is not a three-axis array: {actual!r}")
                return False
            if any(not math.isfinite(float(value)) or float(value) + 1e-6 < required_value
                   for value, required_value in zip(actual, minimum)):
                self.get_logger().error(
                    f"px4ctrl trajectory capability is below this plan: {name}={actual}, needs >= {minimum}"
                )
                return False
        self.runtime_controller_max_angle_deg = capabilities["controller.max_angle_deg"]
        self.get_logger().info(
            "px4ctrl runtime verified: physical ACTIVE request, attitude/full-quaternion, "
            "trajectory feedforward enabled, TD disabled, "
            f"body_rate_ff_scale={self.args.body_rate_ff_scale:.3f}, "
            f"angular_accel_ff_scale={self.args.angular_accel_ff_scale:.3f}, "
            f"max_angle={self.runtime_controller_max_angle_deg:.1f}deg, "
            f"bodyrate_limits={self.args.controller_max_bodyrates}, "
            f"ude_f_hat={self.args.ude_max_f_hat}, ude_u_acc={self.args.ude_max_u_acc}, "
            f"limits={self.xyz_limits}"
        )
        return True

    def build_path(self) -> FigureEightPath:
        center_x = sum(point[0] for point in self.flight_polygon) / len(self.flight_polygon)
        center_y = sum(point[1] for point in self.flight_polygon) / len(self.flight_polygon)
        right_midpoint = (
            0.5 * (self.flight_polygon[0][0] + self.flight_polygon[1][0]),
            0.5 * (self.flight_polygon[0][1] + self.flight_polygon[1][1]),
        )
        left_midpoint = (
            0.5 * (self.flight_polygon[2][0] + self.flight_polygon[3][0]),
            0.5 * (self.flight_polygon[2][1] + self.flight_polygon[3][1]),
        )
        base_heading = math.atan2(
            right_midpoint[1] - left_midpoint[1],
            right_midpoint[0] - left_midpoint[0],
        )
        return FigureEightPath(
            center_xy=(
                center_x + self.args.center_x_offset_m,
                center_y + self.args.center_y_offset_m,
            ),
            x_span_m=self.args.x_span_m,
            y_span_m=self.args.y_span_m,
            heading_rad=base_heading + math.radians(self.args.heading_offset_deg),
            z_min_m=self.args.z_min_m,
            z_max_m=self.args.z_max_m,
            control_count=self.args.control_count,
            arc_samples=self.args.arc_samples,
        )

    def validate_main_path(self, path: FigureEightPath) -> None:
        sample_count = max(1000, self.args.validation_samples)
        points = [
            path.sample(path.length_m * index / sample_count).position
            for index in range(sample_count)
        ]
        valid, reason = validate_points_in_flight_area(
            points,
            polygon=self.flight_polygon,
            margin_m=self.args.safety_margin_m,
            xyz_limits=self.xyz_limits,
            require_margin=True,
        )
        if not valid:
            raise FlightAbort(f"main B-spline path is unsafe: {reason}")

    def validate_transfer(self, reference: list[ReferencePoint], name: str) -> None:
        valid, reason = validate_points_in_flight_area(
            (point.position for point in reference),
            polygon=self.flight_polygon,
            margin_m=0.0,
            xyz_limits=self.xyz_limits,
            require_margin=False,
        )
        if not valid:
            raise FlightAbort(f"{name} transfer is unsafe: {reason}")

    def build_position_command(self, reference: ReferencePoint, active: bool = True) -> PositionCommand:
        msg = PositionCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.position.x, msg.position.y, msg.position.z = reference.position
        msg.velocity.x, msg.velocity.y, msg.velocity.z = reference.velocity
        msg.acceleration.x, msg.acceleration.y, msg.acceleration.z = reference.acceleration
        msg.jerk.x, msg.jerk.y, msg.jerk.z = reference.jerk
        msg.snap.x, msg.snap.y, msg.snap.z = reference.snap
        msg.yaw = reference.yaw
        msg.yaw_dot = reference.yaw_rate
        msg.yaw_ddot = reference.yaw_acceleration
        msg.trajectory_id = self.trajectory_id
        msg.trajectory_flag = (
            getattr(PositionCommand, "TRAJECTORY_STATUS_READY", 1) if active else
            getattr(PositionCommand, "TRAJECTORY_STATUS_COMPLETED", 3)
        )
        return msg

    def current_inputs_safe(
        self,
        require_cmd_ctrl: bool,
        *,
        enforce_position_bounds: bool = True,
    ) -> None:
        if not self.is_fresh("actual_state", self.args.actual_timeout_s):
            raise FlightAbort("combined actual state timed out")
        if not self.is_fresh("mocap_state_status", self.args.actual_timeout_s):
            raise FlightAbort("mocap composed-state status timed out")
        battery_error = self.physical_battery_error()
        if battery_error is not None:
            raise FlightAbort(battery_error)
        status = self.mocap_state_status or ""
        if "p_source=mocap" not in status or "v_source=mocap" not in status or "reason='mocap_synced'" not in status:
            raise FlightAbort(f"combined state is no longer mocap-synchronized: {status}")
        if require_cmd_ctrl and self.fsm_state != "CMD_CTRL":
            raise FlightAbort(f"px4ctrl left CMD_CTRL: state={self.fsm_state}")
        if not enforce_position_bounds:
            return
        position = self.actual_position()
        if not point_in_convex_polygon(position[:2], self.flight_polygon):
            raise FlightAbort(f"actual position left flight polygon: {position}")
        x_min, x_max, y_min, y_max, z_min, z_max = self.xyz_limits
        if not (x_min <= position[0] <= x_max and y_min <= position[1] <= y_max and z_min <= position[2] <= z_max):
            raise FlightAbort(f"actual position left px4ctrl limits: {position}")

    def check_tracking_error(self, reference: ReferencePoint, phase_start_s: float) -> None:
        if time.monotonic() - phase_start_s < self.args.error_grace_s:
            return
        actual = self.actual_position()
        dx = reference.position[0] - actual[0]
        dy = reference.position[1] - actual[1]
        dz = reference.position[2] - actual[2]
        if math.hypot(dx, dy) > self.args.max_xy_error_m:
            raise FlightAbort(
                f"horizontal tracking error {math.hypot(dx, dy):.3f} m exceeds "
                f"{self.args.max_xy_error_m:.3f} m"
            )
        if abs(dz) > self.args.max_z_error_m:
            raise FlightAbort(
                f"vertical tracking error {abs(dz):.3f} m exceeds {self.args.max_z_error_m:.3f} m"
            )

    def publish_reference_sequence(
        self,
        reference: list[ReferencePoint],
        phase: str,
        *,
        require_cmd_ctrl: bool = True,
        enforce_position_bounds: bool = True,
    ) -> None:
        self.trajectory_id += 1
        self.set_phase(phase)
        period_s = 1.0 / self.args.rate_hz
        phase_start_s = time.monotonic()
        next_tick = phase_start_s
        for point in reference:
            if not rclpy.ok():
                raise FlightAbort("ROS shutdown during trajectory")
            self.current_inputs_safe(
                require_cmd_ctrl=require_cmd_ctrl,
                enforce_position_bounds=enforce_position_bounds,
            )
            self.check_tracking_error(point, phase_start_s)
            now_s = time.monotonic()
            lateness = now_s - next_tick
            if lateness > self.args.max_publish_lateness_s:
                raise FlightAbort(
                    f"trajectory publisher missed deadline by {lateness * 1000.0:.1f} ms"
                )
            self.active_reference = point
            self.traj_pub.publish(self.build_position_command(point))
            self.set_phase(phase)
            next_tick += period_s
            time.sleep(max(0.0, next_tick - time.monotonic()))

    def publish_continuous_laps(self, reference: list[ReferencePoint]) -> None:
        if not reference:
            raise FlightAbort("continuous lap reference is empty")
        self.trajectory_id += 1
        period_s = 1.0 / self.args.rate_hz
        next_tick = time.monotonic()
        active_lap = -1
        phase_start_s = next_tick
        for point in reference:
            if not rclpy.ok():
                raise FlightAbort("ROS shutdown during continuous laps")
            if point.lap_index != active_lap:
                active_lap = point.lap_index
                phase_start_s = time.monotonic()
                self.set_phase(f"LAP_{active_lap + 1}")
            self.current_inputs_safe(require_cmd_ctrl=True)
            self.check_tracking_error(point, phase_start_s)
            lateness = time.monotonic() - next_tick
            if lateness > self.args.max_publish_lateness_s:
                raise FlightAbort(
                    f"trajectory publisher missed deadline by {lateness * 1000.0:.1f} ms"
                )
            self.active_reference = point
            self.traj_pub.publish(self.build_position_command(point))
            self.set_phase(f"LAP_{active_lap + 1}")
            next_tick += period_s
            time.sleep(max(0.0, next_tick - time.monotonic()))

    def hold_reference(
        self,
        position: tuple[float, float, float],
        yaw: float,
        duration_s: float,
        phase: str,
        *,
        require_cmd_ctrl: bool = True,
        enforce_position_bounds: bool = True,
    ) -> None:
        count = max(1, math.ceil(duration_s * self.args.rate_hz))
        point = ReferencePoint(
            time_s=0.0,
            position=position,
            velocity=(0.0, 0.0, 0.0),
            acceleration=(0.0, 0.0, 0.0),
            jerk=(0.0, 0.0, 0.0),
            snap=(0.0, 0.0, 0.0),
            yaw=yaw,
            yaw_rate=0.0,
            yaw_acceleration=0.0,
            horizontal_speed_mps=0.0,
            horizontal_accel_mps2=0.0,
        )
        self.publish_reference_sequence(
            [point] * count,
            phase,
            require_cmd_ctrl=require_cmd_ctrl,
            enforce_position_bounds=enforce_position_bounds,
        )

    def enter_cmd_ctrl(self, position: tuple[float, float, float], yaw: float) -> None:
        self.trajectory_id += 1
        point = ReferencePoint(
            time_s=0.0,
            position=position,
            velocity=(0.0, 0.0, 0.0),
            acceleration=(0.0, 0.0, 0.0),
            jerk=(0.0, 0.0, 0.0),
            snap=(0.0, 0.0, 0.0),
            yaw=yaw,
            yaw_rate=0.0,
            yaw_acceleration=0.0,
            horizontal_speed_mps=0.0,
            horizontal_accel_mps2=0.0,
        )
        deadline = time.monotonic() + self.args.cmd_state_timeout
        self.set_phase("ENTER_CMD")
        while rclpy.ok() and time.monotonic() < deadline:
            self.current_inputs_safe(require_cmd_ctrl=False)
            self.traj_pub.publish(self.build_position_command(point))
            self.set_phase("ENTER_CMD")
            if self.fsm_state == "CMD_CTRL":
                self.get_logger().info("CMD_CTRL reached through /position_cmd_traj hold.")
                return
            if self.fsm_state != "AUTO_HOVER":
                raise FlightAbort(f"cannot enter CMD_CTRL from state={self.fsm_state}")
            time.sleep(1.0 / self.args.rate_hz)
        raise FlightAbort("timed out waiting for CMD_CTRL")

    def align_yaw(
        self,
        position: tuple[float, float, float],
        target_yaw: float,
        phase: str,
    ) -> None:
        start_yaw = self.actual_yaw()
        delta = normalize_angle(target_yaw - start_yaw)
        duration = max(1.0, 1.875 * abs(delta) / self.args.align_yaw_rate_radps)
        steps = max(1, math.ceil(duration * self.args.rate_hz))
        reference: list[ReferencePoint] = []
        for step in range(steps + 1):
            time_s = step / self.args.rate_hz
            u = min(1.0, time_s / duration)
            blend, blend_d, blend_dd, _, _ = smootherstep9_kinematics(u, duration)
            reference.append(
                ReferencePoint(
                    time_s=time_s,
                    position=position,
                    velocity=(0.0, 0.0, 0.0),
                    acceleration=(0.0, 0.0, 0.0),
                    jerk=(0.0, 0.0, 0.0),
                    snap=(0.0, 0.0, 0.0),
                    yaw=normalize_angle(start_yaw + delta * blend),
                    yaw_rate=delta * blend_d,
                    yaw_acceleration=delta * blend_dd,
                    horizontal_speed_mps=0.0,
                    horizontal_accel_mps2=0.0,
                )
            )
        self.publish_reference_sequence(reference, phase)

        stable_since: float | None = None
        deadline = time.monotonic() + self.args.align_timeout_s
        target_point = reference[-1]
        while rclpy.ok() and time.monotonic() < deadline:
            self.current_inputs_safe(require_cmd_ctrl=True)
            self.traj_pub.publish(self.build_position_command(target_point))
            self.set_phase(phase)
            yaw_error = abs(normalize_angle(target_yaw - self.actual_yaw()))
            if yaw_error <= math.radians(self.args.align_tolerance_deg):
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= self.args.align_settle_s:
                    return
            else:
                stable_since = None
            time.sleep(1.0 / self.args.rate_hz)
        raise FlightAbort("yaw did not settle at the B-spline tangent")

    def report_plan(
        self,
        path: FigureEightPath,
        entry: EntryPoint,
        transfer_in: list[ReferencePoint] | None,
        continuous_laps: list[ReferencePoint],
    ) -> None:
        bounds = path.bounds()
        nearest_margin = min(
            distance_to_polygon_edges(
                path.sample(path.length_m * index / self.args.validation_samples).position[:2],
                self.flight_polygon,
            )
            for index in range(self.args.validation_samples)
        )
        self.get_logger().info(
            "B-spline plan: "
            f"length={path.length_m:.2f}m bounds=x[{bounds[0]:.2f},{bounds[1]:.2f}] "
            f"y[{bounds[2]:.2f},{bounds[3]:.2f}] z=[{self.args.z_min_m:.2f},{self.args.z_max_m:.2f}] "
            f"min_polygon_margin={nearest_margin:.2f}m"
        )
        transfer_time_text = (
            f"{transfer_in[-1].time_s:.2f}s"
            if transfer_in
            else "pending hover position"
        )
        self.get_logger().info(
            f"entry: p=({entry.position[0]:.2f},{entry.position[1]:.2f},{entry.position[2]:.2f}) "
            f"distance={entry.distance_m:.2f}m yaw={math.degrees(entry.yaw):.1f}deg "
            f"transfer_time={transfer_time_text}"
        )
        peak_tilt_deg = 0.0
        for index in range(len(self.args.lap_speeds)):
            reference = [point for point in continuous_laps if point.lap_index == index]
            if not reference:
                raise FlightAbort(f"lap {index + 1} has no planned samples")
            max_speed = max(point.horizontal_speed_mps for point in reference)
            max_accel = max(norm(point.acceleration) for point in reference)
            max_yaw_rate = max(abs(point.yaw_rate) for point in reference)
            max_jerk = max(norm(point.jerk) for point in reference)
            max_snap = max(norm(point.snap) for point in reference)
            flat_inputs = [
                flat_input
                for point in reference
                if (
                    flat_input := flat_body_rate_and_acceleration(
                        point.acceleration,
                        point.jerk,
                        point.snap,
                        point.yaw,
                        point.yaw_rate,
                        point.yaw_acceleration,
                    )
                ) is not None
            ]
            if len(flat_inputs) != len(reference):
                raise FlightAbort(f"lap {index + 1} contains an invalid analytic flatness input")
            max_bodyrate_ff = max(norm(flat_input[0]) for flat_input in flat_inputs)
            max_bodyrate_dot_ff = tuple(
                max(abs(flat_input[1][axis]) for flat_input in flat_inputs)
                for axis in range(3)
            )
            lap_tilt_deg = max(
                math.degrees(
                    math.atan2(
                        math.hypot(point.acceleration[0], point.acceleration[1]),
                        max(1e-3, 9.81 + point.acceleration[2]),
                    )
                )
                for point in reference
            )
            peak_tilt_deg = max(peak_tilt_deg, lap_tilt_deg)
            self.get_logger().info(
                f"lap {index + 1}: requested={self.args.lap_speeds[index]:.2f}m/s "
                f"duration={reference[-1].time_s - reference[0].time_s:.2f}s "
                f"boundary_speed=[{reference[0].horizontal_speed_mps:.2f},"
                f"{reference[-1].horizontal_speed_mps:.2f}]m/s peak_speed={max_speed:.2f}m/s "
                f"peak_accel={max_accel:.2f}m/s2 peak_yaw_rate={max_yaw_rate:.2f}rad/s "
                f"sampled_peak_jerk={max_jerk:.2f}m/s3 peak_tilt={lap_tilt_deg:.1f}deg"
                f" sampled_peak_snap={max_snap:.2f}m/s4 "
                f"bodyrate_ff_raw/transmitted={max_bodyrate_ff:.2f}/"
                f"{self.args.body_rate_ff_scale * max_bodyrate_ff:.2f}rad/s "
                f"bodyrate_dot_ff_raw=({max_bodyrate_dot_ff[0]:.1f},"
                f"{max_bodyrate_dot_ff[1]:.1f},{max_bodyrate_dot_ff[2]:.1f})rad/s2"
            )
        if peak_tilt_deg > self.runtime_controller_max_angle_deg - 0.5:
            raise FlightAbort(
                f"planned tilt {peak_tilt_deg:.1f}deg leaves insufficient margin below "
                f"controller.max_angle_deg={self.runtime_controller_max_angle_deg:.1f}deg"
            )

    def prepare_main_trajectory(
        self,
    ) -> tuple[FigureEightPath, EntryPoint, list[ReferencePoint]]:
        planning_started = time.monotonic()
        path = self.build_path()
        self.validate_main_path(path)
        preflight_position = self.actual_position()
        preflight_yaw = self.actual_yaw()
        if not point_in_convex_polygon(preflight_position[:2], self.flight_polygon):
            raise FlightAbort(
                f"preflight position is outside the flight polygon: {preflight_position}"
            )
        entry = path.nearest_entry(preflight_position, preflight_yaw)
        limits = MotionLimits(
            self.args.max_long_accel_mps2,
            self.args.max_decel_mps2,
            self.args.max_lateral_accel_mps2,
            self.args.max_jerk_mps3,
            self.args.max_yaw_rate_radps,
            self.args.max_snap_mps4,
            self.args.max_bodyrate_ff_radps,
            self.args.max_bodyrate_dot_ff_radps2,
            math.radians(self.runtime_controller_max_angle_deg - 0.5),
        )
        continuous_laps = generate_multi_lap_reference(
            path,
            entry_arc_m=entry.arc_length_m,
            lap_speeds_mps=self.args.lap_speeds,
            rate_hz=self.args.rate_hz,
            limits=limits,
            jerk_smoothing_s=self.args.jerk_smoothing_s,
        )
        self.report_plan(path, entry, None, continuous_laps)
        self.get_logger().info(
            f"Main B-spline reference precomputed while disarmed in "
            f"{time.monotonic() - planning_started:.2f}s; "
            f"{len(continuous_laps)} samples are cached for flight."
        )
        return path, entry, continuous_laps

    def prepare_flight(
        self,
        cached_main: tuple[FigureEightPath, EntryPoint, list[ReferencePoint]],
    ):
        path, entry, continuous_laps = cached_main
        transfer_started = time.monotonic()
        launch_position = self.actual_position()
        launch_yaw = self.actual_yaw()
        if not point_in_convex_polygon(launch_position[:2], self.flight_polygon):
            raise FlightAbort(f"takeoff hover point is outside the flight polygon: {launch_position}")
        transfer_in = generate_transfer_reference(
            launch_position,
            entry.position,
            yaw=launch_yaw,
            max_speed_mps=self.args.transfer_speed_mps,
            max_accel_mps2=self.args.transfer_accel_mps2,
            rate_hz=self.args.rate_hz,
        )
        transfer_out = generate_transfer_reference(
            entry.position,
            launch_position,
            yaw=entry.yaw,
            max_speed_mps=self.args.transfer_speed_mps,
            max_accel_mps2=self.args.transfer_accel_mps2,
            rate_hz=self.args.rate_hz,
        )
        self.validate_transfer(transfer_in, "inbound")
        self.validate_transfer(transfer_out, "outbound")
        self.get_logger().info(
            f"Reused cached main B-spline; airborne transfer planning took "
            f"{time.monotonic() - transfer_started:.3f}s."
        )
        return path, entry, launch_position, launch_yaw, transfer_in, transfer_out, continuous_laps

    def wait_for_auto_hover_after_commands(self) -> bool:
        self.set_phase("WAIT_AUTO_HOVER")
        deadline = time.monotonic() + self.args.cmd_return_timeout
        stable_since: float | None = None
        while rclpy.ok() and time.monotonic() < deadline:
            if self.fsm_state == "AUTO_HOVER" and self.is_fresh("actual_state", self.args.actual_timeout_s):
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= self.args.hover_settle_s:
                    return True
            else:
                stable_since = None
            time.sleep(0.05)
        return False

    def recover_and_land(self, reason: str) -> bool:
        self.get_logger().error(f"Trajectory aborted: {reason}")
        self.set_phase("ABORT")
        if not self.is_fresh("actual_state", self.args.actual_timeout_s):
            self.get_logger().error("Cannot generate recovery hold because combined state is stale.")
            return False
        if self.fsm_state == "CMD_CTRL":
            position = self.actual_position()
            yaw = self.actual_yaw()
            try:
                self.hold_reference(
                    position,
                    yaw,
                    self.args.abort_hold_s,
                    "ABORT_HOLD",
                    require_cmd_ctrl=True,
                    enforce_position_bounds=False,
                )
            except FlightAbort as exc:
                self.get_logger().error(f"Recovery hold failed: {exc}")
                return False
        if self.fsm_state != "AUTO_HOVER" and not self.wait_for_auto_hover_after_commands():
            self.get_logger().error("AUTO_HOVER recovery was not confirmed; use RC to take over and land.")
            return False
        self.set_phase("AUTO_LAND")
        self.publish_land()
        return self.wait_for_safe_landing()

    def analyze_recorded_bag(self) -> bool:
        physical_ok = super().analyze_recorded_bag()
        if not self.args.analyze_bag or self.bag_output is None:
            return physical_ok
        analyzer = Path(__file__).resolve().with_name("analyze_bspline_tracking_bag.py")
        self.get_logger().info(f"Analyzing B-spline tracking errors: {self.bag_output}")
        result = subprocess.run(
            [
                sys.executable,
                str(analyzer),
                str(self.bag_output),
                "--storage-id",
                self.args.bag_storage,
                "--expected-lap-speeds",
                ",".join(f"{speed:.9g}" for speed in self.args.lap_speeds),
            ],
            check=False,
        )
        return physical_ok and result.returncode == 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = build_base_arg_parser()
    parser.description = "Automatic 3D periodic B-spline figure-eight tracking test."
    parser.set_defaults(
        node_name="bspline_tracking_test",
        bag_prefix="bspline_tracking",
        publish_takeoff=True,
        require_composed_state=True,
    )
    parser.add_argument("--traj-topic", default="/position_cmd_traj")
    parser.add_argument("--actual-topic", default="/px4ctrl/simulink/actual_state")
    parser.add_argument("--tracking-error-topic", default="/px4ctrl/simulink/tracking_error")
    parser.add_argument("--phase-topic", default="/px4ctrl/bspline_test/state")
    parser.add_argument("--battery-topic", default="/mavros/battery")
    parser.add_argument(
        "--expected-actual-frame",
        default="actual_state_mocap_synced_pv_odom_attitude",
    )
    parser.add_argument("--boundary-point", action="append", type=parse_boundary_point, default=[])
    parser.add_argument("--x-span-m", type=float, default=16.7)
    parser.add_argument("--y-span-m", type=float, default=3.8)
    parser.add_argument("--center-x-offset-m", type=float, default=0.0)
    parser.add_argument("--center-y-offset-m", type=float, default=0.0)
    parser.add_argument("--heading-offset-deg", type=float, default=0.0)
    parser.add_argument("--z-min-m", type=float, default=1.0)
    parser.add_argument("--z-max-m", type=float, default=1.5)
    parser.add_argument("--control-count", type=int, default=24)
    parser.add_argument("--arc-samples", type=int, default=4000)
    parser.add_argument("--validation-samples", type=int, default=2000)
    parser.add_argument("--safety-margin-m", type=float, default=0.20)
    parser.add_argument("--lap-speeds", type=parse_lap_speeds, default=(8.0, 8.0, 8.0))
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--transfer-speed-mps", type=float, default=1.0)
    parser.add_argument("--transfer-accel-mps2", type=float, default=1.0)
    parser.add_argument("--max-long-accel-mps2", type=float, default=6.0)
    parser.add_argument("--max-decel-mps2", type=float, default=8.0)
    parser.add_argument("--max-lateral-accel-mps2", type=float, default=15.0)
    parser.add_argument("--max-jerk-mps3", type=float, default=160.0)
    parser.add_argument("--max-snap-mps4", type=float, default=5000.0)
    parser.add_argument("--max-yaw-rate-radps", type=float, default=5.0)
    parser.add_argument("--max-bodyrate-ff-radps", type=float, default=10.0)
    parser.add_argument(
        "--max-bodyrate-dot-ff-radps2",
        type=parse_lap_speeds,
        default=(100.0, 100.0, 50.0),
    )
    parser.add_argument("--jerk-smoothing-s", type=float, default=0.05)
    parser.add_argument("--controller-max-angle-deg", type=float, default=60.0)
    parser.add_argument("--controller-max-bodyrates", type=parse_lap_speeds, default=(8.0, 8.0, 8.0))
    parser.add_argument("--body-rate-ff-scale", type=float, default=1.0)
    parser.add_argument("--angular-accel-ff-scale", type=float, default=1.0)
    parser.add_argument("--ude-max-f-hat", type=parse_lap_speeds, default=(20.0, 20.0, 10.0))
    parser.add_argument("--ude-max-u-acc", type=parse_lap_speeds, default=(25.0, 25.0, 15.0))
    parser.add_argument("--final-hold-s", type=float, default=2.0)
    parser.add_argument("--align-yaw-rate-radps", type=float, default=0.5)
    parser.add_argument("--align-tolerance-deg", type=float, default=5.0)
    parser.add_argument("--align-settle-s", type=float, default=0.5)
    parser.add_argument("--align-timeout-s", type=float, default=10.0)
    parser.add_argument("--actual-timeout-s", type=float, default=0.25)
    parser.add_argument("--minimum-feedback-rate-hz", type=float, default=80.0)
    parser.add_argument("--feedback-rate-window-s", type=float, default=2.0)
    parser.add_argument("--battery-timeout-s", type=float, default=5.0)
    parser.add_argument("--battery-min-voltage-v", type=float, default=18.0)
    parser.add_argument("--battery-max-voltage-v", type=float, default=26.0)
    parser.add_argument("--max-publish-lateness-s", type=float, default=0.08)
    parser.add_argument("--max-xy-error-m", type=float, default=1.5)
    parser.add_argument("--max-z-error-m", type=float, default=0.6)
    parser.add_argument("--error-grace-s", type=float, default=1.0)
    parser.add_argument("--hover-settle-s", type=float, default=1.0)
    parser.add_argument("--abort-hold-s", type=float, default=1.0)
    return parser


def validate_arguments(args: argparse.Namespace) -> None:
    if not args.boundary_point:
        args.boundary_point = list(DEFAULT_BOUNDARY)
    if len(args.boundary_point) != 4:
        raise ValueError("exactly four --boundary-point values are required")
    for name in (
        "controller_max_bodyrates",
        "max_bodyrate_dot_ff_radps2",
        "ude_max_f_hat",
        "ude_max_u_acc",
    ):
        if len(getattr(args, name)) != 3:
            raise ValueError(f"--{name.replace('_', '-')} must contain exactly three values")
    positive_names = (
        "x_span_m",
        "y_span_m",
        "rate_hz",
        "transfer_speed_mps",
        "transfer_accel_mps2",
        "max_long_accel_mps2",
        "max_decel_mps2",
        "max_lateral_accel_mps2",
        "max_jerk_mps3",
        "max_snap_mps4",
        "max_yaw_rate_radps",
        "max_bodyrate_ff_radps",
        "controller_max_angle_deg",
        "align_yaw_rate_radps",
        "actual_timeout_s",
        "minimum_feedback_rate_hz",
        "feedback_rate_window_s",
        "battery_timeout_s",
    )
    for name in positive_names:
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"--{name.replace('_', '-')} must be positive")
    if args.z_max_m <= args.z_min_m:
        raise ValueError("--z-max-m must be greater than --z-min-m")
    if args.battery_max_voltage_v <= args.battery_min_voltage_v:
        raise ValueError("--battery-max-voltage-v must exceed --battery-min-voltage-v")
    if not math.isfinite(args.jerk_smoothing_s) or args.jerk_smoothing_s < 0.0:
        raise ValueError("--jerk-smoothing-s must be finite and non-negative")
    if not math.isfinite(args.angular_accel_ff_scale) or not (
        0.0 <= args.angular_accel_ff_scale <= 1.0
    ):
        raise ValueError("--angular-accel-ff-scale must be in [0, 1]")
    if not math.isfinite(args.body_rate_ff_scale) or not (
        0.0 <= args.body_rate_ff_scale <= 1.0
    ):
        raise ValueError("--body-rate-ff-scale must be in [0, 1]")


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv if argv is None else argv
    parser = build_arg_parser()
    args = parser.parse_args(remove_ros_args(args=argv)[1:])
    validate_arguments(args)
    for topic in (args.traj_topic, args.phase_topic):
        if topic not in args.bag_topic:
            args.bag_topic.append(topic)

    rclpy.init(args=argv)
    node = BsplineTrackingTest(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    def spin_executor() -> None:
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=spin_executor, daemon=True)
    spin_thread.start()

    try:
        if not node.wait_for_startup() or not node.load_and_verify_runtime_parameters():
            return 1
        node.print_status()
        if node.fsm_state != "MANUAL_CTRL":
            node.get_logger().error(
                f"Expected MANUAL_CTRL before takeoff, got {node.fsm_state}; stop old command publishers."
            )
            return 1

        cached_main = node.prepare_main_trajectory()
        print(
            "\nBefore takeoff: confirm MC_PA_MODE=2, MC_TQ_CTRL_EN=1, CH5/CH6 are in "
            "hover+command position, sticks are centered, and the complete polygon is clear."
        )
        if args.auto_confirm:
            print("--auto-confirm is set; publishing TAKEOFF.")
        else:
            input("Press Enter to start AUTO_TAKEOFF and the automatic B-spline test: ")
        if not node.start_bag_recording():
            raise FlightAbort("rosbag failed to start")
        node.set_phase("AUTO_TAKEOFF")
        node.publish_takeoff()
        if not node.wait_for_auto_hover():
            raise FlightAbort("AUTO_TAKEOFF did not reach AUTO_HOVER")

        prepared = node.prepare_flight(cached_main)
        _, entry, launch_position, launch_yaw, transfer_in, transfer_out, continuous_laps = prepared
        if args.auto_confirm:
            node.get_logger().info("--auto-confirm is set; starting the planned trajectory.")
        else:
            input(
                "The vehicle is holding in AUTO_HOVER. Review the plan above, then press Enter "
                "to enter CMD_CTRL and start the trajectory: "
            )
        node.enter_cmd_ctrl(launch_position, launch_yaw)
        if entry.distance_m > 0.02:
            node.align_yaw(launch_position, transfer_in[0].yaw, "TRANSFER_IN_ALIGN")
            node.publish_reference_sequence(transfer_in, "TRANSFER_IN")
        node.align_yaw(entry.position, entry.yaw, "MAIN_TANGENT_ALIGN")

        node.publish_continuous_laps(continuous_laps)

        node.hold_reference(entry.position, entry.yaw, args.final_hold_s, "FINAL_ENTRY_HOLD")
        if entry.distance_m > 0.02:
            node.align_yaw(entry.position, transfer_out[0].yaw, "TRANSFER_OUT_ALIGN")
            node.publish_reference_sequence(transfer_out, "TRANSFER_OUT")
        node.hold_reference(launch_position, transfer_out[-1].yaw, args.final_hold_s, "FINAL_HOME_HOLD")

        if not node.wait_for_auto_hover_after_commands():
            raise FlightAbort("CMD_CTRL did not return to stable AUTO_HOVER after commands stopped")
        node.set_phase("AUTO_LAND")
        node.publish_land()
        if not node.wait_for_safe_landing():
            raise FlightAbort("AUTO_LAND did not disarm before timeout")
        node.set_phase("COMPLETE")
        return 0
    except (FlightAbort, RuntimeError, ValueError) as exc:
        if node.safe_to_clean_stack():
            node.get_logger().error(str(exc))
            return 1
        recovered = node.recover_and_land(str(exc))
        return 1 if recovered else MAYBE_AIRBORNE_EXIT_CODE
    except KeyboardInterrupt:
        if node.safe_to_clean_stack():
            return 0
        recovered = node.recover_and_land("operator interrupted the automatic test")
        return 130 if recovered else MAYBE_AIRBORNE_EXIT_CODE
    except EOFError:
        if node.safe_to_clean_stack():
            node.get_logger().error("Terminal input closed while waiting for operator confirmation")
            return 1
        recovered = node.recover_and_land(
            "terminal input closed while waiting for operator confirmation"
        )
        return 1 if recovered else MAYBE_AIRBORNE_EXIT_CODE
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
