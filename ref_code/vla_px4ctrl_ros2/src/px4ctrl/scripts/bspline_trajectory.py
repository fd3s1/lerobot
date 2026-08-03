#!/usr/bin/python3

from __future__ import annotations

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass, replace
import math
from typing import Iterable, Sequence


Vec3 = tuple[float, float, float]


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Vec3, value: float) -> Vec3:
    return (a[0] * value, a[1] * value, a[2] * value)


def norm(a: Vec3) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm_xy(a: Vec3) -> float:
    return math.hypot(a[0], a[1])


def lerp(a: Vec3, b: Vec3, fraction: float) -> Vec3:
    return add(a, scale(sub(b, a), fraction))


def smootherstep9_kinematics(
    normalized_time: float,
    duration_s: float,
) -> tuple[float, float, float, float, float]:
    u = clamp(normalized_time, 0.0, 1.0)
    duration = max(duration_s, 1e-9)
    value = (
        126.0 * u**5 - 420.0 * u**6 + 540.0 * u**7
        - 315.0 * u**8 + 70.0 * u**9
    )
    first = (
        630.0 * u**4 - 2520.0 * u**5 + 3780.0 * u**6
        - 2520.0 * u**7 + 630.0 * u**8
    ) / duration
    second = (
        2520.0 * u**3 - 12600.0 * u**4 + 22680.0 * u**5
        - 17640.0 * u**6 + 5040.0 * u**7
    ) / (duration**2)
    third = (
        7560.0 * u**2 - 50400.0 * u**3 + 113400.0 * u**4
        - 105840.0 * u**5 + 35280.0 * u**6
    ) / (duration**3)
    fourth = (
        15120.0 * u - 151200.0 * u**2 + 453600.0 * u**3
        - 529200.0 * u**4 + 211680.0 * u**5
    ) / (duration**4)
    return value, first, second, third, fourth


def smootherstep7_velocity_kinematics(
    normalized_time: float,
    duration_s: float,
) -> tuple[float, float, float, float, float]:
    """Return velocity blend, its three time derivatives, and its time integral."""
    u = clamp(normalized_time, 0.0, 1.0)
    duration = max(duration_s, 1e-9)
    value = 35.0 * u**4 - 84.0 * u**5 + 70.0 * u**6 - 20.0 * u**7
    first = (
        140.0 * u**3 - 420.0 * u**4 + 420.0 * u**5 - 140.0 * u**6
    ) / duration
    second = (
        420.0 * u**2 - 1680.0 * u**3 + 2100.0 * u**4 - 840.0 * u**5
    ) / (duration**2)
    third = (
        840.0 * u - 5040.0 * u**2 + 8400.0 * u**3 - 4200.0 * u**4
    ) / (duration**3)
    integral = duration * (
        7.0 * u**5 - 14.0 * u**6 + 10.0 * u**7 - 2.5 * u**8
    )
    return value, first, second, third, integral


def warped_smootherstep7_velocity_kinematics(
    normalized_time: float,
    duration_s: float,
    power: int,
) -> tuple[float, float, float, float, float]:
    """Seventh-order velocity blend evaluated at u**power."""
    if power < 1:
        raise ValueError("velocity blend warp power must be positive")
    if power == 1:
        return smootherstep7_velocity_kinematics(normalized_time, duration_s)

    u = clamp(normalized_time, 0.0, 1.0)
    duration = max(duration_s, 1e-9)
    terms = (
        (35.0, 4 * power),
        (-84.0, 5 * power),
        (70.0, 6 * power),
        (-20.0, 7 * power),
    )
    value = sum(coefficient * u**exponent for coefficient, exponent in terms)
    first = sum(
        coefficient * exponent * u ** (exponent - 1)
        for coefficient, exponent in terms
    ) / duration
    second = sum(
        coefficient * exponent * (exponent - 1) * u ** (exponent - 2)
        for coefficient, exponent in terms
    ) / duration**2
    third = sum(
        coefficient * exponent * (exponent - 1) * (exponent - 2)
        * u ** (exponent - 3)
        for coefficient, exponent in terms
    ) / duration**3
    integral = duration * sum(
        coefficient * u ** (exponent + 1) / (exponent + 1)
        for coefficient, exponent in terms
    )
    return value, first, second, third, integral


@dataclass(frozen=True)
class SplineSample:
    position: Vec3
    derivative: Vec3
    second_derivative: Vec3
    third_derivative: Vec3
    fourth_derivative: Vec3


class PeriodicUniformBSpline:
    def __init__(self, controls: Sequence[Vec3], degree: int = 5) -> None:
        if degree < 3 or degree > 5:
            raise ValueError("periodic B-spline degree must be between three and five")
        if len(controls) < degree + 1:
            raise ValueError("a periodic B-spline needs at least degree + 1 controls")
        self.controls = tuple(controls)
        self.degree = degree
        self.segment_count = len(self.controls)

    def evaluate(self, parameter: float) -> SplineSample:
        wrapped = parameter % self.segment_count
        segment = int(math.floor(wrapped))

        # For a cardinal spline only one degree-zero basis is active. Building
        # the degree table iteratively avoids thousands of small recursive
        # calls while retaining the exact Cox-de Boor result.
        basis_by_degree: list[dict[int, float]] = [{segment: 1.0}]
        for degree in range(1, self.degree + 1):
            previous = basis_by_degree[-1]
            current: dict[int, float] = {}
            for control_index in range(segment - degree, segment + 1):
                current[control_index] = (
                    (wrapped - control_index)
                    / degree
                    * previous.get(control_index, 0.0)
                    + (control_index + degree + 1 - wrapped)
                    / degree
                    * previous.get(control_index + 1, 0.0)
                )
            basis_by_degree.append(current)

        def basis_derivative(control_index: int, order: int) -> float:
            lower_degree = self.degree - order
            return sum(
                (-1.0) ** offset
                * math.comb(order, offset)
                * basis_by_degree[lower_degree].get(control_index + offset, 0.0)
                for offset in range(order + 1)
            )

        control_indices = range(segment - self.degree, segment + 1)

        def evaluate_order(order: int) -> Vec3:
            return tuple(
                sum(
                    basis_derivative(control_index, order)
                    * self.controls[control_index % self.segment_count][axis]
                    for control_index in control_indices
                )
                for axis in range(3)
            )  # type: ignore[return-value]

        return SplineSample(
            position=evaluate_order(0),
            derivative=evaluate_order(1),
            second_derivative=evaluate_order(2),
            third_derivative=evaluate_order(3),
            fourth_derivative=evaluate_order(4),
        )


@dataclass(frozen=True)
class PathSample:
    position: Vec3
    tangent_xy: tuple[float, float]
    curvature_xy: float
    curvature_derivative_xy: float
    curvature_second_derivative_xy: float
    z_slope: float
    z_curvature: float
    z_third_derivative: float
    z_fourth_derivative: float
    yaw: float


@dataclass(frozen=True)
class EntryPoint:
    arc_length_m: float
    parameter: float
    position: Vec3
    yaw: float
    distance_m: float
    curvature_abs: float


class FigureEightPath:
    def __init__(
        self,
        *,
        center_xy: tuple[float, float],
        x_span_m: float,
        y_span_m: float,
        heading_rad: float,
        z_min_m: float,
        z_max_m: float,
        control_count: int = 24,
        arc_samples: int = 4000,
    ) -> None:
        if x_span_m <= 0.0 or y_span_m <= 0.0:
            raise ValueError("figure-eight spans must be positive")
        if z_max_m < z_min_m:
            raise ValueError("z_max_m must be greater than or equal to z_min_m")
        if control_count < 8:
            raise ValueError("control_count must be at least eight")
        if arc_samples < control_count * 20:
            raise ValueError("arc_samples is too small")

        self.center_xy = center_xy
        self.x_span_m = x_span_m
        self.y_span_m = y_span_m
        self.heading_rad = heading_rad
        self.z_min_m = z_min_m
        self.z_max_m = z_max_m
        self.control_count = control_count

        controls = self._make_scaled_controls()
        self.spline = PeriodicUniformBSpline(controls)
        self._arc_parameters, self._arc_lengths = self._build_arc_table(arc_samples)
        self.length_m = self._arc_lengths[-1]
        if self.length_m <= 1e-6:
            raise ValueError("figure-eight has zero horizontal length")

    def _raw_controls(self, x_scale: float, y_scale: float) -> list[Vec3]:
        cos_h = math.cos(self.heading_rad)
        sin_h = math.sin(self.heading_rad)
        controls: list[Vec3] = []
        for index in range(self.control_count):
            theta = 2.0 * math.pi * index / self.control_count
            local_x = x_scale * math.sin(theta)
            local_y = y_scale * math.sin(2.0 * theta)
            x = self.center_xy[0] + cos_h * local_x - sin_h * local_y
            y = self.center_xy[1] + sin_h * local_x + cos_h * local_y
            controls.append((x, y, 0.0))
        return controls

    def _make_scaled_controls(self) -> list[Vec3]:
        controls: list[Vec3] = []
        for index in range(self.control_count):
            theta = 2.0 * math.pi * index / self.control_count
            controls.append((math.sin(theta), math.sin(2.0 * theta), 0.0))
        preliminary = PeriodicUniformBSpline(controls)
        samples = [
            preliminary.evaluate(
                preliminary.segment_count * index / (preliminary.segment_count * 100)
            ).position
            for index in range(preliminary.segment_count * 100)
        ]
        x_extent = max(point[0] for point in samples) - min(point[0] for point in samples)
        y_extent = max(point[1] for point in samples) - min(point[1] for point in samples)
        if x_extent <= 1e-6 or y_extent <= 1e-6:
            raise ValueError("invalid figure-eight control geometry")
        return self._raw_controls(self.x_span_m / x_extent, self.y_span_m / y_extent)

    def _build_arc_table(self, sample_count: int) -> tuple[list[float], list[float]]:
        parameters = [
            self.control_count * index / sample_count for index in range(sample_count + 1)
        ]
        lengths = [0.0]
        previous = self.spline.evaluate(parameters[0]).position
        for parameter in parameters[1:]:
            current = self.spline.evaluate(parameter).position
            lengths.append(lengths[-1] + math.hypot(current[0] - previous[0], current[1] - previous[1]))
            previous = current
        return parameters, lengths

    def arc_from_parameter(self, parameter: float) -> float:
        wrapped = parameter % self.control_count
        scaled = wrapped / self.control_count * (len(self._arc_parameters) - 1)
        index = min(len(self._arc_parameters) - 2, max(0, int(math.floor(scaled))))
        fraction = scaled - index
        return (
            self._arc_lengths[index]
            + fraction * (self._arc_lengths[index + 1] - self._arc_lengths[index])
        )

    def parameter_from_arc(self, arc_length_m: float) -> float:
        wrapped = arc_length_m % self.length_m
        index = bisect_left(self._arc_lengths, wrapped)
        index = min(len(self._arc_lengths) - 1, max(1, index))
        low_length = self._arc_lengths[index - 1]
        high_length = self._arc_lengths[index]
        fraction = 0.0 if high_length <= low_length else (wrapped - low_length) / (high_length - low_length)
        return self._arc_parameters[index - 1] + fraction * (
            self._arc_parameters[index] - self._arc_parameters[index - 1]
        )

    def sample(self, arc_length_m: float, entry_arc_m: float = 0.0) -> PathSample:
        absolute_arc = (entry_arc_m + arc_length_m) % self.length_m
        parameter = self.parameter_from_arc(absolute_arc)
        spline_sample = self.spline.evaluate(parameter)
        dx, dy = spline_sample.derivative[0], spline_sample.derivative[1]
        ddx, ddy = spline_sample.second_derivative[0], spline_sample.second_derivative[1]
        dddx, dddy = spline_sample.third_derivative[0], spline_sample.third_derivative[1]
        ddddx, ddddy = spline_sample.fourth_derivative[0], spline_sample.fourth_derivative[1]
        speed_parameter = math.hypot(dx, dy)
        if speed_parameter <= 1e-9:
            raise ValueError("figure-eight has a singular horizontal tangent")
        tangent = (dx / speed_parameter, dy / speed_parameter)
        cross_first_second = dx * ddy - dy * ddx
        curvature = cross_first_second / (speed_parameter**3)
        curvature_parameter_derivative = (
            (dx * dddy - dy * dddx) / (speed_parameter**3)
            - 3.0
            * cross_first_second
            * (dx * ddx + dy * ddy)
            / (speed_parameter**5)
        )
        curvature_arc_derivative = curvature_parameter_derivative / speed_parameter
        cross_first_third = dx * dddy - dy * dddx
        cross_second_third = ddx * dddy - ddy * dddx
        cross_first_fourth = dx * ddddy - dy * ddddx
        tangent_accel_dot = dx * ddx + dy * ddy
        tangent_accel_dot_derivative = (
            ddx * ddx + ddy * ddy + dx * dddx + dy * dddy
        )
        curvature_parameter_second_derivative = (
            (cross_second_third + cross_first_fourth) / (speed_parameter**3)
            - (
                6.0 * cross_first_third * tangent_accel_dot
                + 3.0 * cross_first_second * tangent_accel_dot_derivative
            )
            / (speed_parameter**5)
            + 15.0
            * cross_first_second
            * tangent_accel_dot
            * tangent_accel_dot
            / (speed_parameter**7)
        )
        curvature_arc_second_derivative = (
            curvature_parameter_second_derivative / (speed_parameter**2)
            - curvature_parameter_derivative
            * tangent_accel_dot
            / (speed_parameter**4)
        )

        phase = (arc_length_m % self.length_m) / self.length_m
        z_range = self.z_max_m - self.z_min_m
        phase_angle = 2.0 * math.pi * phase
        z = self.z_min_m + 0.5 * z_range * (1.0 - math.cos(phase_angle))
        z_slope = z_range * math.pi * math.sin(phase_angle) / self.length_m
        z_curvature = 2.0 * z_range * math.pi * math.pi * math.cos(phase_angle) / (
            self.length_m * self.length_m
        )
        z_third_derivative = -4.0 * z_range * math.pi**3 * math.sin(phase_angle) / (
            self.length_m**3
        )
        z_fourth_derivative = -8.0 * z_range * math.pi**4 * math.cos(phase_angle) / (
            self.length_m**4
        )
        return PathSample(
            position=(spline_sample.position[0], spline_sample.position[1], z),
            tangent_xy=tangent,
            curvature_xy=curvature,
            curvature_derivative_xy=curvature_arc_derivative,
            curvature_second_derivative_xy=curvature_arc_second_derivative,
            z_slope=z_slope,
            z_curvature=z_curvature,
            z_third_derivative=z_third_derivative,
            z_fourth_derivative=z_fourth_derivative,
            yaw=math.atan2(tangent[1], tangent[0]),
        )

    def bounds(self, sample_count: int = 4000) -> tuple[float, float, float, float]:
        points = [
            self.sample(self.length_m * index / sample_count).position
            for index in range(sample_count)
        ]
        return (
            min(point[0] for point in points),
            max(point[0] for point in points),
            min(point[1] for point in points),
            max(point[1] for point in points),
        )

    def nearest_entry(
        self,
        position: Vec3,
        current_yaw: float,
        *,
        sample_count: int = 2000,
        distance_window_m: float = 0.25,
        yaw_weight_m_per_rad: float = 0.15,
        curvature_weight_m2: float = 0.10,
    ) -> EntryPoint:
        candidates: list[tuple[float, float, PathSample]] = []
        minimum_distance = math.inf
        for index in range(sample_count):
            arc = self.length_m * index / sample_count
            sample = self.sample(0.0, arc)
            distance = math.hypot(sample.position[0] - position[0], sample.position[1] - position[1])
            candidates.append((distance, arc, sample))
            minimum_distance = min(minimum_distance, distance)

        shortlist = [item for item in candidates if item[0] <= minimum_distance + distance_window_m]
        _, best_arc, _ = min(
            shortlist,
            key=lambda item: (
                item[0]
                + yaw_weight_m_per_rad * abs(normalize_angle(item[2].yaw - current_yaw))
                + curvature_weight_m2 * abs(item[2].curvature_xy)
            ),
        )

        spacing = self.length_m / sample_count
        left = best_arc - spacing
        right = best_arc + spacing
        for _ in range(32):
            first = left + (right - left) / 3.0
            second = right - (right - left) / 3.0
            first_sample = self.sample(0.0, first)
            second_sample = self.sample(0.0, second)
            first_distance = math.hypot(
                first_sample.position[0] - position[0], first_sample.position[1] - position[1]
            )
            second_distance = math.hypot(
                second_sample.position[0] - position[0], second_sample.position[1] - position[1]
            )
            if first_distance <= second_distance:
                right = second
            else:
                left = first
        refined_arc = 0.5 * (left + right)
        refined_sample = self.sample(0.0, refined_arc)
        refined_distance = math.hypot(
            refined_sample.position[0] - position[0], refined_sample.position[1] - position[1]
        )
        return EntryPoint(
            arc_length_m=refined_arc % self.length_m,
            parameter=self.parameter_from_arc(refined_arc),
            position=(refined_sample.position[0], refined_sample.position[1], self.z_min_m),
            yaw=refined_sample.yaw,
            distance_m=refined_distance,
            curvature_abs=abs(refined_sample.curvature_xy),
        )


@dataclass(frozen=True)
class ReferencePoint:
    time_s: float
    position: Vec3
    velocity: Vec3
    acceleration: Vec3
    jerk: Vec3
    snap: Vec3
    yaw: float
    yaw_rate: float
    yaw_acceleration: float
    horizontal_speed_mps: float
    horizontal_accel_mps2: float
    lap_index: int = 0
    arc_progress_m: float = 0.0


@dataclass(frozen=True)
class MotionLimits:
    max_long_accel_mps2: float
    max_decel_mps2: float
    max_lateral_accel_mps2: float
    max_jerk_mps3: float
    max_yaw_rate_radps: float
    max_snap_mps4: float = 5000.0
    max_bodyrate_ff_radps: float = math.inf
    # Keep planning below the PX4 protocol limits of [120, 120, 60] rad/s^2.
    max_bodyrate_dot_ff_radps2: Vec3 = (100.0, 100.0, 50.0)
    max_tilt_rad: float = 0.5 * math.pi


def _path_speed_cap(
    path: FigureEightPath,
    progress_m: float,
    entry_arc_m: float,
    cruise_speed_mps: float,
    limits: MotionLimits,
) -> float:
    sample = path.sample(progress_m, entry_arc_m)
    curvature = abs(sample.curvature_xy)
    cap = cruise_speed_mps
    if curvature > 1e-6:
        cap = min(
            cap,
            math.sqrt(max(0.0, limits.max_lateral_accel_mps2 / curvature)),
            limits.max_yaw_rate_radps / curvature,
        )
    if cap > 0.0 and not _flat_kinematics_within_limits(
        sample,
        cap,
        0.0,
        0.0,
        0.0,
        limits,
    ):
        lower = 0.0
        upper = cap
        for _ in range(24):
            candidate = 0.5 * (lower + upper)
            if _flat_kinematics_within_limits(
                sample,
                candidate,
                0.0,
                0.0,
                0.0,
                limits,
            ):
                lower = candidate
            else:
                upper = candidate
        cap = lower
    # Leave a small interpolation margin because curvature is sampled on a
    # spatial grid while the final reference is sampled on a time grid.
    return max(0.0, 0.999 * cap)


def _path_spatial_derivatives(
    sample: PathSample,
) -> tuple[Vec3, Vec3, Vec3, Vec3]:
    tangent = (sample.tangent_xy[0], sample.tangent_xy[1], sample.z_slope)
    normal_xy = (-sample.tangent_xy[1], sample.tangent_xy[0])
    second = (
        sample.curvature_xy * normal_xy[0],
        sample.curvature_xy * normal_xy[1],
        sample.z_curvature,
    )
    third = (
        -sample.curvature_xy**2 * sample.tangent_xy[0]
        + sample.curvature_derivative_xy * normal_xy[0],
        -sample.curvature_xy**2 * sample.tangent_xy[1]
        + sample.curvature_derivative_xy * normal_xy[1],
        sample.z_third_derivative,
    )
    fourth = (
        -3.0
        * sample.curvature_xy
        * sample.curvature_derivative_xy
        * sample.tangent_xy[0]
        + (sample.curvature_second_derivative_xy - sample.curvature_xy**3)
        * normal_xy[0],
        -3.0
        * sample.curvature_xy
        * sample.curvature_derivative_xy
        * sample.tangent_xy[1]
        + (sample.curvature_second_derivative_xy - sample.curvature_xy**3)
        * normal_xy[1],
        sample.z_fourth_derivative,
    )
    return tangent, second, third, fourth


def _path_time_derivatives(
    sample: PathSample,
    speed: float,
    longitudinal_acceleration: float,
    longitudinal_jerk: float,
    longitudinal_snap: float,
) -> tuple[Vec3, Vec3, Vec3, Vec3]:
    tangent, second, third, fourth = _path_spatial_derivatives(sample)
    velocity = scale(tangent, speed)
    acceleration = add(
        scale(second, speed * speed),
        scale(tangent, longitudinal_acceleration),
    )
    jerk = add(
        add(
            scale(third, speed**3),
            scale(second, 3.0 * speed * longitudinal_acceleration),
        ),
        scale(tangent, longitudinal_jerk),
    )
    snap = add(
        add(
            scale(fourth, speed**4),
            scale(third, 6.0 * speed * speed * longitudinal_acceleration),
        ),
        add(
            scale(
                second,
                3.0 * longitudinal_acceleration**2
                + 4.0 * speed * longitudinal_jerk,
            ),
            scale(tangent, longitudinal_snap),
        ),
    )
    return velocity, acceleration, jerk, snap


def _normalized_with_derivative(
    vector: Vec3,
    derivative: Vec3,
) -> tuple[Vec3, Vec3] | None:
    magnitude = norm(vector)
    if magnitude <= 1e-6 or not math.isfinite(magnitude):
        return None
    unit = scale(vector, 1.0 / magnitude)
    unit_derivative = scale(
        sub(derivative, scale(unit, dot(unit, derivative))),
        1.0 / magnitude,
    )
    if not all(math.isfinite(value) for value in (*unit, *unit_derivative)):
        return None
    return unit, unit_derivative


def _normalized_with_second_derivative(
    vector: Vec3,
    derivative: Vec3,
    second_derivative: Vec3,
) -> tuple[Vec3, Vec3, Vec3] | None:
    magnitude = norm(vector)
    if magnitude <= 1e-6 or not math.isfinite(magnitude):
        return None
    unit = scale(vector, 1.0 / magnitude)
    magnitude_dot = dot(unit, derivative)
    unit_dot = scale(
        sub(derivative, scale(unit, magnitude_dot)),
        1.0 / magnitude,
    )
    unit_ddot = sub(
        sub(
            scale(
                sub(second_derivative, scale(unit, dot(unit, second_derivative))),
                1.0 / magnitude,
            ),
            scale(unit_dot, 2.0 * magnitude_dot / magnitude),
        ),
        scale(unit, dot(unit_dot, unit_dot)),
    )
    if not all(math.isfinite(value) for value in (*unit, *unit_dot, *unit_ddot)):
        return None
    return unit, unit_dot, unit_ddot


def flat_body_rate_and_acceleration(
    acceleration: Vec3,
    jerk: Vec3,
    snap: Vec3,
    yaw: float,
    yaw_rate: float,
    yaw_acceleration: float,
    gravity_mps2: float = 9.81,
) -> tuple[Vec3, Vec3] | None:
    """Compute desired-frame body rate and angular acceleration analytically."""
    thrust_acceleration = (
        acceleration[0],
        acceleration[1],
        acceleration[2] + gravity_mps2,
    )
    z_axis_result = _normalized_with_second_derivative(
        thrust_acceleration,
        jerk,
        snap,
    )
    if z_axis_result is None:
        return None
    z_axis, z_axis_dot, z_axis_ddot = z_axis_result

    heading = (math.cos(yaw), math.sin(yaw), 0.0)
    heading_dot = (
        -math.sin(yaw) * yaw_rate,
        math.cos(yaw) * yaw_rate,
        0.0,
    )
    heading_ddot = (
        -math.cos(yaw) * yaw_rate * yaw_rate - math.sin(yaw) * yaw_acceleration,
        -math.sin(yaw) * yaw_rate * yaw_rate + math.cos(yaw) * yaw_acceleration,
        0.0,
    )
    y_axis_raw = cross(z_axis, heading)
    y_axis_raw_dot = add(
        cross(z_axis_dot, heading),
        cross(z_axis, heading_dot),
    )
    y_axis_raw_ddot = add(
        add(
            cross(z_axis_ddot, heading),
            scale(cross(z_axis_dot, heading_dot), 2.0),
        ),
        cross(z_axis, heading_ddot),
    )
    y_axis_result = _normalized_with_second_derivative(
        y_axis_raw,
        y_axis_raw_dot,
        y_axis_raw_ddot,
    )
    if y_axis_result is None:
        return None
    y_axis, y_axis_dot, y_axis_ddot = y_axis_result

    x_axis = cross(y_axis, z_axis)
    x_axis_dot = add(
        cross(y_axis_dot, z_axis),
        cross(y_axis, z_axis_dot),
    )
    x_axis_ddot = add(
        add(
            cross(y_axis_ddot, z_axis),
            scale(cross(y_axis_dot, z_axis_dot), 2.0),
        ),
        cross(y_axis, z_axis_ddot),
    )

    rotation = (x_axis, y_axis, z_axis)
    rotation_dot = (x_axis_dot, y_axis_dot, z_axis_dot)
    rotation_ddot = (x_axis_ddot, y_axis_ddot, z_axis_ddot)

    def transpose_product(
        left_columns: tuple[Vec3, Vec3, Vec3],
        right_columns: tuple[Vec3, Vec3, Vec3],
    ) -> tuple[tuple[float, float, float], ...]:
        return tuple(
            tuple(dot(left_columns[row], right_columns[column]) for column in range(3))
            for row in range(3)
        )

    def matrix_product(
        left: tuple[tuple[float, float, float], ...],
        right: tuple[tuple[float, float, float], ...],
    ) -> tuple[tuple[float, float, float], ...]:
        return tuple(
            tuple(
                sum(left[row][inner] * right[inner][column] for inner in range(3))
                for column in range(3)
            )
            for row in range(3)
        )

    omega_raw = transpose_product(rotation, rotation_dot)
    omega_dot_base = transpose_product(rotation, rotation_ddot)
    omega_squared = matrix_product(omega_raw, omega_raw)
    omega_dot_raw = tuple(
        tuple(omega_dot_base[row][column] - omega_squared[row][column] for column in range(3))
        for row in range(3)
    )

    def skew_vee(matrix: tuple[tuple[float, float, float], ...]) -> Vec3:
        return (
            0.5 * (matrix[2][1] - matrix[1][2]),
            0.5 * (matrix[0][2] - matrix[2][0]),
            0.5 * (matrix[1][0] - matrix[0][1]),
        )

    body_rate = skew_vee(omega_raw)
    body_rate_dot = skew_vee(omega_dot_raw)
    if not all(math.isfinite(value) for value in (*body_rate, *body_rate_dot)):
        return None
    return body_rate, body_rate_dot


def flat_body_rate(
    acceleration: Vec3,
    jerk: Vec3,
    yaw: float,
    yaw_rate: float,
    gravity_mps2: float = 9.81,
) -> Vec3 | None:
    result = flat_body_rate_and_acceleration(
        acceleration,
        jerk,
        (0.0, 0.0, 0.0),
        yaw,
        yaw_rate,
        0.0,
        gravity_mps2,
    )
    return result[0] if result is not None else None


def _flat_kinematics_within_limits(
    sample: PathSample,
    speed: float,
    longitudinal_acceleration: float,
    longitudinal_jerk: float,
    longitudinal_snap: float,
    limits: MotionLimits,
    margin: float = 0.98,
) -> bool:
    _, acceleration, jerk, snap = _path_time_derivatives(
        sample,
        speed,
        longitudinal_acceleration,
        longitudinal_jerk,
        longitudinal_snap,
    )
    vertical_thrust_acceleration = 9.81 + acceleration[2]
    if vertical_thrust_acceleration <= 1e-3:
        return False
    tilt = math.atan2(
        math.hypot(acceleration[0], acceleration[1]),
        vertical_thrust_acceleration,
    )
    if tilt > limits.max_tilt_rad + 1e-9:
        return False
    if norm(jerk) > margin * limits.max_jerk_mps3 + 1e-6:
        return False
    if norm(snap) > margin * limits.max_snap_mps4 + 1e-6:
        return False

    yaw_rate = sample.curvature_xy * speed
    yaw_acceleration = (
        sample.curvature_derivative_xy * speed * speed
        + sample.curvature_xy * longitudinal_acceleration
    )
    flat_input = flat_body_rate_and_acceleration(
        acceleration,
        jerk,
        snap,
        sample.yaw,
        yaw_rate,
        yaw_acceleration,
    )
    if flat_input is None:
        return False
    body_rate, body_rate_dot = flat_input
    if norm(body_rate) > margin * limits.max_bodyrate_ff_radps + 1e-6:
        return False
    return all(
        abs(body_rate_dot[axis])
        <= margin * limits.max_bodyrate_dot_ff_radps2[axis] + 1e-6
        for axis in range(3)
    )


def _smooth_squared_speed_under_cap(
    squared_speed: Sequence[float],
    half_window_samples: int,
) -> list[float]:
    """Smooth a non-negative profile without exceeding its input cap."""
    if half_window_samples <= 0:
        return list(squared_speed)

    width = 2 * half_window_samples + 1
    padded = [0.0] * half_window_samples + list(squared_speed) + [0.0] * half_window_samples
    minimum_queue: deque[int] = deque()
    eroded: list[float] = []
    for index, value in enumerate(padded):
        while minimum_queue and padded[minimum_queue[-1]] >= value:
            minimum_queue.pop()
        minimum_queue.append(index)
        if minimum_queue[0] <= index - width:
            minimum_queue.popleft()
        if index >= width - 1:
            eroded.append(padded[minimum_queue[0]])

    averaged_input = [0.0] * half_window_samples + eroded + [0.0] * half_window_samples
    prefix_sum = [0.0]
    for value in averaged_input:
        prefix_sum.append(prefix_sum[-1] + value)
    return [
        (prefix_sum[index + width] - prefix_sum[index]) / width
        for index in range(len(squared_speed))
    ]


def _attach_observational_high_derivatives(
    reference: Sequence[ReferencePoint],
    *,
    rate_hz: float,
    smoothing_window_s: float,
    max_jerk_mps3: float,
) -> list[ReferencePoint]:
    """Reproduce the stable-flight jerk and retain snap for later experiments."""
    if not reference:
        return []

    raw_jerk: list[Vec3] = []
    for index, point in enumerate(reference):
        if len(reference) == 1:
            jerk = (0.0, 0.0, 0.0)
        elif index == 0:
            duration = max(1e-6, reference[1].time_s - point.time_s)
            jerk = scale(sub(reference[1].acceleration, point.acceleration), 1.0 / duration)
        elif index + 1 == len(reference):
            duration = max(1e-6, point.time_s - reference[index - 1].time_s)
            jerk = scale(sub(point.acceleration, reference[index - 1].acceleration), 1.0 / duration)
        else:
            duration = max(1e-6, reference[index + 1].time_s - reference[index - 1].time_s)
            jerk = scale(
                sub(reference[index + 1].acceleration, reference[index - 1].acceleration),
                1.0 / duration,
            )
        raw_jerk.append(jerk)

    radius = max(0, int(round(0.5 * smoothing_window_s * rate_hz)))
    with_jerk: list[ReferencePoint] = []
    for index, point in enumerate(reference):
        weighted_sum = [0.0, 0.0, 0.0]
        weight_total = 0.0
        for offset in range(-radius, radius + 1):
            sample_index = min(len(raw_jerk) - 1, max(0, index + offset))
            weight = float(radius + 1 - abs(offset))
            weight_total += weight
            for axis in range(3):
                weighted_sum[axis] += weight * raw_jerk[sample_index][axis]
        jerk = tuple(value / max(weight_total, 1.0) for value in weighted_sum)
        jerk_norm = norm(jerk)
        if jerk_norm > max_jerk_mps3:
            jerk = scale(jerk, max_jerk_mps3 / jerk_norm)
        if index == 0 or index + 1 == len(reference):
            jerk = (0.0, 0.0, 0.0)
        with_jerk.append(replace(point, jerk=jerk))

    result: list[ReferencePoint] = []
    for index, point in enumerate(with_jerk):
        if index == 0 or index + 1 == len(with_jerk):
            snap = (0.0, 0.0, 0.0)
        else:
            duration = max(
                1e-6,
                with_jerk[index + 1].time_s - with_jerk[index - 1].time_s,
            )
            snap = scale(
                sub(with_jerk[index + 1].jerk, with_jerk[index - 1].jerk),
                1.0 / duration,
            )
        result.append(replace(point, snap=snap))
    return result


def generate_multi_lap_reference_high_order(
    path: FigureEightPath,
    *,
    entry_arc_m: float,
    lap_speeds_mps: Sequence[float],
    rate_hz: float,
    limits: MotionLimits,
    jerk_smoothing_s: float = 0.05,
) -> list[ReferencePoint]:
    if not lap_speeds_mps or any(speed <= 0.0 for speed in lap_speeds_mps) or rate_hz <= 0.0:
        raise ValueError("lap speeds and rate must be positive")
    if not math.isfinite(jerk_smoothing_s) or jerk_smoothing_s < 0.0:
        raise ValueError("jerk smoothing duration must be finite and non-negative")
    if min(
        limits.max_long_accel_mps2,
        limits.max_decel_mps2,
        limits.max_lateral_accel_mps2,
        limits.max_jerk_mps3,
        limits.max_yaw_rate_radps,
        limits.max_snap_mps4,
        limits.max_bodyrate_ff_radps,
        *limits.max_bodyrate_dot_ff_radps2,
        limits.max_tilt_rad,
    ) <= 0.0:
        raise ValueError("motion limits must be positive")

    dt = 1.0 / rate_hz
    lap_count = len(lap_speeds_mps)
    total_length_m = lap_count * path.length_m

    def lap_index_for_progress(progress_m: float) -> int:
        if progress_m >= total_length_m:
            return lap_count - 1
        return min(lap_count - 1, max(0, int(progress_m / path.length_m)))

    def speed_cap_for_progress(progress_m: float) -> float:
        lap_index = lap_index_for_progress(progress_m)
        local_progress_m = progress_m - lap_index * path.length_m
        return _path_speed_cap(
            path,
            local_progress_m,
            entry_arc_m,
            lap_speeds_mps[lap_index],
            limits,
        )

    # Each segment blends velocity with a seventh-order smootherstep. Its
    # acceleration, jerk and snap are all zero at segment boundaries, so the
    # complete multi-lap time law is C4 without online differentiation.
    transition_length_m = clamp(
        max(lap_speeds_mps) * max(jerk_smoothing_s, 0.10),
        0.80,
        2.00,
    )
    segments_per_lap = max(32, int(math.ceil(path.length_m / transition_length_m)))
    profile_count = segments_per_lap * lap_count
    profile_step_m = total_length_m / profile_count
    cap_samples_per_segment = 24
    interval_speed_caps = [
        min(
            speed_cap_for_progress(
                profile_step_m
                * (segment + sample_index / cap_samples_per_segment)
            )
            for sample_index in range(cap_samples_per_segment + 1)
        )
        for segment in range(profile_count)
    ]
    speed_profile = [0.0] * (profile_count + 1)
    for index in range(1, profile_count):
        speed_profile[index] = min(
            interval_speed_caps[index - 1],
            interval_speed_caps[index],
        )

    def transition_kinematics(
        segment: int,
        normalized_time: float,
        duration: float,
    ) -> tuple[float, float, float, float, float]:
        del segment
        return smootherstep7_velocity_kinematics(normalized_time, duration)

    def transition_duration(
        segment: int,
        start_speed: float,
        end_speed: float,
    ) -> float:
        _, _, _, _, integral_fraction = transition_kinematics(segment, 1.0, 1.0)
        average_speed = start_speed + (end_speed - start_speed) * integral_fraction
        if average_speed <= 1e-9:
            return math.inf
        return profile_step_m / average_speed

    def transition_is_feasible(
        segment: int,
        start_speed: float,
        end_speed: float,
    ) -> bool:
        speed_sum = start_speed + end_speed
        if speed_sum <= 1e-9:
            return True
        duration = transition_duration(segment, start_speed, end_speed)
        speed_delta = end_speed - start_speed
        segment_start_m = segment * profile_step_m
        feasibility_samples = 65 if segment == 0 else 17
        for sample_index in range(feasibility_samples):
            normalized_time = sample_index / (feasibility_samples - 1)
            local_time = normalized_time * duration
            blend, blend_dot, blend_ddot, blend_dddot, blend_integral = (
                transition_kinematics(segment, normalized_time, duration)
            )
            speed = start_speed + speed_delta * blend
            long_accel = speed_delta * blend_dot
            long_jerk = speed_delta * blend_ddot
            long_snap = speed_delta * blend_dddot
            if long_accel > 0.90 * limits.max_long_accel_mps2 + 1e-6:
                return False
            if -long_accel > 0.90 * limits.max_decel_mps2 + 1e-6:
                return False

            progress = clamp(
                segment_start_m
                + start_speed * local_time
                + speed_delta * blend_integral,
                0.0,
                total_length_m,
            )
            lap_index = lap_index_for_progress(progress)
            local_progress_m = progress - lap_index * path.length_m
            sample = path.sample(local_progress_m, entry_arc_m)
            _, acceleration, jerk, snap = _path_time_derivatives(
                sample,
                speed,
                long_accel,
                long_jerk,
                long_snap,
            )
            if norm(jerk) > 0.98 * limits.max_jerk_mps3 + 1e-6:
                return False
            snap_margin = 0.90 if segment == 0 else 0.98
            if norm(snap) > snap_margin * limits.max_snap_mps4 + 1e-6:
                return False
            body_rate = flat_body_rate(
                acceleration,
                jerk,
                sample.yaw,
                sample.curvature_xy * speed,
            )
            if body_rate is None or norm(body_rate) > 0.98 * limits.max_bodyrate_ff_radps + 1e-6:
                return False
            if not _flat_kinematics_within_limits(
                sample,
                speed,
                long_accel,
                long_jerk,
                long_snap,
                limits,
            ):
                return False
        return True

    def reachable_end_speed(
        segment: int,
        start_speed: float,
        requested_speed: float,
    ) -> float:
        if requested_speed <= start_speed or transition_is_feasible(
            segment, start_speed, requested_speed
        ):
            return requested_speed
        lower = start_speed
        upper = requested_speed
        for _ in range(28):
            candidate = 0.5 * (lower + upper)
            if transition_is_feasible(segment, start_speed, candidate):
                lower = candidate
            else:
                upper = candidate
        return lower

    def reachable_start_speed(
        segment: int,
        requested_speed: float,
        end_speed: float,
    ) -> float:
        if requested_speed <= end_speed or transition_is_feasible(
            segment, requested_speed, end_speed
        ):
            return requested_speed
        lower = end_speed
        upper = requested_speed
        for _ in range(28):
            candidate = 0.5 * (lower + upper)
            if transition_is_feasible(segment, candidate, end_speed):
                lower = candidate
            else:
                upper = candidate
        return lower

    for _ in range(3):
        for index in range(profile_count):
            speed_profile[index + 1] = reachable_end_speed(
                index,
                speed_profile[index],
                speed_profile[index + 1],
            )
        for index in range(profile_count - 1, -1, -1):
            speed_profile[index] = reachable_start_speed(
                index,
                speed_profile[index],
                speed_profile[index + 1],
            )

    segment_durations: list[float] = []
    cumulative_times = [0.0]
    for index in range(profile_count):
        duration = transition_duration(
            index,
            speed_profile[index],
            speed_profile[index + 1],
        )
        if not math.isfinite(duration):
            raise RuntimeError("multi-lap speed profile contains an unreachable zero-speed segment")
        segment_durations.append(duration)
        cumulative_times.append(cumulative_times[-1] + duration)

    total_time_s = cumulative_times[-1]
    sample_count = max(1, int(math.ceil(total_time_s * rate_hz)))
    result: list[ReferencePoint] = []
    previous_yaw: float | None = None
    for step in range(sample_count + 1):
        time_s = min(step * dt, total_time_s)
        if step == sample_count:
            time_s = total_time_s
        segment = min(
            profile_count - 1,
            max(0, bisect_left(cumulative_times, time_s) - 1),
        )
        local_time = clamp(
            time_s - cumulative_times[segment],
            0.0,
            segment_durations[segment],
        )
        start_speed = speed_profile[segment]
        end_speed = speed_profile[segment + 1]
        segment_duration = segment_durations[segment]
        blend, blend_dot, blend_ddot, blend_dddot, blend_integral = (
            transition_kinematics(
                segment,
                local_time / segment_duration,
                segment_duration,
            )
        )
        speed_delta = end_speed - start_speed
        speed = start_speed + speed_delta * blend
        long_accel = speed_delta * blend_dot
        long_jerk = speed_delta * blend_ddot
        long_snap = speed_delta * blend_dddot
        progress = clamp(
            segment * profile_step_m
            + start_speed * local_time
            + speed_delta * blend_integral,
            0.0,
            total_length_m,
        )
        if step == sample_count:
            progress = total_length_m
            speed = 0.0
            long_accel = 0.0
            long_jerk = 0.0
            long_snap = 0.0

        lap_index = lap_index_for_progress(progress)
        local_progress_m = progress - lap_index * path.length_m
        sample = path.sample(local_progress_m, entry_arc_m)
        yaw = sample.yaw
        if previous_yaw is not None:
            yaw = previous_yaw + normalize_angle(yaw - previous_yaw)
        previous_yaw = yaw
        velocity, acceleration, jerk, snap = _path_time_derivatives(
            sample,
            speed,
            long_accel,
            long_jerk,
            long_snap,
        )
        result.append(
            ReferencePoint(
                time_s=time_s,
                position=sample.position,
                velocity=velocity,
                acceleration=acceleration,
                jerk=jerk,
                snap=snap,
                yaw=yaw,
                yaw_rate=sample.curvature_xy * speed,
                yaw_acceleration=(
                    sample.curvature_derivative_xy * speed * speed
                    + sample.curvature_xy * long_accel
                ),
                horizontal_speed_mps=speed,
                horizontal_accel_mps2=long_accel,
                lap_index=lap_index,
                arc_progress_m=progress,
            )
        )
        hard_cap = speed_cap_for_progress(progress)
        if speed > hard_cap + 1e-3:
            raise RuntimeError(
                f"multi-lap speed planner exceeded a path cap: {speed:.3f} > {hard_cap:.3f}"
            )

    peak_positive_accel = max(point.horizontal_accel_mps2 for point in result)
    peak_decel = max(-point.horizontal_accel_mps2 for point in result)
    if peak_positive_accel > limits.max_long_accel_mps2 + 1e-3:
        raise RuntimeError(
            f"multi-lap speed planner cannot satisfy acceleration limit: "
            f"{peak_positive_accel:.3f} > {limits.max_long_accel_mps2:.3f} m/s^2"
        )
    if peak_decel > limits.max_decel_mps2 + 1e-3:
        raise RuntimeError(
            f"multi-lap speed planner cannot satisfy deceleration limit: "
            f"{peak_decel:.3f} > {limits.max_decel_mps2:.3f} m/s^2"
        )
    peak_jerk = max(norm(point.jerk) for point in result)
    if peak_jerk > limits.max_jerk_mps3 + 1e-3:
        raise RuntimeError(
            f"multi-lap speed planner cannot satisfy jerk limit: {peak_jerk:.3f} > "
            f"{limits.max_jerk_mps3:.3f} m/s^3"
        )
    peak_snap = max(norm(point.snap) for point in result)
    if peak_snap > limits.max_snap_mps4 + 1e-3:
        raise RuntimeError(
            f"multi-lap speed planner cannot satisfy snap limit: {peak_snap:.3f} > "
            f"{limits.max_snap_mps4:.3f} m/s^4"
        )
    peak_bodyrate_ff = max(
        norm(body_rate)
        for point in result
        if (
            body_rate := flat_body_rate(
                point.acceleration,
                point.jerk,
                point.yaw,
                point.yaw_rate,
            )
        ) is not None
    )
    if peak_bodyrate_ff > limits.max_bodyrate_ff_radps + 1e-3:
        raise RuntimeError(
            f"multi-lap speed planner cannot satisfy body-rate feedforward limit: "
            f"{peak_bodyrate_ff:.3f} > {limits.max_bodyrate_ff_radps:.3f} rad/s"
        )
    bodyrate_dot_samples = []
    for point in result:
        flat_input = flat_body_rate_and_acceleration(
            point.acceleration,
            point.jerk,
            point.snap,
            point.yaw,
            point.yaw_rate,
            point.yaw_acceleration,
        )
        if flat_input is None:
            raise RuntimeError("multi-lap reference contains an invalid flatness input")
        bodyrate_dot_samples.append(flat_input[1])
    peak_bodyrate_dot_ff = tuple(
        max(abs(value[axis]) for value in bodyrate_dot_samples)
        for axis in range(3)
    )
    for axis in range(3):
        if peak_bodyrate_dot_ff[axis] > limits.max_bodyrate_dot_ff_radps2[axis] + 1e-3:
            raise RuntimeError(
                f"multi-lap speed planner cannot satisfy body-rate-dot axis {axis} limit: "
                f"{peak_bodyrate_dot_ff[axis]:.3f} > "
                f"{limits.max_bodyrate_dot_ff_radps2[axis]:.3f} rad/s^2"
            )

    return result


def generate_multi_lap_reference(
    path: FigureEightPath,
    *,
    entry_arc_m: float,
    lap_speeds_mps: Sequence[float],
    rate_hz: float,
    limits: MotionLimits,
    jerk_smoothing_s: float = 0.05,
) -> list[ReferencePoint]:
    """Generate the P/V/A time law used by the stable 8 m/s flight."""
    if not lap_speeds_mps or any(speed <= 0.0 for speed in lap_speeds_mps) or rate_hz <= 0.0:
        raise ValueError("lap speeds and rate must be positive")
    if not math.isfinite(jerk_smoothing_s) or jerk_smoothing_s < 0.0:
        raise ValueError("jerk smoothing duration must be finite and non-negative")
    if min(
        limits.max_long_accel_mps2,
        limits.max_decel_mps2,
        limits.max_lateral_accel_mps2,
        limits.max_jerk_mps3,
        limits.max_yaw_rate_radps,
    ) <= 0.0:
        raise ValueError("motion limits must be positive")

    dt = 1.0 / rate_hz
    lap_count = len(lap_speeds_mps)
    total_length_m = lap_count * path.length_m
    profile_count = max(1200 * lap_count, int(math.ceil(total_length_m * 50.0)))
    profile_step_m = total_length_m / profile_count

    def lap_index_for_progress(progress_m: float) -> int:
        if progress_m >= total_length_m:
            return lap_count - 1
        return min(lap_count - 1, max(0, int(progress_m / path.length_m)))

    def speed_cap_for_progress(progress_m: float) -> float:
        lap_index = lap_index_for_progress(progress_m)
        local_progress_m = progress_m - lap_index * path.length_m
        return _path_speed_cap(
            path,
            local_progress_m,
            entry_arc_m,
            lap_speeds_mps[lap_index],
            limits,
        )

    speed_profile = [
        speed_cap_for_progress(profile_step_m * index)
        for index in range(profile_count + 1)
    ]
    speed_profile[0] = 0.0
    speed_profile[-1] = 0.0
    planning_accel = 0.60 * limits.max_long_accel_mps2
    planning_decel = 0.50 * limits.max_decel_mps2
    for index in range(1, profile_count + 1):
        reachable = math.sqrt(
            max(0.0, speed_profile[index - 1] ** 2 + 2.0 * planning_accel * profile_step_m)
        )
        speed_profile[index] = min(speed_profile[index], reachable)
    for index in range(profile_count - 1, -1, -1):
        reachable = math.sqrt(
            max(0.0, speed_profile[index + 1] ** 2 + 2.0 * planning_decel * profile_step_m)
        )
        speed_profile[index] = min(speed_profile[index], reachable)

    smoothing_half_width_m = max(
        0.55,
        0.60
        * max(lap_speeds_mps)
        * max(planning_accel, planning_decel)
        / limits.max_jerk_mps3,
    )
    smoothing_samples = max(1, int(round(smoothing_half_width_m / profile_step_m)))
    squared_speed = _smooth_squared_speed_under_cap(
        [speed * speed for speed in speed_profile],
        smoothing_samples,
    )
    speed_profile = [math.sqrt(max(0.0, value)) for value in squared_speed]
    speed_profile[0] = 0.0
    speed_profile[-1] = 0.0

    segment_durations: list[float] = []
    cumulative_times = [0.0]
    for index in range(profile_count):
        speed_sum = speed_profile[index] + speed_profile[index + 1]
        if speed_sum <= 1e-9:
            raise RuntimeError("multi-lap speed profile contains an unreachable zero-speed segment")
        duration = 2.0 * profile_step_m / speed_sum
        segment_durations.append(duration)
        cumulative_times.append(cumulative_times[-1] + duration)

    total_time_s = cumulative_times[-1]
    sample_count = max(1, int(math.ceil(total_time_s * rate_hz)))
    result: list[ReferencePoint] = []
    previous_yaw: float | None = None
    for step in range(sample_count + 1):
        time_s = min(step * dt, total_time_s)
        if step == sample_count:
            time_s = total_time_s
        segment = min(
            profile_count - 1,
            max(0, bisect_left(cumulative_times, time_s) - 1),
        )
        local_time = clamp(
            time_s - cumulative_times[segment],
            0.0,
            segment_durations[segment],
        )
        start_speed = speed_profile[segment]
        end_speed = speed_profile[segment + 1]
        long_accel = (end_speed - start_speed) / segment_durations[segment]
        speed = clamp(
            start_speed + long_accel * local_time,
            min(start_speed, end_speed),
            max(start_speed, end_speed),
        )
        progress = clamp(
            segment * profile_step_m
            + start_speed * local_time
            + 0.5 * long_accel * local_time * local_time,
            0.0,
            total_length_m,
        )
        if step == sample_count:
            progress = total_length_m
            speed = 0.0
            long_accel = 0.0

        lap_index = lap_index_for_progress(progress)
        local_progress_m = progress - lap_index * path.length_m
        sample = path.sample(local_progress_m, entry_arc_m)
        yaw = sample.yaw
        if previous_yaw is not None:
            yaw = previous_yaw + normalize_angle(yaw - previous_yaw)
        previous_yaw = yaw
        normal_xy = (-sample.tangent_xy[1], sample.tangent_xy[0])
        lateral_accel = sample.curvature_xy * speed * speed
        velocity = (
            sample.tangent_xy[0] * speed,
            sample.tangent_xy[1] * speed,
            sample.z_slope * speed,
        )
        acceleration = (
            sample.tangent_xy[0] * long_accel + normal_xy[0] * lateral_accel,
            sample.tangent_xy[1] * long_accel + normal_xy[1] * lateral_accel,
            sample.z_curvature * speed * speed + sample.z_slope * long_accel,
        )
        result.append(
            ReferencePoint(
                time_s=time_s,
                position=sample.position,
                velocity=velocity,
                acceleration=acceleration,
                jerk=(0.0, 0.0, 0.0),
                snap=(0.0, 0.0, 0.0),
                yaw=yaw,
                yaw_rate=sample.curvature_xy * speed,
                yaw_acceleration=(
                    sample.curvature_derivative_xy * speed * speed
                    + sample.curvature_xy * long_accel
                ),
                horizontal_speed_mps=speed,
                horizontal_accel_mps2=long_accel,
                lap_index=lap_index,
                arc_progress_m=progress,
            )
        )
        hard_cap = speed_cap_for_progress(progress)
        if speed > hard_cap + 1e-3:
            raise RuntimeError(
                f"multi-lap speed planner exceeded a path cap: {speed:.3f} > {hard_cap:.3f}"
            )

    result = _attach_observational_high_derivatives(
        result,
        rate_hz=rate_hz,
        smoothing_window_s=jerk_smoothing_s,
        max_jerk_mps3=limits.max_jerk_mps3,
    )
    peak_jerk = max(norm(point.jerk) for point in result)
    if peak_jerk > limits.max_jerk_mps3 + 1e-3:
        raise RuntimeError(
            f"multi-lap speed planner cannot satisfy jerk limit: {peak_jerk:.3f} > "
            f"{limits.max_jerk_mps3:.3f} m/s^3"
        )
    return result


def generate_lap_reference(
    path: FigureEightPath,
    *,
    entry_arc_m: float,
    cruise_speed_mps: float,
    rate_hz: float,
    limits: MotionLimits,
) -> list[ReferencePoint]:
    return generate_multi_lap_reference(
        path,
        entry_arc_m=entry_arc_m,
        lap_speeds_mps=(cruise_speed_mps,),
        rate_hz=rate_hz,
        limits=limits,
    )


def generate_transfer_reference(
    start: Vec3,
    end: Vec3,
    *,
    yaw: float,
    max_speed_mps: float,
    max_accel_mps2: float,
    rate_hz: float,
) -> list[ReferencePoint]:
    distance = norm(sub(end, start))
    if max_speed_mps <= 0.0 or max_accel_mps2 <= 0.0 or rate_hz <= 0.0:
        raise ValueError("transfer limits must be positive")
    if distance <= 1e-6:
        return [
            ReferencePoint(
                time_s=0.0,
                position=end,
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
        ]

    duration = max(
        1.0,
        1.875 * distance / max_speed_mps,
        math.sqrt(5.78 * distance / max_accel_mps2),
    )
    steps = max(1, math.ceil(duration * rate_hz))
    duration = steps / rate_hz
    delta = sub(end, start)
    direction_xy_norm = math.hypot(delta[0], delta[1])
    transfer_yaw = yaw if direction_xy_norm <= 1e-6 else math.atan2(delta[1], delta[0])
    result: list[ReferencePoint] = []
    for step in range(steps + 1):
        time_s = step / rate_hz
        u = clamp(time_s / duration, 0.0, 1.0)
        blend, blend_d, blend_dd, blend_ddd, blend_dddd = smootherstep9_kinematics(
            u,
            duration,
        )
        velocity = scale(delta, blend_d)
        acceleration = scale(delta, blend_dd)
        jerk = scale(delta, blend_ddd)
        snap = scale(delta, blend_dddd)
        result.append(
            ReferencePoint(
                time_s=time_s,
                position=lerp(start, end, blend),
                velocity=velocity,
                acceleration=acceleration,
                jerk=jerk,
                snap=snap,
                yaw=transfer_yaw,
                yaw_rate=0.0,
                yaw_acceleration=0.0,
                horizontal_speed_mps=norm_xy(velocity),
                horizontal_accel_mps2=norm_xy(acceleration),
            )
        )
    return result


def polygon_signed_area(points: Sequence[tuple[float, float]]) -> float:
    return 0.5 * sum(
        points[index][0] * points[(index + 1) % len(points)][1]
        - points[(index + 1) % len(points)][0] * points[index][1]
        for index in range(len(points))
    )


def point_in_convex_polygon(
    point: tuple[float, float],
    polygon: Sequence[tuple[float, float]],
) -> bool:
    if len(polygon) < 3:
        return False
    orientation = 1.0 if polygon_signed_area(polygon) >= 0.0 else -1.0
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        cross = (second[0] - first[0]) * (point[1] - first[1]) - (
            second[1] - first[1]
        ) * (point[0] - first[0])
        if orientation * cross < -1e-9:
            return False
    return True


def distance_to_polygon_edges(
    point: tuple[float, float],
    polygon: Sequence[tuple[float, float]],
) -> float:
    distances = []
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-12:
            continue
        fraction = clamp(
            ((point[0] - first[0]) * dx + (point[1] - first[1]) * dy) / length_squared,
            0.0,
            1.0,
        )
        nearest = (first[0] + fraction * dx, first[1] + fraction * dy)
        distances.append(math.hypot(point[0] - nearest[0], point[1] - nearest[1]))
    return min(distances) if distances else 0.0


def validate_points_in_flight_area(
    points: Iterable[Vec3],
    *,
    polygon: Sequence[tuple[float, float]],
    margin_m: float,
    xyz_limits: tuple[float, float, float, float, float, float],
    require_margin: bool,
) -> tuple[bool, str]:
    numerical_tolerance_m = 1e-9
    x_min, x_max, y_min, y_max, z_min, z_max = xyz_limits
    for index, point in enumerate(points):
        xy = (point[0], point[1])
        if not point_in_convex_polygon(xy, polygon):
            return False, f"sample {index} is outside the flight polygon: {point}"
        if (
            require_margin
            and distance_to_polygon_edges(xy, polygon) + numerical_tolerance_m < margin_m
        ):
            return False, f"sample {index} violates the {margin_m:.2f} m polygon margin: {point}"
        if not (x_min <= point[0] <= x_max):
            return False, f"sample {index} violates x limits [{x_min}, {x_max}]: {point}"
        if not (y_min <= point[1] <= y_max):
            return False, f"sample {index} violates y limits [{y_min}, {y_max}]: {point}"
        if not (z_min <= point[2] <= z_max):
            return False, f"sample {index} violates z limits [{z_min}, {z_max}]: {point}"
    return True, "ok"
