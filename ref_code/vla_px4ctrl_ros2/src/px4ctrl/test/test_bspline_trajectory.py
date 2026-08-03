from __future__ import annotations

import math

from bspline_trajectory import (
    FigureEightPath,
    MotionLimits,
    flat_body_rate_and_acceleration,
    generate_lap_reference,
    generate_multi_lap_reference,
    generate_multi_lap_reference_high_order,
    generate_transfer_reference,
    normalize_angle,
    smootherstep7_velocity_kinematics,
    warped_smootherstep7_velocity_kinematics,
    validate_points_in_flight_area,
)


BOUNDARY = (
    (14.736, 2.162),
    (15.197, -1.650),
    (-1.558, -1.754),
    (-1.300, 1.850),
)
LIMITS = (-7.0, 14.0, -2.5, 2.5, -0.3, 2.5)


def make_path() -> FigureEightPath:
    return FigureEightPath(
        center_xy=(6.76875, 0.152),
        x_span_m=14.0,
        y_span_m=2.6,
        heading_rad=math.radians(0.7268386353),
        z_min_m=1.0,
        z_max_m=1.5,
    )


def test_periodic_figure_eight_closes_with_continuous_tangent() -> None:
    path = make_path()
    start = path.sample(0.0)
    end = path.sample(path.length_m)
    assert math.dist(start.position, end.position) < 1e-6
    assert math.hypot(
        start.tangent_xy[0] - end.tangent_xy[0],
        start.tangent_xy[1] - end.tangent_xy[1],
    ) < 1e-6
    assert abs(normalize_angle(start.yaw - end.yaw)) < 1e-6
    assert abs(start.position[2] - 1.0) < 1e-9
    assert abs(path.sample(0.5 * path.length_m).position[2] - 1.5) < 1e-9


def test_default_path_respects_polygon_margin_and_px4ctrl_limits() -> None:
    path = make_path()
    points = [path.sample(path.length_m * index / 8000).position for index in range(8000)]
    valid, reason = validate_points_in_flight_area(
        points,
        polygon=BOUNDARY,
        margin_m=0.40,
        xyz_limits=LIMITS,
        require_margin=True,
    )
    assert valid, reason


def test_polygon_margin_accepts_roundoff_but_rejects_real_shortfall() -> None:
    polygon = ((1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0))
    limits = (-1.0, 1.0, -1.0, 1.0, 0.0, 2.0)
    valid, reason = validate_points_in_flight_area(
        [(0.0, -0.8000000000000004, 1.0)],
        polygon=polygon,
        margin_m=0.20,
        xyz_limits=limits,
        require_margin=True,
    )
    assert valid, reason

    valid, _ = validate_points_in_flight_area(
        [(0.0, -0.80001, 1.0)],
        polygon=polygon,
        margin_m=0.20,
        xyz_limits=limits,
        require_margin=True,
    )
    assert not valid


def test_nearest_entry_is_on_path_and_handles_center_crossing() -> None:
    path = make_path()
    launch = (6.8, 0.1, 1.0)
    entry = path.nearest_entry(launch, current_yaw=0.0)
    sampled = path.sample(0.0, entry.arc_length_m)
    assert math.dist(entry.position, sampled.position) < 1e-6
    assert entry.distance_m < 0.2
    assert math.isfinite(entry.yaw)


def test_lap_profiles_reach_requested_speed_and_obey_dynamic_limits() -> None:
    path = make_path()
    entry = path.nearest_entry((0.0, 0.0, 1.0), current_yaw=0.0)
    limits = MotionLimits(6.0, 8.0, 12.0, 40.0, 1.2)
    first = generate_lap_reference(
        path,
        entry_arc_m=entry.arc_length_m,
        cruise_speed_mps=2.0,
        rate_hz=100.0,
        limits=limits,
    )
    second = generate_lap_reference(
        path,
        entry_arc_m=entry.arc_length_m,
        cruise_speed_mps=4.0,
        rate_hz=100.0,
        limits=limits,
    )
    assert max(point.horizontal_speed_mps for point in first) >= 1.95
    assert max(point.horizontal_speed_mps for point in second) >= 3.8
    for reference in (first, second):
        assert reference[0].horizontal_speed_mps == 0.0
        assert reference[-1].horizontal_speed_mps == 0.0
        assert math.dist(reference[0].position, reference[-1].position) < 1e-6
        assert max(
            abs(reference[index].yaw - reference[index - 1].yaw)
            for index in range(1, len(reference))
        ) < 0.05
        assert max(abs(point.yaw_rate) for point in reference) <= 1.2 + 1e-6
        assert max(abs(point.horizontal_accel_mps2) for point in reference) <= 8.0 + 1e-6
        assert max(math.hypot(point.acceleration[0], point.acceleration[1]) for point in reference) <= 12.6
        assert max(math.dist(point.jerk, (0.0, 0.0, 0.0)) for point in reference) <= 40.0 + 1e-3


def test_three_laps_are_one_continuous_two_four_six_mps_reference() -> None:
    path = make_path()
    entry = path.nearest_entry((0.0, 0.0, 1.0), current_yaw=0.0)
    limits = MotionLimits(8.0, 10.0, 9.0, 80.0, 3.0)
    reference = generate_multi_lap_reference(
        path,
        entry_arc_m=entry.arc_length_m,
        lap_speeds_mps=(2.0, 4.0, 6.0),
        rate_hz=100.0,
        limits=limits,
        jerk_smoothing_s=0.05,
    )

    assert reference[0].horizontal_speed_mps == 0.0
    assert reference[-1].horizontal_speed_mps == 0.0
    assert math.dist(reference[0].position, reference[-1].position) < 1e-6
    assert {point.lap_index for point in reference} == {0, 1, 2}
    for lap_index, minimum_peak in enumerate((1.95, 3.8, 5.7)):
        lap = [point for point in reference if point.lap_index == lap_index]
        assert max(point.horizontal_speed_mps for point in lap) >= minimum_peak

    transitions = [
        (reference[index - 1], reference[index])
        for index in range(1, len(reference))
        if reference[index - 1].lap_index != reference[index].lap_index
    ]
    assert len(transitions) == 2
    assert all(
        before.horizontal_speed_mps > 1.5 and after.horizontal_speed_mps > 1.5
        for before, after in transitions
    )
    assert max(abs(point.yaw_rate) for point in reference) <= 3.0 + 1e-6
    assert max(math.dist(point.jerk, (0.0, 0.0, 0.0)) for point in reference) <= 80.0 + 1e-3
    assert max(
        abs(reference[index].yaw - reference[index - 1].yaw)
        for index in range(1, len(reference))
    ) < 0.05


def test_stable_eight_mps_baseline_reference_is_reproduced() -> None:
    path = FigureEightPath(
        center_xy=(6.55, 0.0),
        x_span_m=16.7,
        y_span_m=3.8,
        heading_rad=0.0,
        z_min_m=1.0,
        z_max_m=1.5,
    )
    entry = path.nearest_entry(
        (1.8601549875371333, -1.7658533319071517, 1.0),
        current_yaw=0.20028439154399205,
    )
    reference = generate_multi_lap_reference(
        path,
        entry_arc_m=entry.arc_length_m,
        lap_speeds_mps=(8.0, 8.0, 8.0),
        rate_hz=100.0,
        limits=MotionLimits(14.0, 16.0, 15.0, 160.0, 5.0),
        jerk_smoothing_s=0.05,
    )

    assert len(reference) == 2087
    assert math.isclose(reference[-1].time_s, 20.855787194661254, abs_tol=1e-9)
    assert math.dist(
        reference[0].position,
        (1.8601549875371333, -1.7658533319071517, 1.0),
    ) < 2e-8
    assert math.dist(
        reference[1].acceleration,
        (0.14442245386552247, 0.02931904403412967, 2.1297081312153047e-08),
    ) < 4e-10
    assert math.dist(
        reference[1].jerk,
        (-1.9958820458545715e-05, 9.831266176458843e-05, 5.442587446070659e-06),
    ) < 1e-12


def test_transfer_has_zero_endpoint_velocity_and_acceleration() -> None:
    reference = generate_transfer_reference(
        (0.0, 0.0, 1.0),
        (4.0, 1.0, 1.0),
        yaw=0.0,
        max_speed_mps=1.0,
        max_accel_mps2=1.0,
        rate_hz=100.0,
    )
    assert math.dist(reference[0].position, (0.0, 0.0, 1.0)) < 1e-9
    assert math.dist(reference[-1].position, (4.0, 1.0, 1.0)) < 1e-9
    assert reference[0].velocity == (0.0, 0.0, 0.0)
    assert reference[-1].velocity == (0.0, 0.0, 0.0)
    assert reference[0].acceleration == (0.0, 0.0, 0.0)
    assert all(abs(value) < 1e-9 for value in reference[-1].acceleration)
    assert all(abs(value) < 1e-9 for value in reference[0].jerk)
    assert all(abs(value) < 1e-9 for value in reference[-1].jerk)
    assert all(abs(value) < 1e-9 for value in reference[0].snap)
    assert all(abs(value) < 1e-9 for value in reference[-1].snap)


def test_seventh_order_velocity_blend_has_c4_segment_boundaries() -> None:
    duration = 0.8
    start = smootherstep7_velocity_kinematics(0.0, duration)
    end = smootherstep7_velocity_kinematics(1.0, duration)
    assert start[:4] == (0.0, 0.0, 0.0, 0.0)
    assert end[0] == 1.0
    assert all(abs(value) < 1e-9 for value in end[1:4])
    assert abs(end[4] - 0.5 * duration) < 1e-12


def test_warped_velocity_blend_delays_start_and_keeps_c4_boundaries() -> None:
    duration = 1.5
    regular_midpoint = smootherstep7_velocity_kinematics(0.5, duration)[0]
    warped_midpoint = warped_smootherstep7_velocity_kinematics(0.5, duration, 3)[0]
    start = warped_smootherstep7_velocity_kinematics(0.0, duration, 3)
    end = warped_smootherstep7_velocity_kinematics(1.0, duration, 3)
    assert warped_midpoint < 0.05 * regular_midpoint
    assert start[:4] == (0.0, 0.0, 0.0, 0.0)
    assert abs(end[0] - 1.0) < 1e-12
    assert all(abs(value) < 1e-9 for value in end[1:4])


def test_quintic_spline_fourth_derivative_matches_numerical_derivative() -> None:
    path = make_path()
    spline = path.spline
    step = 1e-4
    for parameter in (0.37, 4.25, 9.5, 17.1):
        before = spline.evaluate(parameter - step).third_derivative
        after = spline.evaluate(parameter + step).third_derivative
        numerical = tuple(
            (after[axis] - before[axis]) / (2.0 * step)
            for axis in range(3)
        )
        analytical = spline.evaluate(parameter).fourth_derivative
        assert math.dist(numerical, analytical) < 2e-5


def test_main_reference_high_derivatives_are_finite_and_limited() -> None:
    path = make_path()
    entry = path.nearest_entry((0.0, 0.0, 1.0), current_yaw=0.0)
    limits = MotionLimits(14.0, 16.0, 15.0, 160.0, 5.0, 1000.0)
    reference = generate_multi_lap_reference_high_order(
        path,
        entry_arc_m=entry.arc_length_m,
        lap_speeds_mps=(8.0, 8.0, 8.0),
        rate_hz=100.0,
        limits=limits,
        jerk_smoothing_s=0.05,
    )
    assert all(
        math.isfinite(value)
        for point in reference
        for value in (*point.jerk, *point.snap, point.yaw_acceleration)
    )
    assert max(
        math.dist(point.jerk, (0.0, 0.0, 0.0)) for point in reference
    ) <= 160.0 + 1e-3
    assert max(
        math.dist(point.snap, (0.0, 0.0, 0.0)) for point in reference
    ) <= 1000.0 + 1e-3


def test_yaw_acceleration_matches_yaw_rate_derivative_away_from_boundaries() -> None:
    path = make_path()
    entry = path.nearest_entry((0.0, 0.0, 1.0), current_yaw=0.0)
    reference = generate_multi_lap_reference_high_order(
        path,
        entry_arc_m=entry.arc_length_m,
        lap_speeds_mps=(4.0, 4.0),
        rate_hz=100.0,
        limits=MotionLimits(8.0, 10.0, 12.0, 80.0, 3.0, 1000.0),
        jerk_smoothing_s=0.05,
    )
    errors = []
    for index in range(5, len(reference) - 5):
        if reference[index - 1].lap_index != reference[index + 1].lap_index:
            continue
        duration = reference[index + 1].time_s - reference[index - 1].time_s
        numerical = (
            reference[index + 1].yaw_rate - reference[index - 1].yaw_rate
        ) / duration
        errors.append(abs(numerical - reference[index].yaw_acceleration))
    assert errors
    assert sorted(errors)[int(0.95 * (len(errors) - 1))] < 0.5


def test_main_reference_high_derivatives_match_numerical_derivatives() -> None:
    path = make_path()
    entry = path.nearest_entry((0.0, 0.0, 1.0), current_yaw=0.0)
    reference = generate_multi_lap_reference_high_order(
        path,
        entry_arc_m=entry.arc_length_m,
        lap_speeds_mps=(2.0,),
        rate_hz=500.0,
        limits=MotionLimits(20.0, 20.0, 50.0, 1e6, 20.0, 1e7),
        jerk_smoothing_s=0.05,
    )

    def percentile_error(source: str, expected: str) -> float:
        errors = []
        for index in range(2, len(reference) - 2):
            duration = reference[index + 1].time_s - reference[index - 1].time_s
            numerical = tuple(
                (
                    getattr(reference[index + 1], source)[axis]
                    - getattr(reference[index - 1], source)[axis]
                )
                / duration
                for axis in range(3)
            )
            errors.append(math.dist(numerical, getattr(reference[index], expected)))
        errors.sort()
        return errors[int(0.95 * (len(errors) - 1))]

    assert percentile_error("position", "velocity") < 3e-4
    assert percentile_error("velocity", "acceleration") < 5e-4
    assert percentile_error("acceleration", "jerk") < 2e-3
    assert percentile_error("jerk", "snap") < 3e-2


def test_analytic_body_rate_acceleration_matches_body_rate_derivative() -> None:
    path = make_path()
    entry = path.nearest_entry((0.0, 0.0, 1.0), current_yaw=0.0)
    reference = generate_multi_lap_reference_high_order(
        path,
        entry_arc_m=entry.arc_length_m,
        lap_speeds_mps=(2.0,),
        rate_hz=500.0,
        limits=MotionLimits(
            20.0,
            20.0,
            50.0,
            1e6,
            20.0,
            1e7,
            max_bodyrate_ff_radps=1e6,
            max_bodyrate_dot_ff_radps2=(1e6, 1e6, 1e6),
        ),
        jerk_smoothing_s=0.05,
    )
    body_rates = []
    body_rate_dots = []
    for point in reference:
        flat_input = flat_body_rate_and_acceleration(
            point.acceleration,
            point.jerk,
            point.snap,
            point.yaw,
            point.yaw_rate,
            point.yaw_acceleration,
        )
        assert flat_input is not None
        body_rates.append(flat_input[0])
        body_rate_dots.append(flat_input[1])

    errors = []
    for index in range(2, len(reference) - 2):
        duration = reference[index + 1].time_s - reference[index - 1].time_s
        numerical = tuple(
            (body_rates[index + 1][axis] - body_rates[index - 1][axis]) / duration
            for axis in range(3)
        )
        errors.append(math.dist(numerical, body_rate_dots[index]))

    errors.sort()
    assert errors[int(0.95 * (len(errors) - 1))] < 0.1
