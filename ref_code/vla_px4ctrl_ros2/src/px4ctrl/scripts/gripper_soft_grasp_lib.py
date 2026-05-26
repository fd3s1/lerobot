#!/usr/bin/python3

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, TypeVar


T = TypeVar("T")


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


@dataclass(frozen=True)
class GripperFeedbackState:
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
    received_s: float


@dataclass(frozen=True)
class SoftGraspConfig:
    gripper_open: float = 100.0
    grasp_open_timeout_s: float = 2.0
    grasp_open_tolerance: float = 5.0
    grasp_step_size: float = 3.0
    grasp_step_settle_s: float = 0.10
    grasp_step_timeout_s: float = 0.80
    grasp_goal_tolerance: float = 3.0
    grasp_close_min: float = 15.0
    grasp_contact_current_delta: float = 250.0
    grasp_contact_load_delta: float = 800.0
    grasp_position_error_threshold: float = 20.0
    grasp_angle_contact_delta: float = 20.0
    grasp_stall_delta: float = 0.25
    grasp_contact_min_close_delta: float = 15.0
    grasp_contact_confirm_steps: int = 2
    grasp_balance_load_diff: float = 60.0
    grasp_balance_step: float = 1.5
    grasp_max_balance_steps: int = 8
    grasp_angle_balance_diff: float = 5.0


@dataclass(frozen=True)
class SoftGraspResult:
    left_goal: float
    right_goal: float
    left_contact: bool
    right_contact: bool
    context: object | None


@dataclass(frozen=True)
class _SideMetrics:
    current_delta: float
    load_delta: float
    position_error: float
    angle_lag: float
    stall: bool


class SoftGraspController:
    """Feedback-based soft grasp for normalized gripper positions.

    Position semantics are the project convention: 100 is fully open and
    smaller values close the gripper. Contact is detected primarily from the
    actual position lagging behind the commanded goal, with current/load as
    secondary confirmation.
    """

    def __init__(
        self,
        *,
        config: SoftGraspConfig,
        log: Callable[[str], None],
        publish_pair: Callable[[float, float], None],
        open_gripper: Callable[[int], None],
        wait_feedback: Callable[[], GripperFeedbackState],
        hold_pair_step: Callable[[float, float, float], T],
    ) -> None:
        self.config = config
        self.log = log
        self.publish_pair = publish_pair
        self.open_gripper = open_gripper
        self.wait_feedback = wait_feedback
        self.hold_pair_step = hold_pair_step

    def _side_metrics(
        self,
        *,
        feedback: GripperFeedbackState,
        baseline: GripperFeedbackState,
        previous_feedback: GripperFeedbackState | None,
        side: str,
        goal: float,
        previous_goal: float,
    ) -> _SideMetrics:
        if side == "left":
            pos = feedback.left_pos
            current = feedback.left_current
            load = feedback.left_load
            baseline_current = baseline.left_current
            baseline_load = baseline.left_load
            reported_error = feedback.left_position_error
            previous_pos = previous_feedback.left_pos if previous_feedback is not None else pos
        else:
            pos = feedback.right_pos
            current = feedback.right_current
            load = feedback.right_load
            baseline_current = baseline.right_current
            baseline_load = baseline.right_load
            reported_error = feedback.right_position_error
            previous_pos = previous_feedback.right_pos if previous_feedback is not None else pos

        current_delta = max(0.0, abs(current) - abs(baseline_current))
        load_delta = max(0.0, abs(load) - abs(baseline_load))
        position_error = max(abs(reported_error), max(0.0, pos - goal))
        angle_lag = max(0.0, pos - goal)

        goal_moved = abs(goal - previous_goal) >= max(0.5, 0.5 * self.config.grasp_step_size)
        actual_moved = abs(pos - previous_pos)
        stall = goal_moved and actual_moved <= self.config.grasp_stall_delta

        return _SideMetrics(
            current_delta=current_delta,
            load_delta=load_delta,
            position_error=position_error,
            angle_lag=angle_lag,
            stall=stall,
        )

    def _contact_from_metrics(self, metrics: _SideMetrics, close_delta: float, timed_out: bool) -> bool:
        angle_contact_enabled = close_delta >= self.config.grasp_contact_min_close_delta
        return (
            metrics.current_delta >= self.config.grasp_contact_current_delta
            or metrics.load_delta >= self.config.grasp_contact_load_delta
            or (
                angle_contact_enabled
                and timed_out
                and (
                    metrics.angle_lag >= self.config.grasp_angle_contact_delta
                    or metrics.position_error >= self.config.grasp_position_error_threshold
                    or metrics.stall
                )
            )
        )

    def _wait_open_ready(self) -> GripperFeedbackState:
        cfg = self.config
        deadline = time.monotonic() + max(0.0, cfg.grasp_open_timeout_s)
        feedback = self.wait_feedback()
        while time.monotonic() < deadline:
            left_ready = feedback.left_pos >= cfg.gripper_open - cfg.grasp_open_tolerance
            right_ready = feedback.right_pos >= cfg.gripper_open - cfg.grasp_open_tolerance
            if left_ready and right_ready:
                return feedback
            self.hold_pair_step(cfg.gripper_open, cfg.gripper_open, cfg.grasp_step_settle_s)
            feedback = self.wait_feedback()
        self.log(
            "Soft grasp open precheck timed out: "
            f"L_pos={feedback.left_pos:.1f} R_pos={feedback.right_pos:.1f}; "
            "continuing with current feedback as baseline."
        )
        return feedback

    def run(self) -> SoftGraspResult:
        cfg = self.config
        min_goal = clamp(cfg.grasp_close_min, 0.0, cfg.gripper_open)

        self.log(
            "Soft grasp: step closing with settle wait; current/load are immediate checks, "
            "angle lag is only contact after step timeout."
        )
        self.open_gripper(5)
        baseline = self._wait_open_ready()
        previous_feedback: GripperFeedbackState | None = baseline

        left_goal = cfg.gripper_open
        right_goal = cfg.gripper_open
        prev_left_goal = left_goal
        prev_right_goal = right_goal
        left_confirm = 0
        right_confirm = 0
        left_contact = False
        right_contact = False
        context: object | None = None

        while not (left_contact and right_contact):
            prev_left_goal = left_goal
            prev_right_goal = right_goal
            if not left_contact:
                left_goal = max(min_goal, left_goal - cfg.grasp_step_size)
            if not right_contact:
                right_goal = max(min_goal, right_goal - cfg.grasp_step_size)

            step_deadline = time.monotonic() + max(cfg.grasp_step_timeout_s, cfg.grasp_step_settle_s)
            feedback = previous_feedback if previous_feedback is not None else baseline
            left_metrics = self._side_metrics(
                feedback=feedback,
                baseline=baseline,
                previous_feedback=previous_feedback,
                side="left",
                goal=left_goal,
                previous_goal=prev_left_goal,
            )
            right_metrics = self._side_metrics(
                feedback=feedback,
                baseline=baseline,
                previous_feedback=previous_feedback,
                side="right",
                goal=right_goal,
                previous_goal=prev_right_goal,
            )

            while True:
                context = self.hold_pair_step(left_goal, right_goal, cfg.grasp_step_settle_s)
                feedback = self.wait_feedback()
                timed_out = time.monotonic() >= step_deadline

                left_metrics = self._side_metrics(
                    feedback=feedback,
                    baseline=baseline,
                    previous_feedback=previous_feedback,
                    side="left",
                    goal=left_goal,
                    previous_goal=prev_left_goal,
                )
                right_metrics = self._side_metrics(
                    feedback=feedback,
                    baseline=baseline,
                    previous_feedback=previous_feedback,
                    side="right",
                    goal=right_goal,
                    previous_goal=prev_right_goal,
                )

                if not left_contact:
                    if self._contact_from_metrics(left_metrics, cfg.gripper_open - left_goal, timed_out):
                        left_confirm += 1
                        left_contact = left_confirm >= cfg.grasp_contact_confirm_steps
                    else:
                        left_confirm = 0
                if not right_contact:
                    if self._contact_from_metrics(right_metrics, cfg.gripper_open - right_goal, timed_out):
                        right_confirm += 1
                        right_contact = right_confirm >= cfg.grasp_contact_confirm_steps
                    else:
                        right_confirm = 0

                left_settled = left_contact or left_metrics.angle_lag <= cfg.grasp_goal_tolerance
                right_settled = right_contact or right_metrics.angle_lag <= cfg.grasp_goal_tolerance
                previous_feedback = feedback
                if (left_settled and right_settled) or timed_out or (left_contact and right_contact):
                    break

            self.log(
                "Soft grasp step: "
                f"L_goal={left_goal:.1f} R_goal={right_goal:.1f} "
                f"L_pos={feedback.left_pos:.1f} R_pos={feedback.right_pos:.1f} "
                f"L_lag={left_metrics.angle_lag:.1f} R_lag={right_metrics.angle_lag:.1f} "
                f"close=({cfg.gripper_open - left_goal:.1f},{cfg.gripper_open - right_goal:.1f}) "
                f"L_load={feedback.left_load:.0f} R_load={feedback.right_load:.0f} "
                f"L_cur={feedback.left_current:.0f} R_cur={feedback.right_current:.0f} "
                f"L_contact={left_contact} R_contact={right_contact}"
            )

            if left_goal <= min_goal and right_goal <= min_goal and not (left_contact and right_contact):
                self.open_gripper(5)
                raise RuntimeError(
                    "Soft grasp reached close limit without confirmed two-sided contact; opened gripper."
                )

        for _ in range(max(0, cfg.grasp_max_balance_steps)):
            feedback = self.wait_feedback()
            left_load = abs(feedback.left_load)
            right_load = abs(feedback.right_load)
            load_diff = left_load - right_load
            angle_diff = feedback.left_pos - feedback.right_pos

            if (
                abs(load_diff) <= cfg.grasp_balance_load_diff
                and abs(angle_diff) <= cfg.grasp_angle_balance_diff
            ):
                break

            if abs(load_diff) > cfg.grasp_balance_load_diff:
                if load_diff > 0.0:
                    right_goal = max(min_goal, right_goal - cfg.grasp_balance_step)
                else:
                    left_goal = max(min_goal, left_goal - cfg.grasp_balance_step)
            elif angle_diff > 0.0:
                left_goal = max(min_goal, left_goal - cfg.grasp_balance_step)
            else:
                right_goal = max(min_goal, right_goal - cfg.grasp_balance_step)

            context = self.hold_pair_step(left_goal, right_goal, cfg.grasp_step_settle_s)
            time.sleep(0.0)

        feedback = self.wait_feedback()
        self.log(
            "Soft grasp finished: "
            f"L_goal={left_goal:.1f} R_goal={right_goal:.1f} "
            f"L_pos={feedback.left_pos:.1f} R_pos={feedback.right_pos:.1f} "
            f"L_load={feedback.left_load:.0f} R_load={feedback.right_load:.0f}."
        )
        self.publish_pair(left_goal, right_goal)
        return SoftGraspResult(
            left_goal=left_goal,
            right_goal=right_goal,
            left_contact=left_contact,
            right_contact=right_contact,
            context=context,
        )
