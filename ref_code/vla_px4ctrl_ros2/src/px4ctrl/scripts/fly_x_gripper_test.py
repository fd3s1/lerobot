#!/usr/bin/python3

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import TakeoffLand
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64


DEFAULT_X_MAX_IN_CONFIG = 8.0


@dataclass(frozen=True)
class TestConfig:
    mocap_topic: str
    mocap_msg_type: str
    mocap_discovery_timeout: float
    cmd_topic: str
    gripper_topic: str
    takeoff_land_topic: str
    trigger_topic: str
    frame_id: str
    distance: float
    relative: bool
    target_x: float
    target_y: float | None
    target_z: float | None
    duration: float
    rate_hz: float
    yaw_amplitude: float
    gripper_open: float
    gripper_closed: float
    odom_timeout: float
    start_delay: float
    stable_duration: float
    stable_pos_tolerance: float
    max_start_wait: float
    final_hold: float
    command_stop_before_land: float
    wait_trigger: bool
    trigger_timeout: float
    no_land: bool


@dataclass
class MocapPose:
    x: float
    y: float
    z: float
    yaw: float


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return normalize_angle(math.atan2(siny_cosp, cosy_cosp))


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def interpolate_angle(start: float, end: float, alpha: float) -> float:
    delta = normalize_angle(end - start)
    return normalize_angle(start + delta * smoothstep(alpha))


def yaw_profile(start_yaw: float, progress: float, amplitude: float) -> float:
    progress = max(0.0, min(1.0, progress))
    left_yaw = normalize_angle(start_yaw + amplitude)
    right_yaw = normalize_angle(start_yaw - amplitude)

    if progress < 0.25:
        return interpolate_angle(start_yaw, left_yaw, progress / 0.25)
    if progress < 0.5:
        return interpolate_angle(left_yaw, start_yaw, (progress - 0.25) / 0.25)
    if progress < 0.75:
        return interpolate_angle(start_yaw, right_yaw, (progress - 0.5) / 0.25)
    return interpolate_angle(right_yaw, start_yaw, (progress - 0.75) / 0.25)


class FlyXGripperTest(Node):
    def __init__(self, config: TestConfig) -> None:
        super().__init__("fly_x_gripper_test")
        self.config = config
        self.latest_pose: MocapPose | None = None
        self.trigger_received = False
        self.active_mocap_topic: str | None = None
        self.active_mocap_msg_type: str | None = None
        self.mocap_subscription = None

        self.cmd_pub = self.create_publisher(PoseStamped, config.cmd_topic, 10)
        self.gripper_pub = self.create_publisher(Float64, config.gripper_topic, 10)
        self.takeoff_land_pub = self.create_publisher(TakeoffLand, config.takeoff_land_topic, 10)

        if config.wait_trigger:
            self.create_subscription(PoseStamped, config.trigger_topic, self._trigger_cb, 10)

    def ensure_mocap_subscription(self) -> None:
        if self.mocap_subscription is not None:
            return

        topic, msg_type = self.resolve_mocap_topic()
        if msg_type == "pose":
            self.mocap_subscription = self.create_subscription(PoseStamped, topic, self._pose_cb, 10)
        elif msg_type == "odometry":
            from nav_msgs.msg import Odometry

            self.mocap_subscription = self.create_subscription(Odometry, topic, self._odometry_cb, 10)
        else:
            raise ValueError(f"Unsupported mocap message type: {msg_type}")

        self.active_mocap_topic = topic
        self.active_mocap_msg_type = msg_type
        self.get_logger().info(f"Using Nokov/mocap topic {topic} as {msg_type}.")

    def resolve_mocap_topic(self) -> tuple[str, str]:
        if self.config.mocap_topic != "auto":
            if self.config.mocap_msg_type != "auto":
                return self.config.mocap_topic, self.config.mocap_msg_type
            return self.config.mocap_topic, self.resolve_explicit_topic_type(self.config.mocap_topic)

        deadline = time.monotonic() + self.config.mocap_discovery_timeout
        last_candidates: list[tuple[int, str, str]] = []
        while rclpy.ok() and time.monotonic() < deadline:
            last_candidates = self.find_mocap_topic_candidates()
            if last_candidates:
                best_score = last_candidates[0][0]
                best = [candidate for candidate in last_candidates if candidate[0] == best_score]
                if len(best) == 1:
                    _, topic, msg_type = best[0]
                    return topic, msg_type
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        if last_candidates:
            candidate_text = ", ".join(f"{topic} ({msg_type})" for _, topic, msg_type in last_candidates)
            raise RuntimeError(
                "Multiple possible Nokov/VRPN pose topics found. "
                f"Please choose one with --mocap-topic. Candidates: {candidate_text}"
            )

        raise RuntimeError(
            "No Nokov/VRPN pose topic found. Expected topics look like "
            "/Tracker0/pose or /vrpn_client_node/<tracker_name>/pose. "
            "Check that vrpn_client_ros is running and XINGYING VRPN broadcast is enabled."
        )

    def resolve_explicit_topic_type(self, topic: str) -> str:
        deadline = time.monotonic() + self.config.mocap_discovery_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            topic_types = dict(self.get_topic_names_and_types())
            types = topic_types.get(topic, [])
            if "geometry_msgs/msg/PoseStamped" in types:
                return "pose"
            if "nav_msgs/msg/Odometry" in types:
                return "odometry"
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn(
            f"Could not query type for explicit mocap topic {topic}; assuming geometry_msgs/msg/PoseStamped."
        )
        return "pose"

    def find_mocap_topic_candidates(self) -> list[tuple[int, str, str]]:
        candidates: list[tuple[int, str, str]] = []
        excluded_topics = {
            self.config.cmd_topic,
            self.config.trigger_topic,
            "/drone6/mavros/vision_pose/pose",
        }

        for topic, types in self.get_topic_names_and_types():
            if topic in excluded_topics:
                continue
            if "/mavros/" in topic:
                continue

            msg_type = ""
            if "geometry_msgs/msg/PoseStamped" in types and topic.endswith("/pose"):
                msg_type = "pose"
            elif "nav_msgs/msg/Odometry" in types and topic.endswith("/odom"):
                msg_type = "odometry"
            else:
                continue

            score = self.score_mocap_topic(topic)
            if score <= 0:
                continue
            candidates.append((score, topic, msg_type))

        candidates.sort(key=lambda item: (-item[0], item[1]))
        return candidates

    @staticmethod
    def score_mocap_topic(topic: str) -> int:
        lower_topic = topic.lower()
        basename = topic.rstrip("/").split("/")[-2] if topic.endswith("/pose") else topic.rstrip("/").split("/")[-1]
        score = 0
        if "/vrpn_client_node/" in topic:
            score += 100
        if basename.startswith("Tracker") or basename.startswith("U_Tracker") or "tracker" in basename.lower():
            score += 80
        if "nokov" in lower_topic or "mocap" in lower_topic or "vrpn" in lower_topic:
            score += 50
        if "drone" in lower_topic:
            score += 20
        return score

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.latest_pose = MocapPose(
            x=float(msg.pose.position.x),
            y=float(msg.pose.position.y),
            z=float(msg.pose.position.z),
            yaw=quaternion_to_yaw(msg),
        )

    def _odometry_cb(self, msg) -> None:
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        self._pose_cb(pose_msg)

    def _trigger_cb(self, _msg: PoseStamped) -> None:
        self.trigger_received = True

    def wait_for_odom(self) -> MocapPose:
        self.ensure_mocap_subscription()
        deadline = time.monotonic() + self.config.odom_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_pose is not None:
                return self.latest_pose

        raise RuntimeError(
            f"No Nokov/mocap pose received on {self.active_mocap_topic or self.config.mocap_topic} "
            f"within {self.config.odom_timeout:.1f}s."
        )

    def wait_for_stable_pose(self) -> MocapPose:
        self.get_logger().info(
            "Waiting for stable mocap pose before capturing the test start pose "
            f"({self.config.stable_duration:.1f}s stable window)."
        )
        reference = self.wait_for_odom()
        stable_since = time.monotonic()
        deadline = time.monotonic() + self.config.max_start_wait

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_pose is None:
                continue

            current = self.latest_pose
            delta = math.sqrt(
                (current.x - reference.x) ** 2
                + (current.y - reference.y) ** 2
                + (current.z - reference.z) ** 2
            )
            if delta > self.config.stable_pos_tolerance:
                reference = current
                stable_since = time.monotonic()
                continue

            if time.monotonic() - stable_since >= self.config.stable_duration:
                return current

        raise RuntimeError(
            "Mocap pose did not become stable before starting the test. "
            "If the drone is intentionally moving, increase --stable-pos-tolerance "
            "or --max-start-wait."
        )

    def wait_for_trigger(self) -> None:
        if not self.config.wait_trigger:
            return

        self.get_logger().info(f"Waiting for controller trigger on {self.config.trigger_topic}")
        deadline = time.monotonic() + self.config.trigger_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.trigger_received:
                self.get_logger().info("Controller trigger received.")
                return

        raise RuntimeError(
            f"No trigger received on {self.config.trigger_topic} "
            f"within {self.config.trigger_timeout:.1f}s."
        )

    def spin_sleep(self, seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, seconds)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def publish_gripper(self, target: float, repeats: int = 3, interval_s: float = 0.05) -> None:
        target = max(0.0, min(100.0, target))
        msg = Float64()
        msg.data = target
        for _ in range(max(1, repeats)):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(interval_s)

    def publish_cmd(self, x: float, y: float, z: float, yaw: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.config.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.cmd_pub.publish(msg)

    def publish_land(self, repeats: int = 5, interval_s: float = 0.2) -> None:
        msg = TakeoffLand()
        msg.takeoff_land_cmd = TakeoffLand.LAND
        for _ in range(max(1, repeats)):
            self.publish_gripper(self.config.gripper_open, repeats=1, interval_s=0.0)
            self.takeoff_land_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(interval_s)

    def run_sequence(self) -> None:
        self.wait_for_odom()
        self.wait_for_trigger()

        if self.config.start_delay > 0.0:
            self.get_logger().info(f"Waiting {self.config.start_delay:.1f}s before capturing start pose.")
            self.spin_sleep(self.config.start_delay)

        start = self.wait_for_stable_pose()
        if self.config.relative:
            target_x = start.x + self.config.distance
        else:
            target_x = self.config.target_x
        target_y = start.y if self.config.target_y is None else self.config.target_y
        target_z = start.z if self.config.target_z is None else self.config.target_z

        delta_x = target_x - start.x
        delta_y = target_y - start.y
        delta_z = target_z - start.z

        self.get_logger().info(
            f"Reading Nokov/mocap pose from {self.active_mocap_topic} "
            f"as {self.active_mocap_msg_type}."
        )
        self.get_logger().info(
            "Start pose: "
            f"x={start.x:.3f}, y={start.y:.3f}, z={start.z:.3f}, yaw={start.yaw:.3f}"
        )
        self.get_logger().info(
            "Target: "
            f"x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}, "
            f"delta=({delta_x:.3f}, {delta_y:.3f}, {delta_z:.3f})m"
        )

        if target_x > DEFAULT_X_MAX_IN_CONFIG:
            self.get_logger().warn(
                f"Default ctrl_param_fpv.yaml has limits.x_max={DEFAULT_X_MAX_IN_CONFIG:.1f}. "
                "If your controller config is lower than this target, px4ctrl will clamp the test."
            )
        if delta_x <= 0.0:
            self.get_logger().warn(
                "Target x is not greater than the current Nokov x. "
                "This test is intended to fly along mocap +X."
            )

        self.publish_gripper(self.config.gripper_open, repeats=5)

        events = [
            (0.12, self.config.gripper_closed, "gripper cycle 1 close"),
            (0.22, self.config.gripper_open, "gripper cycle 1 open"),
            (0.38, self.config.gripper_closed, "gripper cycle 2 close"),
            (0.48, self.config.gripper_open, "gripper cycle 2 open"),
            (0.64, self.config.gripper_closed, "gripper cycle 3 close"),
            (0.74, self.config.gripper_open, "gripper cycle 3 open"),
        ]
        next_event = 0

        period = 1.0 / self.config.rate_hz
        started_at = time.monotonic()
        next_tick = started_at

        while rclpy.ok():
            now = time.monotonic()
            elapsed = now - started_at
            progress = min(1.0, elapsed / self.config.duration)
            pos_alpha = smoothstep(progress)
            x = start.x + delta_x * pos_alpha
            y = start.y + delta_y * pos_alpha
            z = start.z + delta_z * pos_alpha
            yaw = yaw_profile(start.yaw, progress, self.config.yaw_amplitude)

            while next_event < len(events) and progress >= events[next_event][0]:
                _, gripper_target, label = events[next_event]
                self.get_logger().info(label)
                self.publish_gripper(gripper_target)
                next_event += 1

            self.publish_cmd(x, y, z, yaw)
            rclpy.spin_once(self, timeout_sec=0.0)

            if progress >= 1.0:
                break

            next_tick += period
            sleep_time = max(0.0, next_tick - time.monotonic())
            time.sleep(sleep_time)

        self.get_logger().info("Final hold with gripper open.")
        self.publish_gripper(self.config.gripper_open, repeats=5)
        hold_started_at = time.monotonic()
        while rclpy.ok() and time.monotonic() - hold_started_at < self.config.final_hold:
            self.publish_cmd(target_x, target_y, target_z, start.yaw)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        self.publish_gripper(self.config.gripper_open, repeats=5)

        if self.config.no_land:
            self.get_logger().warn("Skipping LAND because --no-land was set.")
            return

        self.get_logger().info(
            "Stopping position commands before LAND so px4ctrl can return from CMD_CTRL to AUTO_HOVER."
        )
        self.spin_sleep(self.config.command_stop_before_land)
        self.publish_gripper(self.config.gripper_open, repeats=5)
        self.get_logger().info("Publishing LAND command with gripper open.")
        self.publish_land()

    def emergency_open_and_land(self) -> None:
        self.get_logger().warn("Interrupted. Opening gripper and publishing LAND.")
        self.publish_gripper(self.config.gripper_open, repeats=5)
        if not self.config.no_land:
            self.spin_sleep(self.config.command_stop_before_land)
            self.publish_land()


def parse_args() -> TestConfig:
    parser = argparse.ArgumentParser(
        description=(
            "After px4ctrl takeoff, read Nokov/mocap pose directly, fly to a target in that frame, "
            "sweep yaw left/right, cycle the Feetech gripper three times, then land open."
        )
    )
    parser.add_argument(
        "--mocap-topic",
        "--odom-topic",
        dest="mocap_topic",
        default="auto",
        help="Nokov/VRPN pose topic. Default auto-discovers /Tracker*/pose or /vrpn_client_node/*/pose.",
    )
    parser.add_argument("--mocap-msg-type", choices=("auto", "pose", "odometry"), default="auto")
    parser.add_argument("--mocap-discovery-timeout", type=float, default=5.0)
    parser.add_argument("--cmd-topic", default="/drone6/position_cmd")
    parser.add_argument("--gripper-topic", default="/drone6/gripper/command")
    parser.add_argument("--takeoff-land-topic", default="/drone6/px4ctrl/takeoff_land")
    parser.add_argument("--trigger-topic", default="/drone6/traj_start_trigger")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--distance", type=float, default=7.0)
    parser.add_argument(
        "--relative",
        action="store_true",
        help="Use start_x + --distance. Default is absolute target x=--target-x.",
    )
    parser.add_argument("--target-x", type=float, default=7.0)
    parser.add_argument("--target-y", type=float, default=None)
    parser.add_argument("--target-z", type=float, default=None)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--yaw-amplitude", type=float, default=1.0)
    parser.add_argument("--gripper-open", type=float, default=100.0)
    parser.add_argument("--gripper-closed", type=float, default=0.0)
    parser.add_argument("--odom-timeout", type=float, default=10.0)
    parser.add_argument("--start-delay", type=float, default=2.0)
    parser.add_argument("--stable-duration", type=float, default=2.0)
    parser.add_argument("--stable-pos-tolerance", type=float, default=0.05)
    parser.add_argument("--max-start-wait", type=float, default=30.0)
    parser.add_argument("--final-hold", type=float, default=2.0)
    parser.add_argument("--command-stop-before-land", type=float, default=1.2)
    parser.add_argument("--wait-trigger", action="store_true")
    parser.add_argument("--trigger-timeout", type=float, default=30.0)
    parser.add_argument("--no-land", action="store_true")

    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    if args.duration <= 0.0:
        raise ValueError("--duration must be positive.")
    if args.rate_hz <= 0.0:
        raise ValueError("--rate-hz must be positive.")
    if args.command_stop_before_land < 0.6:
        raise ValueError("--command-stop-before-land must be at least 0.6s.")
    if args.stable_duration < 0.0:
        raise ValueError("--stable-duration must be non-negative.")
    if args.stable_pos_tolerance <= 0.0:
        raise ValueError("--stable-pos-tolerance must be positive.")
    if args.max_start_wait <= 0.0:
        raise ValueError("--max-start-wait must be positive.")
    if args.mocap_discovery_timeout <= 0.0:
        raise ValueError("--mocap-discovery-timeout must be positive.")

    return TestConfig(
        mocap_topic=args.mocap_topic,
        mocap_msg_type=args.mocap_msg_type,
        mocap_discovery_timeout=args.mocap_discovery_timeout,
        cmd_topic=args.cmd_topic,
        gripper_topic=args.gripper_topic,
        takeoff_land_topic=args.takeoff_land_topic,
        trigger_topic=args.trigger_topic,
        frame_id=args.frame_id,
        distance=args.distance,
        relative=args.relative,
        target_x=args.target_x,
        target_y=args.target_y,
        target_z=args.target_z,
        duration=args.duration,
        rate_hz=args.rate_hz,
        yaw_amplitude=args.yaw_amplitude,
        gripper_open=args.gripper_open,
        gripper_closed=args.gripper_closed,
        odom_timeout=args.odom_timeout,
        start_delay=args.start_delay,
        stable_duration=args.stable_duration,
        stable_pos_tolerance=args.stable_pos_tolerance,
        max_start_wait=args.max_start_wait,
        final_hold=args.final_hold,
        command_stop_before_land=args.command_stop_before_land,
        wait_trigger=args.wait_trigger,
        trigger_timeout=args.trigger_timeout,
        no_land=args.no_land,
    )


def main() -> None:
    config = parse_args()
    rclpy.init()
    node = FlyXGripperTest(config)
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        node.emergency_open_and_land()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
