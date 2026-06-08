#!/usr/bin/python3

from __future__ import annotations

import argparse
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
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


def mavros_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
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
        super().__init__("ude_takeoff_hover_test")
        self.args = args

        self.last_seen: dict[str, float] = {}
        self.fsm_state: str | None = None
        self.mavros_state: State | None = None
        self.odom: Odometry | None = None
        self.imu: Imu | None = None
        self.vision_pose: PoseStamped | None = None
        self.attitude_sp: AttitudeTarget | None = None
        self.takeoff_sent = False

        self.takeoff_pub = self.create_publisher(TakeoffLand, args.takeoff_land_topic, 10)
        self.cmd_pub = self.create_publisher(PoseStamped, args.cmd_topic, 10)

        self.create_subscription(String, args.state_topic, self._fsm_state_cb, 10)
        self.create_subscription(State, args.mavros_state_topic, self._mavros_state_cb, 10)
        self.create_subscription(Odometry, args.odom_topic, self._odom_cb, mavros_qos())
        self.create_subscription(Imu, args.imu_topic, self._imu_cb, mavros_qos())
        self.create_subscription(PoseStamped, args.vision_topic, self._vision_pose_cb, mavros_qos())
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
            "vision_pose",
        ]
        if not self.args.skip_setpoint_check:
            required.append("attitude_setpoint")

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

    def wait_for_state(self, target: str, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if self.fsm_state == target:
                return True
            time.sleep(0.05)
        return False

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
        return (not armed) and self.fsm_state in (None, "MANUAL_CTRL")

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
                choice = "EXIT"

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
        description="Independent UDE/bodyrate manual-takeoff AUTO_HOVER test helper."
    )
    parser.add_argument("--state-topic", default="/px4ctrl/state")
    parser.add_argument("--takeoff-land-topic", default="/px4ctrl/takeoff_land")
    parser.add_argument("--cmd-topic", default="/position_cmd")
    parser.add_argument("--mavros-state-topic", default="/mavros/state")
    parser.add_argument("--odom-topic", default="/mavros/local_position/odom")
    parser.add_argument("--imu-topic", default="/mavros/imu/data")
    parser.add_argument("--vision-topic", default="/mavros/vision_pose/pose")
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

        node.print_status()

        if args.publish_takeoff:
            if node.fsm_state != "MANUAL_CTRL":
                node.get_logger().warn(
                    "Publishing TAKEOFF usually expects px4ctrl to start from MANUAL_CTRL."
                )

            print(
                "\nBefore takeoff: keep RC in hover+command mode, sticks centered, "
                "and verify the flight area is clear."
            )
            if args.auto_confirm:
                print("--auto-confirm is set; publishing TAKEOFF.")
            else:
                confirm = input("Type TAKEOFF to publish /px4ctrl/takeoff_land: ").strip()
                if confirm != "TAKEOFF":
                    node.get_logger().warn("Takeoff confirmation not received. Exiting.")
                    return 0

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
            return 0 if node.safe_to_clean_stack() else MAYBE_AIRBORNE_EXIT_CODE

        if args.enter_cmd:
            node.run_optional_cmd_ctrl_test()

        exit_code = node.interactive_after_hover()
        return exit_code
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted.")
        if node.safe_to_clean_stack():
            return 0
        return MAYBE_AIRBORNE_EXIT_CODE
    finally:
        executor.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
