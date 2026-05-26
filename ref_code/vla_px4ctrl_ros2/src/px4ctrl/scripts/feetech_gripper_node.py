#!/usr/bin/python3

from __future__ import annotations

import sys
from dataclasses import dataclass

import rclpy
from quadrotor_msgs.msg import GripperCommandPair, GripperFeedback
from rclpy.node import Node
from std_msgs.msg import Float64


LEFT_MOTOR = "gripper_left"
RIGHT_MOTOR = "gripper_right"
DEFAULT_BAUDRATE = 1_000_000

MIN_POSITION_LIMIT = 9
MAX_POSITION_LIMIT = 11
HOMING_OFFSET = 31
OPERATING_MODE = 33
TORQUE_ENABLE = 40
ACCELERATION = 41
GOAL_POSITION = 42
GOAL_VELOCITY = 46
TORQUE_LIMIT = 48
PRESENT_POSITION = 56
PRESENT_LOAD = 60
PRESENT_CURRENT = 69

POSITION_MODE = 0
TORQUE_ENABLED = 1
GOAL_POSITION_SIGN_BIT = 15
PRESENT_POSITION_SIGN_BIT = 15
HOMING_OFFSET_SIGN_BIT = 11
PRESENT_LOAD_SIGN_BIT = 10


@dataclass
class MotorCalibration:
    id: int
    range_min: int
    range_max: int
    homing_offset: int
    inverted: bool = False


@dataclass(frozen=True)
class MotorFeedback:
    position: float
    load: float
    current: float
    position_error: float
    goal_position: float


def encode_sign_magnitude(value: int, sign_bit_index: int) -> int:
    max_magnitude = (1 << sign_bit_index) - 1
    magnitude = abs(int(value))
    if magnitude > max_magnitude:
        raise ValueError(f"Magnitude {magnitude} exceeds {max_magnitude}.")
    return ((1 if value < 0 else 0) << sign_bit_index) | magnitude


def decode_sign_magnitude(value: int, sign_bit_index: int) -> int:
    direction_bit = (int(value) >> sign_bit_index) & 1
    magnitude_mask = (1 << sign_bit_index) - 1
    magnitude = int(value) & magnitude_mask
    return -magnitude if direction_bit else magnitude


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


class MinimalFeetechGripperBus:
    """Small STS3215 gripper bus used by the ROS2 gripper manager."""

    def __init__(
        self,
        port: str,
        left_id: int,
        right_id: int,
        left_inverted: bool,
        right_inverted: bool,
        baudrate: int = DEFAULT_BAUDRATE,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.motor_ids = {LEFT_MOTOR: int(left_id), RIGHT_MOTOR: int(right_id)}
        self.inverted = {LEFT_MOTOR: bool(left_inverted), RIGHT_MOTOR: bool(right_inverted)}
        self.calibration: dict[str, MotorCalibration] = {}
        self._scs = None
        self._port_handler = None
        self._packet_handler = None
        self._comm_success = None

    def connect(self) -> None:
        import scservo_sdk as scs

        self._scs = scs
        self._comm_success = scs.COMM_SUCCESS
        self._port_handler = scs.PortHandler(self.port)
        self._packet_handler = scs.PacketHandler(0)

        if not self._port_handler.openPort():
            raise RuntimeError(f"Failed to open Feetech port {self.port}.")
        if not self._port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f"Failed to set Feetech baudrate to {self.baudrate}.")

        self.calibration = self.read_calibration()

    def disconnect(self) -> None:
        if self._port_handler is not None:
            self._port_handler.closePort()

    def _assert_connected(self) -> None:
        if self._packet_handler is None or self._port_handler is None or self._comm_success is None:
            raise RuntimeError("Feetech bus is not connected.")

    def _check_result(self, comm: int, error: int, context: str) -> None:
        self._assert_connected()
        if comm != self._comm_success:
            raise RuntimeError(f"{context}: {self._packet_handler.getTxRxResult(comm)}")
        if error != 0:
            raise RuntimeError(f"{context}: {self._packet_handler.getRxPacketError(error)}")

    def read1(self, motor_id: int, address: int) -> int:
        self._assert_connected()
        value, comm, error = self._packet_handler.read1ByteTxRx(self._port_handler, motor_id, address)
        self._check_result(comm, error, f"read1 id={motor_id} address={address}")
        return int(value)

    def read2(self, motor_id: int, address: int) -> int:
        self._assert_connected()
        value, comm, error = self._packet_handler.read2ByteTxRx(self._port_handler, motor_id, address)
        self._check_result(comm, error, f"read2 id={motor_id} address={address}")
        return int(value)

    def write1(self, motor_id: int, address: int, value: int) -> None:
        self._assert_connected()
        comm, error = self._packet_handler.write1ByteTxRx(self._port_handler, motor_id, address, int(value))
        self._check_result(comm, error, f"write1 id={motor_id} address={address}")

    def write2(self, motor_id: int, address: int, value: int) -> None:
        self._assert_connected()
        comm, error = self._packet_handler.write2ByteTxRx(self._port_handler, motor_id, address, int(value))
        self._check_result(comm, error, f"write2 id={motor_id} address={address}")

    def read_calibration(self) -> dict[str, MotorCalibration]:
        calibration = {}
        for motor, motor_id in self.motor_ids.items():
            calibration[motor] = MotorCalibration(
                id=motor_id,
                range_min=self.read2(motor_id, MIN_POSITION_LIMIT),
                range_max=self.read2(motor_id, MAX_POSITION_LIMIT),
                homing_offset=decode_sign_magnitude(self.read2(motor_id, HOMING_OFFSET), HOMING_OFFSET_SIGN_BIT),
                inverted=self.inverted[motor],
            )
        return calibration

    def configure_motors(self, *, torque_limit: int = 0, goal_velocity: int = 0, acceleration: int = 0) -> None:
        for motor_id in self.motor_ids.values():
            self.write1(motor_id, OPERATING_MODE, POSITION_MODE)
            if torque_limit > 0:
                self.write2(motor_id, TORQUE_LIMIT, int(torque_limit))
            if goal_velocity > 0:
                self.write2(motor_id, GOAL_VELOCITY, int(goal_velocity))
            if acceleration > 0:
                self.write1(motor_id, ACCELERATION, int(acceleration))
            self.write1(motor_id, TORQUE_ENABLE, TORQUE_ENABLED)

    def normalized_to_raw(self, motor: str, value: float) -> int:
        cal = self.calibration[motor]
        if cal.range_min == cal.range_max:
            raise ValueError(f"Invalid calibration for {motor}: min and max are equal.")

        bounded = clamp(float(value), 0.0, 100.0)
        if cal.inverted:
            bounded = 100.0 - bounded
        raw = int((bounded / 100.0) * (cal.range_max - cal.range_min) + cal.range_min)
        return encode_sign_magnitude(raw, GOAL_POSITION_SIGN_BIT)

    def raw_to_normalized(self, motor: str, raw_value: int) -> float:
        cal = self.calibration[motor]
        raw = decode_sign_magnitude(raw_value, PRESENT_POSITION_SIGN_BIT)
        bounded = min(cal.range_max, max(cal.range_min, raw))
        norm = ((bounded - cal.range_min) / (cal.range_max - cal.range_min)) * 100.0
        return 100.0 - norm if cal.inverted else norm

    def read_normalized_positions(self) -> dict[str, float]:
        return {
            motor: self.raw_to_normalized(motor, self.read2(motor_id, PRESENT_POSITION))
            for motor, motor_id in self.motor_ids.items()
        }

    def write_gripper_pair(self, left: float, right: float) -> None:
        goals = {LEFT_MOTOR: left, RIGHT_MOTOR: right}
        for motor, motor_id in self.motor_ids.items():
            self.write2(motor_id, GOAL_POSITION, self.normalized_to_raw(motor, goals[motor]))

    def read_feedback(self, goal_positions: dict[str, float]) -> dict[str, MotorFeedback]:
        feedback = {}
        for motor, motor_id in self.motor_ids.items():
            position = self.raw_to_normalized(motor, self.read2(motor_id, PRESENT_POSITION))
            load = float(decode_sign_magnitude(self.read2(motor_id, PRESENT_LOAD), PRESENT_LOAD_SIGN_BIT))
            current = float(self.read2(motor_id, PRESENT_CURRENT))
            goal = float(goal_positions[motor])
            feedback[motor] = MotorFeedback(
                position=position,
                load=load,
                current=current,
                position_error=goal - position,
                goal_position=goal,
            )
        return feedback


class FeetechGripperNode(Node):
    def __init__(self) -> None:
        super().__init__("feetech_gripper_node")

        self.command_topic = self.declare_parameter("command_topic", "/gripper/command").value
        self.command_pair_topic = self.declare_parameter("command_pair_topic", "/gripper/command_pair").value
        self.feedback_topic = self.declare_parameter("feedback_topic", "/gripper/feedback").value
        self.feedback_rate_hz = float(self.declare_parameter("feedback_rate_hz", 20.0).value)
        self.port = self.declare_parameter("port", "/dev/ttyACM1").value
        self.left_id = int(self.declare_parameter("left_id", 1).value)
        self.right_id = int(self.declare_parameter("right_id", 2).value)
        self.left_inverted = bool(self.declare_parameter("left_inverted", True).value)
        self.right_inverted = bool(self.declare_parameter("right_inverted", True).value)
        self.no_configure = bool(self.declare_parameter("no_configure", False).value)
        self.dry_run = bool(self.declare_parameter("dry_run", False).value)
        self.torque_limit = int(self.declare_parameter("torque_limit", 0).value)
        self.goal_velocity = int(self.declare_parameter("goal_velocity", 0).value)
        self.acceleration = int(self.declare_parameter("acceleration", 0).value)
        self.open_on_shutdown = bool(self.declare_parameter("open_on_shutdown", True).value)
        self.shutdown_open_position = float(self.declare_parameter("shutdown_open_position", 100.0).value)
        self.shutdown_open_repeats = int(self.declare_parameter("shutdown_open_repeats", 3).value)

        self.goal_positions = {LEFT_MOTOR: 100.0, RIGHT_MOTOR: 100.0}
        self.bus: MinimalFeetechGripperBus | None = None
        if not self.dry_run:
            self._connect_bus()
        else:
            self.get_logger().warn("Running in dry_run mode; Feetech bus will not be opened.")

        self.create_subscription(Float64, self.command_topic, self._command_cb, 10)
        self.create_subscription(GripperCommandPair, self.command_pair_topic, self._command_pair_cb, 10)
        self.feedback_pub = self.create_publisher(GripperFeedback, self.feedback_topic, 10)

        period_s = 1.0 / max(self.feedback_rate_hz, 1e-3)
        self.create_timer(period_s, self._feedback_timer_cb)

        self.get_logger().info(f"Listening for scalar gripper commands on {self.command_topic}")
        self.get_logger().info(f"Listening for pair gripper commands on {self.command_pair_topic}")
        self.get_logger().info(f"Publishing gripper feedback on {self.feedback_topic} at {self.feedback_rate_hz:.1f} Hz")

    def _connect_bus(self) -> None:
        self.bus = MinimalFeetechGripperBus(
            port=self.port,
            left_id=self.left_id,
            right_id=self.right_id,
            left_inverted=self.left_inverted,
            right_inverted=self.right_inverted,
        )
        self.bus.connect()

        if not self.no_configure:
            self.bus.configure_motors(
                torque_limit=self.torque_limit,
                goal_velocity=self.goal_velocity,
                acceleration=self.acceleration,
            )

        for motor, cal in self.bus.calibration.items():
            direction = "inverted" if cal.inverted else "normal"
            self.get_logger().info(
                f"{motor}: id={cal.id}, min={cal.range_min}, max={cal.range_max}, "
                f"homing_offset={cal.homing_offset}, direction={direction}"
            )

    def _command_cb(self, msg: Float64) -> None:
        target = clamp(float(msg.data), 0.0, 100.0)
        self._set_pair(target, target)

    def _command_pair_cb(self, msg: GripperCommandPair) -> None:
        self._set_pair(float(msg.left_pos), float(msg.right_pos))

    def _set_pair(self, left: float, right: float) -> None:
        left = clamp(left, 0.0, 100.0)
        right = clamp(right, 0.0, 100.0)
        self.goal_positions = {LEFT_MOTOR: left, RIGHT_MOTOR: right}

        if self.dry_run:
            self.get_logger().info(f"dry_run gripper target pair: left={left:.1f}, right={right:.1f}")
            return

        if self.bus is None:
            self.get_logger().error("Feetech bus is not connected.")
            return

        try:
            self.bus.write_gripper_pair(left, right)
        except Exception as exc:
            self.get_logger().error(f"Failed to write gripper target pair: {exc}")

    def _feedback_timer_cb(self) -> None:
        if self.dry_run:
            feedback = {
                LEFT_MOTOR: MotorFeedback(self.goal_positions[LEFT_MOTOR], 0.0, 0.0, 0.0, self.goal_positions[LEFT_MOTOR]),
                RIGHT_MOTOR: MotorFeedback(
                    self.goal_positions[RIGHT_MOTOR], 0.0, 0.0, 0.0, self.goal_positions[RIGHT_MOTOR]
                ),
            }
        else:
            if self.bus is None:
                return
            try:
                feedback = self.bus.read_feedback(self.goal_positions)
            except Exception as exc:
                self.get_logger().warn(f"Failed to read gripper feedback: {exc}")
                return

        msg = GripperFeedback()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gripper"
        left = feedback[LEFT_MOTOR]
        right = feedback[RIGHT_MOTOR]
        msg.left_pos = left.position
        msg.right_pos = right.position
        msg.left_load = left.load
        msg.right_load = right.load
        msg.left_current = left.current
        msg.right_current = right.current
        msg.left_position_error = left.position_error
        msg.right_position_error = right.position_error
        msg.left_goal_pos = left.goal_position
        msg.right_goal_pos = right.goal_position
        self.feedback_pub.publish(msg)

    def destroy_node(self) -> bool:
        if self.bus is not None:
            if self.open_on_shutdown:
                target = clamp(self.shutdown_open_position, 0.0, 100.0)
                for _ in range(max(1, self.shutdown_open_repeats)):
                    try:
                        self.bus.write_gripper_pair(target, target)
                    except Exception as exc:
                        self.get_logger().warn(f"Failed to open gripper during shutdown: {exc}")
                        break
            self.bus.disconnect()
            self.bus = None
        return super().destroy_node()


def print_help() -> None:
    print(
        """Feetech STS3215 ROS2 gripper manager.

Subscribes to:
  std_msgs/msg/Float64 on /gripper/command
  quadrotor_msgs/msg/GripperCommandPair on /gripper/command_pair

Publishes:
  quadrotor_msgs/msg/GripperFeedback on /gripper/feedback

Normalized position convention is 100=fully open, 0=fully closed.

Examples:
  ros2 run px4ctrl feetech_gripper_node.py
  ros2 run px4ctrl feetech_gripper_node.py --ros-args -p port:=/dev/ttyACM1
  ros2 run px4ctrl feetech_gripper_node.py --ros-args -p dry_run:=true

Parameters:
  command_topic        default /gripper/command
  command_pair_topic   default /gripper/command_pair
  feedback_topic       default /gripper/feedback
  feedback_rate_hz     default 20.0
  port                 default /dev/ttyACM1
  left_id              default 1
  right_id             default 2
  left_inverted        default true
  right_inverted       default true
  torque_limit         default 0, disabled when <=0
  goal_velocity        default 0, disabled when <=0
  acceleration         default 0, disabled when <=0
  open_on_shutdown     default true
  shutdown_open_position default 100.0
  shutdown_open_repeats  default 3
  no_configure         default false
  dry_run              default false
"""
    )


def main() -> None:
    if any(arg in {"-h", "--help"} for arg in sys.argv[1:]):
        print_help()
        return

    rclpy.init()
    node = FeetechGripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
