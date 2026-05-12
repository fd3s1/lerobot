#!/usr/bin/python3

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


LEFT_MOTOR = "gripper_left"
RIGHT_MOTOR = "gripper_right"
MOTOR_MODEL = "sts3215"


class FeetechGripperNode(Node):
    def __init__(self) -> None:
        super().__init__("feetech_gripper_node")

        self.command_topic = self.declare_parameter("command_topic", "/drone6/gripper/command").value
        self.port = self.declare_parameter("port", "/dev/ttyACM0").value
        self.left_id = int(self.declare_parameter("left_id", 1).value)
        self.right_id = int(self.declare_parameter("right_id", 2).value)
        self.left_inverted = bool(self.declare_parameter("left_inverted", False).value)
        self.right_inverted = bool(self.declare_parameter("right_inverted", False).value)
        self.no_configure = bool(self.declare_parameter("no_configure", False).value)
        self.dry_run = bool(self.declare_parameter("dry_run", False).value)
        self.lerobot_src = self.declare_parameter(
            "lerobot_src", "/home/user/vla_drone/lerobot/src"
        ).value

        self.bus = None
        if not self.dry_run:
            self._connect_bus()
        else:
            self.get_logger().warn("Running in dry_run mode; Feetech bus will not be opened.")

        self.subscription = self.create_subscription(Float64, self.command_topic, self._command_cb, 10)
        self.get_logger().info(f"Listening for gripper commands on {self.command_topic}")

    def _add_lerobot_to_path(self) -> None:
        candidates = [Path(str(self.lerobot_src))]
        current = Path(__file__).resolve()
        candidates.extend(parent / "src" for parent in current.parents if (parent / "src" / "lerobot").exists())
        for candidate in candidates:
            if (candidate / "lerobot").exists() and str(candidate) not in sys.path:
                sys.path.insert(0, str(candidate))
                return

    def _connect_bus(self) -> None:
        self._add_lerobot_to_path()
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus

        self.bus = FeetechMotorsBus(
            port=self.port,
            motors={
                LEFT_MOTOR: Motor(self.left_id, MOTOR_MODEL, MotorNormMode.RANGE_0_100),
                RIGHT_MOTOR: Motor(self.right_id, MOTOR_MODEL, MotorNormMode.RANGE_0_100),
            },
        )
        self.bus.connect()

        calibration = self.bus.read_calibration()
        if self.left_inverted:
            calibration[LEFT_MOTOR] = replace(calibration[LEFT_MOTOR], drive_mode=1)
        if self.right_inverted:
            calibration[RIGHT_MOTOR] = replace(calibration[RIGHT_MOTOR], drive_mode=1)
        self.bus.calibration = calibration

        if not self.no_configure:
            self.bus.configure_motors()

        for motor, cal in calibration.items():
            direction = "inverted" if cal.drive_mode else "normal"
            self.get_logger().info(
                f"{motor}: id={cal.id}, min={cal.range_min}, max={cal.range_max}, "
                f"homing_offset={cal.homing_offset}, direction={direction}"
            )

    def _command_cb(self, msg: Float64) -> None:
        target = min(100.0, max(0.0, float(msg.data)))
        if self.dry_run:
            self.get_logger().info(f"dry_run gripper target: {target:.1f}")
            return

        if self.bus is None:
            self.get_logger().error("Feetech bus is not connected.")
            return

        self.bus.sync_write(
            "Goal_Position",
            {
                LEFT_MOTOR: target,
                RIGHT_MOTOR: target,
            },
        )
        self.get_logger().info(f"gripper target: {target:.1f}")

    def destroy_node(self) -> bool:
        if self.bus is not None:
            self.bus.disconnect()
            self.bus = None
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = FeetechGripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
