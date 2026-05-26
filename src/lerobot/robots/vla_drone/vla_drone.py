#!/usr/bin/env python

from __future__ import annotations

import logging
import time
from dataclasses import replace
from functools import cached_property
from typing import Protocol

from lerobot.cameras import Camera, make_cameras_from_configs
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots.robot import Robot
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.errors import DeviceNotConnectedError

from .config_vla_drone import VLADroneConfig
from .ros2_gripper_bridge import GripperFeedbackSample, ROS2GripperBridge
from .ros2_bridge import DronePose, ROS2PoseBridge

logger = logging.getLogger(__name__)

ACTION_X = "x"
ACTION_Y = "y"
ACTION_Z = "z"
ACTION_YAW = "yaw"
GRIPPER_LEFT = "gripper_left"
GRIPPER_RIGHT = "gripper_right"
GRIPPER_LEFT_POS = f"{GRIPPER_LEFT}.pos"
GRIPPER_RIGHT_POS = f"{GRIPPER_RIGHT}.pos"


class GripperBus(Protocol):
    is_connected: bool
    motors: dict
    calibration: dict

    def connect(self) -> None: ...
    def disconnect(self, disable_torque: bool = True) -> None: ...
    def configure_motors(self) -> None: ...
    def read_calibration(self) -> dict: ...
    def sync_read(self, data_name: str, motors=None, *, normalize: bool = True, num_retry: int = 0) -> dict: ...
    def sync_write(self, data_name: str, values, *, normalize: bool = True, num_retry: int = 0) -> None: ...
    def write(self, data_name: str, motor: str, value, *, normalize: bool = True, num_retry: int = 0) -> None: ...


class PoseBridge(Protocol):
    is_connected: bool

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def get_latest_pose(self, max_age_s: float) -> DronePose | None: ...
    def publish_setpoint(self, x: float, y: float, z: float, yaw: float) -> None: ...


class GripperBridge(Protocol):
    is_connected: bool

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def get_latest_feedback(self, max_age_s: float) -> GripperFeedbackSample | None: ...
    def publish_pair(self, left: float, right: float) -> None: ...


def clamp(value: float, bounds: tuple[float, float]) -> float:
    low, high = bounds
    return min(high, max(low, value))


class VLADrone(Robot):
    config_class = VLADroneConfig
    name = "vla_drone"

    def __init__(
        self,
        config: VLADroneConfig,
        *,
        bus: GripperBus | None = None,
        pose_bridge: PoseBridge | None = None,
        gripper_bridge: GripperBridge | None = None,
        cameras: dict[str, Camera] | None = None,
    ):
        super().__init__(config)
        self.config = config
        self.bus = None if config.use_ros_gripper else (
            bus or FeetechMotorsBus(
                port=config.gripper_port,
                motors={
                    GRIPPER_LEFT: Motor(config.gripper_left_id, "sts3215", MotorNormMode.RANGE_0_100),
                    GRIPPER_RIGHT: Motor(config.gripper_right_id, "sts3215", MotorNormMode.RANGE_0_100),
                },
            )
        )
        self.gripper_bridge = gripper_bridge or (
            ROS2GripperBridge(
                node_name=f"{config.ros_node_name}_gripper",
                command_topic=config.gripper_command_topic,
                feedback_topic=config.gripper_feedback_topic,
            )
            if config.use_ros_gripper
            else None
        )
        self.pose_bridge = pose_bridge or ROS2PoseBridge(
            node_name=config.ros_node_name,
            pose_topic=config.nokov_pose_topic,
            setpoint_topic=config.mavros_setpoint_topic,
            frame_id=config.ros_frame_id,
        )
        self.cameras = cameras if cameras is not None else make_cameras_from_configs(config.cameras)

    @property
    def is_connected(self) -> bool:
        gripper_connected = (
            self.gripper_bridge.is_connected
            if self.config.use_ros_gripper and self.gripper_bridge is not None
            else self.bus is not None and self.bus.is_connected
        )
        return (
            gripper_connected
            and self.pose_bridge.is_connected
            and all(camera.is_connected for camera in self.cameras.values())
        )

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        features: dict[str, type | tuple] = {
            ACTION_X: float,
            ACTION_Y: float,
            ACTION_Z: float,
            ACTION_YAW: float,
            GRIPPER_LEFT_POS: float,
            GRIPPER_RIGHT_POS: float,
        }
        features.update(
            {
                name: (camera.height, camera.width, 3)
                for name, camera in self.cameras.items()
            }
        )
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {
            ACTION_X: float,
            ACTION_Y: float,
            ACTION_Z: float,
            ACTION_YAW: float,
            GRIPPER_LEFT_POS: float,
            GRIPPER_RIGHT_POS: float,
        }

    @property
    def is_calibrated(self) -> bool:
        return True

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        if self.config.use_ros_gripper:
            if self.gripper_bridge is None:
                raise RuntimeError("ROS gripper bridge is not configured.")
            self.gripper_bridge.connect()
        else:
            if self.bus is None:
                raise RuntimeError("Feetech gripper bus is not configured.")
            self.bus.connect()
            self._read_gripper_calibration()

        for camera in self.cameras.values():
            camera.connect()

        self.pose_bridge.connect()
        self.configure()
        logger.info("%s connected.", self)

    def calibrate(self) -> None:
        if self.bus is not None and self.bus.is_connected:
            self._read_gripper_calibration()

    def configure(self) -> None:
        if self.config.use_ros_gripper:
            return
        if not self.config.configure_gripper_motors:
            return
        if self.bus is None:
            raise RuntimeError("Feetech gripper bus is not configured.")
        self.bus.configure_motors()
        for motor in (GRIPPER_LEFT, GRIPPER_RIGHT):
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    def _read_gripper_calibration(self) -> None:
        if self.bus is None:
            raise RuntimeError("Feetech gripper bus is not configured.")
        calibration = self.bus.read_calibration()
        if self.config.gripper_left_inverted:
            calibration[GRIPPER_LEFT] = replace(calibration[GRIPPER_LEFT], drive_mode=1)
        if self.config.gripper_right_inverted:
            calibration[GRIPPER_RIGHT] = replace(calibration[GRIPPER_RIGHT], drive_mode=1)
        self.bus.calibration = calibration

    def open_gripper_for_safety(self, reason: str = "safety") -> None:
        if self.config.use_ros_gripper:
            if self.gripper_bridge is None or not self.gripper_bridge.is_connected:
                return
        elif self.bus is None or not self.bus.is_connected:
            return

        open_position = clamp(self.config.disconnect_gripper_open_position, (0.0, 100.0))
        repeats = max(1, int(self.config.disconnect_gripper_repeats))
        for _ in range(repeats):
            if self.config.use_ros_gripper:
                self.gripper_bridge.publish_pair(open_position, open_position)
            else:
                self.bus.sync_write(
                    "Goal_Position",
                    {
                        GRIPPER_LEFT: open_position,
                        GRIPPER_RIGHT: open_position,
                    },
                    num_retry=1,
                )
            if repeats > 1:
                time.sleep(0.05)

        if self.config.disconnect_gripper_settle_s > 0.0:
            time.sleep(self.config.disconnect_gripper_settle_s)
        logger.info("Opened gripper to %.1f for %s.", open_position, reason)

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        camera_observation: RobotObservation = {}
        for name, camera in self.cameras.items():
            camera_observation[name] = camera.read_latest()

        if self.config.use_ros_gripper:
            if self.gripper_bridge is None:
                raise DeviceNotConnectedError("ROS gripper bridge is not configured.")
            gripper_feedback = self.gripper_bridge.get_latest_feedback(self.config.gripper_feedback_timeout_s)
            if gripper_feedback is None:
                raise DeviceNotConnectedError(
                    f"No fresh gripper feedback received on '{self.config.gripper_feedback_topic}' "
                    f"within {self.config.gripper_feedback_timeout_s:.3f}s."
                )
            gripper_left_pos = gripper_feedback.left_pos
            gripper_right_pos = gripper_feedback.right_pos
        else:
            if self.bus is None:
                raise DeviceNotConnectedError("Feetech gripper bus is not configured.")
            gripper_pos = self.bus.sync_read("Present_Position", [GRIPPER_LEFT, GRIPPER_RIGHT])
            gripper_left_pos = float(gripper_pos[GRIPPER_LEFT])
            gripper_right_pos = float(gripper_pos[GRIPPER_RIGHT])

        # Read pose last so observation.state is as close as possible to the action sample
        # taken immediately after this method returns in the LeRobot record loop.
        pose = self.pose_bridge.get_latest_pose(self.config.max_pose_age_s)
        if pose is None:
            raise DeviceNotConnectedError(
                f"No fresh Nokov pose received on '{self.config.nokov_pose_topic}' "
                f"within {self.config.max_pose_age_s:.3f}s."
            )

        observation: RobotObservation = {
            ACTION_X: pose.x,
            ACTION_Y: pose.y,
            ACTION_Z: pose.z,
            ACTION_YAW: pose.yaw,
            GRIPPER_LEFT_POS: gripper_left_pos,
            GRIPPER_RIGHT_POS: gripper_right_pos,
        }
        observation.update(camera_observation)

        return observation

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        safe_action = {
            ACTION_X: clamp(float(action.get(ACTION_X, 0.0)), self.config.x_bounds),
            ACTION_Y: clamp(float(action.get(ACTION_Y, 0.0)), self.config.y_bounds),
            ACTION_Z: clamp(float(action.get(ACTION_Z, 0.0)), self.config.z_bounds),
            ACTION_YAW: clamp(float(action.get(ACTION_YAW, 0.0)), self.config.yaw_bounds),
            GRIPPER_LEFT_POS: clamp(float(action.get(GRIPPER_LEFT_POS, 0.0)), (0.0, 100.0)),
            GRIPPER_RIGHT_POS: clamp(float(action.get(GRIPPER_RIGHT_POS, 0.0)), (0.0, 100.0)),
        }

        if self.config.send_pose_actions:
            self.pose_bridge.publish_setpoint(
                safe_action[ACTION_X],
                safe_action[ACTION_Y],
                safe_action[ACTION_Z],
                safe_action[ACTION_YAW],
            )
        if self.config.use_ros_gripper:
            if self.gripper_bridge is None:
                raise RuntimeError("ROS gripper bridge is not configured.")
            self.gripper_bridge.publish_pair(
                safe_action[GRIPPER_LEFT_POS],
                safe_action[GRIPPER_RIGHT_POS],
            )
        else:
            if self.bus is None:
                raise RuntimeError("Feetech gripper bus is not configured.")
            self.bus.sync_write(
                "Goal_Position",
                {
                    GRIPPER_LEFT: safe_action[GRIPPER_LEFT_POS],
                    GRIPPER_RIGHT: safe_action[GRIPPER_RIGHT_POS],
                },
            )
        return safe_action

    @check_if_not_connected
    def disconnect(self) -> None:
        if self.config.safe_open_gripper_on_disconnect:
            try:
                self.open_gripper_for_safety("disconnect")
            except Exception as exc:
                logger.warning("Failed to open gripper before disconnect: %s", exc)

        self.pose_bridge.disconnect()
        for camera in self.cameras.values():
            camera.disconnect()
        if self.config.use_ros_gripper:
            if self.gripper_bridge is not None:
                self.gripper_bridge.disconnect()
        elif self.bus is not None:
            self.bus.disconnect(self.config.disable_torque_on_disconnect)
        logger.info("%s disconnected.", self)
