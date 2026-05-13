#!/usr/bin/env python

from __future__ import annotations

import logging
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
        cameras: dict[str, Camera] | None = None,
    ):
        super().__init__(config)
        self.config = config
        self.bus = bus or FeetechMotorsBus(
            port=config.gripper_port,
            motors={
                GRIPPER_LEFT: Motor(config.gripper_left_id, "sts3215", MotorNormMode.RANGE_0_100),
                GRIPPER_RIGHT: Motor(config.gripper_right_id, "sts3215", MotorNormMode.RANGE_0_100),
            },
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
        return (
            self.bus.is_connected
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
        self.bus.connect()
        self._read_gripper_calibration()

        for camera in self.cameras.values():
            camera.connect()

        self.pose_bridge.connect()
        self.configure()
        logger.info("%s connected.", self)

    def calibrate(self) -> None:
        if self.bus.is_connected:
            self._read_gripper_calibration()

    def configure(self) -> None:
        if not self.config.configure_gripper_motors:
            return
        self.bus.configure_motors()
        for motor in (GRIPPER_LEFT, GRIPPER_RIGHT):
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    def _read_gripper_calibration(self) -> None:
        calibration = self.bus.read_calibration()
        if self.config.gripper_left_inverted:
            calibration[GRIPPER_LEFT] = replace(calibration[GRIPPER_LEFT], drive_mode=1)
        if self.config.gripper_right_inverted:
            calibration[GRIPPER_RIGHT] = replace(calibration[GRIPPER_RIGHT], drive_mode=1)
        self.bus.calibration = calibration

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        pose = self.pose_bridge.get_latest_pose(self.config.max_pose_age_s)
        if pose is None:
            raise DeviceNotConnectedError(
                f"No fresh Nokov pose received on '{self.config.nokov_pose_topic}' "
                f"within {self.config.max_pose_age_s:.3f}s."
            )

        gripper_pos = self.bus.sync_read("Present_Position", [GRIPPER_LEFT, GRIPPER_RIGHT])
        observation: RobotObservation = {
            ACTION_X: pose.x,
            ACTION_Y: pose.y,
            ACTION_Z: pose.z,
            ACTION_YAW: pose.yaw,
            GRIPPER_LEFT_POS: float(gripper_pos[GRIPPER_LEFT]),
            GRIPPER_RIGHT_POS: float(gripper_pos[GRIPPER_RIGHT]),
        }

        for name, camera in self.cameras.items():
            observation[name] = camera.read_latest()

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

        self.pose_bridge.publish_setpoint(
            safe_action[ACTION_X],
            safe_action[ACTION_Y],
            safe_action[ACTION_Z],
            safe_action[ACTION_YAW],
        )
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
        self.pose_bridge.disconnect()
        for camera in self.cameras.values():
            camera.disconnect()
        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        logger.info("%s disconnected.", self)
