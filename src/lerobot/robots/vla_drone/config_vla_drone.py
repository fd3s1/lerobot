#!/usr/bin/env python

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("vla_drone")
@dataclass
class VLADroneConfig(RobotConfig):
    """Configuration for a VLA-controlled drone with a two-motor Feetech gripper."""

    gripper_port: str = "/dev/ttyACM1"
    gripper_left_id: int = 1
    gripper_right_id: int = 2
    gripper_left_inverted: bool = True
    gripper_right_inverted: bool = True
    disable_torque_on_disconnect: bool = True
    configure_gripper_motors: bool = True

    nokov_pose_topic: str = "/mavros/vision_pose/pose"
    mavros_setpoint_topic: str = "/position_cmd"
    ros_node_name: str = "lerobot_vla_drone"
    ros_frame_id: str = "map"
    max_pose_age_s: float = 0.5
    send_pose_actions: bool = True

    x_bounds: tuple[float, float] = (-5.0, 5.0)
    y_bounds: tuple[float, float] = (-5.0, 5.0)
    z_bounds: tuple[float, float] = (0.0, 3.0)
    yaw_bounds: tuple[float, float] = (-3.141592653589793, 3.141592653589793)

    cameras: dict[str, CameraConfig] = field(default_factory=dict)
