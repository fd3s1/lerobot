#!/usr/bin/env python

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("ros_expert_pose")
@dataclass
class ROSExpertPoseTeleopConfig(TeleoperatorConfig):
    """Teleoperator that records expert actions from ROS2 topics."""

    expert_pose_topic: str = "/px4ctrl/expert_pose"
    gripper_topic: str = "/gripper/command"
    ros_node_name: str = "lerobot_ros_expert_pose_teleop"
    startup_timeout_s: float = 2.0
    max_pose_age_s: float = 0.2
    max_gripper_age_s: float = 0.0
    default_gripper_pos: float = 100.0
    require_gripper_command: bool = False
