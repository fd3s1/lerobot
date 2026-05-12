from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("px4ctrl"))
    default_params = package_share / "config" / "ctrl_param_fpv.yaml"
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_params),
                description="Path to the px4ctrl parameter file.",
            ),
            Node(
                package="px4ctrl",
                executable="px4ctrl_node",
                name="px4ctrl",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="px4ctrl",
                executable="feetech_gripper_node.py",
                name="feetech_gripper_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
