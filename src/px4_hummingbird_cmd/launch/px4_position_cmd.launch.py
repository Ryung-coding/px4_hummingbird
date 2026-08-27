import re
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_OPTI_CONFIG_PATH = Path(get_package_share_directory("px4_hummingbird_opti")) / "config" / "opti_params.yaml"
_CMD_PARAMS = (Path(get_package_share_directory("px4_hummingbird_cmd")) / "include" / "params.hpp").read_text()

def _viewer_config():
    with _OPTI_CONFIG_PATH.open(encoding="utf-8") as config_file:
        return yaml.safe_load(config_file)["viewer"]

def _cmd_str(name):
    return re.search(rf"{name}\[\]\s*=\s*\"([^\"]+)\"", _CMD_PARAMS).group(1)

def generate_launch_description():
    path = LaunchConfiguration("path")
    auto_offboard = LaunchConfiguration("auto_offboard")
    auto_arm = LaunchConfiguration("auto_arm")
    enable_cmd = LaunchConfiguration("enable_cmd")
    enable_gz_servo = LaunchConfiguration("enable_gz_servo")
    enable_dynamixel_servo = LaunchConfiguration("enable_dynamixel_servo")
    enable_viewer = LaunchConfiguration("enable_viewer")
    model_name = LaunchConfiguration("model_name")

    viewer_mode = LaunchConfiguration("viewer_mode")
    viewer_log_enabled = LaunchConfiguration("viewer_log_enabled")
    viewer_log_dir = _cmd_str("viewer_log_dir")
    viewer_log_filename = _cmd_str("viewer_log_filename")
    viewer_config = _viewer_config()
    replay_log_path = LaunchConfiguration("replay_log_path")

    return LaunchDescription([
        DeclareLaunchArgument("path", default_value="pos"),
        DeclareLaunchArgument("auto_offboard", default_value="false"),
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("enable_cmd", default_value="true"),
        DeclareLaunchArgument("enable_gz_servo", default_value="false"),
        DeclareLaunchArgument("enable_dynamixel_servo", default_value="false"),
        DeclareLaunchArgument("enable_viewer", default_value="false"),
        DeclareLaunchArgument("model_name", default_value="hummingbird_0"),

        DeclareLaunchArgument("viewer_mode", default_value="sim"),
        DeclareLaunchArgument("viewer_log_enabled", default_value="true"),
        DeclareLaunchArgument("replay_log_path", default_value=""),

        Node(
            package="px4_hummingbird_cmd",
            executable="px4_position_cmd",
            name="px4_position_cmd",
            output="screen",
            condition=IfCondition(enable_cmd),
            parameters=[{
                "path": path,
                "auto_offboard": ParameterValue(auto_offboard, value_type=bool),
                "auto_arm": ParameterValue(auto_arm, value_type=bool),
            }],
        ),

        Node(
            package="px4_hummingbird_cmd",
            executable="px4_servo_to_gz",
            name="px4_servo_to_gz",
            output="screen",
            condition=IfCondition(enable_gz_servo),
            parameters=[{"model_name": model_name}],
        ),

        Node(
            package="px4_hummingbird_cmd",
            executable="px4_servo_to_dynamixel",
            name="px4_servo_to_dynamixel",
            output="screen",
            condition=IfCondition(enable_dynamixel_servo),
        ),

        Node(
            package="px4_hummingbird_cmd",
            executable="px4_multirotor_viewer.py",
            name="px4_multirotor_viewer",
            output="screen",
            condition=IfCondition(enable_viewer),
            parameters=[{
                "viewer_mode": viewer_mode,
                "log_enabled": ParameterValue(viewer_log_enabled, value_type=bool),
                "log_dir": viewer_log_dir,
                "log_filename": viewer_log_filename,
                **viewer_config,
                "replay_log_path": replay_log_path,
            }],
        ),
    ])
