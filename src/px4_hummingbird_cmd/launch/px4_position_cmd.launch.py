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


def _viewer_config():
    config_path = Path(get_package_share_directory("px4_hummingbird_opti")) / "config" / "opti_params.yaml"
    with config_path.open(encoding="utf-8") as config_file:
        return yaml.safe_load(config_file)["viewer"]


def _viewer_log_setting(name):
    params_path = Path(get_package_share_directory("px4_hummingbird_cmd")) / "include" / "params.hpp"
    match = re.search(rf'{name}\[\]\s*=\s*"([^"]+)"', params_path.read_text())
    if match is None:
        raise RuntimeError(f"Missing {name} in {params_path}")
    return match.group(1)


def generate_launch_description():
    path = LaunchConfiguration("path")
    auto_offboard = LaunchConfiguration("auto_offboard")
    auto_arm = LaunchConfiguration("auto_arm")
    enable_cmd = LaunchConfiguration("enable_cmd")
    enable_viewer = LaunchConfiguration("enable_viewer")
    viewer_mode = LaunchConfiguration("viewer_mode")
    viewer_log_enabled = LaunchConfiguration("viewer_log_enabled")
    replay_log_path = LaunchConfiguration("replay_log_path")

    return LaunchDescription([
        DeclareLaunchArgument("path", default_value="pos"),
        DeclareLaunchArgument("auto_offboard", default_value="true"),
        DeclareLaunchArgument("auto_arm", default_value="true"),
        DeclareLaunchArgument("enable_cmd", default_value="true"),
        DeclareLaunchArgument("enable_viewer", default_value="false"),
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
            executable="px4_multirotor_viewer.py",
            name="px4_multirotor_viewer",
            output="screen",
            condition=IfCondition(enable_viewer),
            parameters=[{
                "viewer_mode": viewer_mode,
                "log_enabled": ParameterValue(viewer_log_enabled, value_type=bool),
                "log_dir": _viewer_log_setting("viewer_log_dir"),
                "log_filename": _viewer_log_setting("viewer_log_filename"),
                "replay_log_path": replay_log_path,
                **_viewer_config(),
            }],
        ),
    ])
