from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


_CONFIG_PATH = Path(get_package_share_directory("px4_hummingbird_opti")) / "config" / "opti_params.yaml"


def _load_config():
    with _CONFIG_PATH.open(encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    return config["mocap_bridge"], config["mocap_to_px4"]


def generate_launch_description():
    mocap_params, converter_params = _load_config()

    return LaunchDescription([
        Node(
            package="libmotioncapture",
            executable="mocap_bridge.py",
            name="mocap_bridge",
            output="screen",
            parameters=[mocap_params],
        ),
        Node(
            package="px4_hummingbird_opti",
            executable="mocap_to_px4",
            name="mocap_to_px4",
            output="screen",
            parameters=[converter_params],
        ),
    ])
