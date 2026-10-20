from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


_CONFIG_PATH = Path(get_package_share_directory("px4_hummingbird_opti")) / "config" / "opti_params.yaml"


def _load_config():
    with _CONFIG_PATH.open(encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    return config["vrpn_client"], config["mocap_to_px4"]


def generate_launch_description():
    vrpn_params, converter_params = _load_config()

    return LaunchDescription([
        Node(
            package="vrpn_mocap",
            executable="client_node",
            namespace="nokov_raw",
            name="vrpn_client",
            output="screen",
            parameters=[vrpn_params],
        ),
        Node(
            package="px4_hummingbird_opti",
            executable="mocap_to_px4",
            name="mocap_to_px4",
            output="screen",
            parameters=[converter_params],
        ),
    ])
