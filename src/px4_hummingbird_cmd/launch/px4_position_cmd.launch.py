from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    path = LaunchConfiguration("path")

    return LaunchDescription([
        DeclareLaunchArgument("path", default_value="pos"),
        Node(
            package="px4_hummingbird_cmd",
            executable="px4_position_cmd",
            name="px4_position_cmd",
            output="screen",
            parameters=[{"path": path}],
        ),
    ])
