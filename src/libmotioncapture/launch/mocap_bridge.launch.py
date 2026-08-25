from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    host_ip = LaunchConfiguration("host_ip")
    mocap_ip = LaunchConfiguration("mocap_ip")
    mocap_pub_name = LaunchConfiguration("mocap_pub_name")
    max_radius = LaunchConfiguration("max_radius")
    fps = LaunchConfiguration("fps")

    return LaunchDescription([
        DeclareLaunchArgument("host_ip", default_value="192.168.0.105"),
        DeclareLaunchArgument("mocap_ip", default_value="202.169.1.100"),
        DeclareLaunchArgument("mocap_pub_name", default_value="/opti_raw"),
        DeclareLaunchArgument("max_radius", default_value="3.0"),
        DeclareLaunchArgument("fps", default_value="500.0"),

        Node(
            package="libmotioncapture",
            executable="mocap_bridge.py",
            name="mocap_bridge",
            output="screen",
            parameters=[{
                "host_ip": host_ip,
                "mocap_ip": mocap_ip,
                "type": "motionanalysis",
                "mocap_pub_name": mocap_pub_name,
                "max_radius": ParameterValue(max_radius, value_type=float),
                "fps": ParameterValue(fps, value_type=float),
            }],
        ),
    ])
