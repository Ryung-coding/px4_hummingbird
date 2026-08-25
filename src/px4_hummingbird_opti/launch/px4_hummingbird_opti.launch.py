from typing import List
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    host_ip = LaunchConfiguration("host_ip")
    mocap_ip = LaunchConfiguration("mocap_ip")
    mocap_type = LaunchConfiguration("mocap_type")
    mocap_pub_name = LaunchConfiguration("mocap_pub_name")
    px4_dds_name = LaunchConfiguration("px4_dds_name")
    target_body_name = LaunchConfiguration("target_body_name")
    opti_origin = LaunchConfiguration("opti_origin")
    position_stddev = LaunchConfiguration("position_stddev")
    orientation_stddev_deg = LaunchConfiguration("orientation_stddev_deg")
    quality = LaunchConfiguration("quality")
    max_radius = LaunchConfiguration("max_radius")
    fps = LaunchConfiguration("fps")

    return LaunchDescription([
        DeclareLaunchArgument("host_ip", default_value="192.168.0.5"),
        DeclareLaunchArgument("mocap_ip", default_value="202.169.1.100"),
        DeclareLaunchArgument("mocap_type", default_value="motionanalysis"),
        DeclareLaunchArgument("mocap_pub_name", default_value="/opti_raw"),
        DeclareLaunchArgument("px4_dds_name", default_value="/fmu/in/vehicle_visual_odometry"),
        DeclareLaunchArgument("target_body_name", default_value="hummingbird"),
        DeclareLaunchArgument("opti_origin", default_value="[1.4, 1.4, 0.0]"),
        DeclareLaunchArgument("position_stddev", default_value="0.02"),
        DeclareLaunchArgument("orientation_stddev_deg", default_value="3.0"),
        DeclareLaunchArgument("quality", default_value="100"),
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
                "type": mocap_type,
                "mocap_pub_name": mocap_pub_name,
                "max_radius": ParameterValue(max_radius, value_type=float),
                "fps": ParameterValue(fps, value_type=float),
            }],
        ),

        Node(
            package="px4_hummingbird_opti",
            executable="mocap_to_px4",
            name="mocap_to_px4",
            output="screen",
            parameters=[{
                "input_topic": mocap_pub_name,
                "output_topic": px4_dds_name,
                "target_body_name": target_body_name,
                "opti_origin": ParameterValue(opti_origin, value_type=List[float]),
                "position_stddev": ParameterValue(position_stddev, value_type=float),
                "orientation_stddev_deg": ParameterValue(orientation_stddev_deg, value_type=float),
                "quality": ParameterValue(quality, value_type=int),
            }],
        ),
    ])
