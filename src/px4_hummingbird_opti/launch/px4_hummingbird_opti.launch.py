from typing import List
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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

    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_sub_name = LaunchConfiguration("rviz_sub_name")
    rviz_pub_name = LaunchConfiguration("rviz_pub_name")
    rviz_frame_id = LaunchConfiguration("rviz_frame_id")

    return LaunchDescription([
        DeclareLaunchArgument("host_ip", default_value="202.169.1.197"),
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

        DeclareLaunchArgument("enable_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_sub_name", default_value="/fmu/out/vehicle_local_position"),
        DeclareLaunchArgument("rviz_pub_name", default_value="/rviz/px4_local_pose"),
        DeclareLaunchArgument("rviz_frame_id", default_value="map"),

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

        Node(
            package="px4_hummingbird_opti",
            executable="px4_local_to_rviz",
            name="px4_local_to_rviz",
            output="screen",
            condition=IfCondition(enable_rviz),
            parameters=[{
                "input_topic": rviz_sub_name,
                "output_topic": rviz_pub_name,
                "frame_id": rviz_frame_id,
            }],
        ),
    ])
