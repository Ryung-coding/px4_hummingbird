from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    viewer_log_dir = LaunchConfiguration("viewer_log_dir")
    opti_topic = LaunchConfiguration("opti_topic")
    fmu_odom_topic = LaunchConfiguration("fmu_odom_topic")
    target_body_name = LaunchConfiguration("target_body_name")
    opti_origin = LaunchConfiguration("opti_origin")
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
        DeclareLaunchArgument("viewer_log_dir", default_value="~/Desktop/px4_hummingbird/src/px4_hummingbird_cmd/scripts/viewer_logs"),
        DeclareLaunchArgument("opti_topic", default_value="/opti_raw"),
        DeclareLaunchArgument("fmu_odom_topic", default_value="/fmu/out/vehicle_odometry"),
        DeclareLaunchArgument("target_body_name", default_value="hummingbird"),
        DeclareLaunchArgument("opti_origin", default_value="[1.4, 1.4, 0.0]"),
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
                "opti_topic": opti_topic,
                "fmu_odom_topic": fmu_odom_topic,
                "target_body_name": target_body_name,
                "opti_origin": ParameterValue(opti_origin, value_type=List[float]),
                "replay_log_path": replay_log_path,
            }],
        ),
    ])
