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
    enable_gz_servo = LaunchConfiguration("enable_gz_servo")
    enable_dynamixel_servo = LaunchConfiguration("enable_dynamixel_servo")
    model_name = LaunchConfiguration("model_name")

    return LaunchDescription([
        DeclareLaunchArgument("path", default_value="position_tuning"),
        DeclareLaunchArgument("auto_offboard", default_value="false"),
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("enable_gz_servo", default_value="true"),
        DeclareLaunchArgument("enable_dynamixel_servo", default_value="false"),
        DeclareLaunchArgument("model_name", default_value="hummingbird_0"),

        Node(
            package="px4_hummingbird_cmd",
            executable="px4_position_cmd",
            name="px4_position_cmd",
            output="screen",
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
    ])