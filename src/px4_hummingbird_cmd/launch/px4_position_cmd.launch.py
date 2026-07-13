from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    path = LaunchConfiguration("path")
    auto_offboard = LaunchConfiguration("auto_offboard")
    auto_arm = LaunchConfiguration("auto_arm")
    require_vehicle_status = LaunchConfiguration("require_vehicle_status")
    require_vehicle_local_position = LaunchConfiguration("require_vehicle_local_position")
    require_hb_cmd_source_dds = LaunchConfiguration("require_hb_cmd_source_dds")
    hummingbird_status_topic = LaunchConfiguration("hummingbird_status_topic")
    manual_control_topic = LaunchConfiguration("manual_control_topic")
    vehicle_local_position_topic = LaunchConfiguration("vehicle_local_position_topic")
    vehicle_status_topic = LaunchConfiguration("vehicle_status_topic")
    handoff_duration_sec = LaunchConfiguration("handoff_duration_sec")
    handoff_position_offset_m = LaunchConfiguration("handoff_position_offset_m")
    handoff_yaw_offset_rad = LaunchConfiguration("handoff_yaw_offset_rad")
    command_delay_sec = LaunchConfiguration("command_delay_sec")
    command_repeat_sec = LaunchConfiguration("command_repeat_sec")
    status_timeout_sec = LaunchConfiguration("status_timeout_sec")
    model_name = LaunchConfiguration("model_name")

    return LaunchDescription([
        DeclareLaunchArgument("path", default_value="position_tuning"),
        DeclareLaunchArgument("auto_offboard", default_value="false"),
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("require_vehicle_status", default_value="true"),
        DeclareLaunchArgument("require_vehicle_local_position", default_value="true"),
        DeclareLaunchArgument("require_hb_cmd_source_dds", default_value="true"),
        DeclareLaunchArgument("hummingbird_status_topic", default_value="/fmu/out/hummingbird_status"),
        DeclareLaunchArgument("manual_control_topic", default_value="/fmu/out/manual_control_setpoint"),
        DeclareLaunchArgument("vehicle_local_position_topic", default_value="/fmu/out/vehicle_local_position_v1"),
        DeclareLaunchArgument("vehicle_status_topic", default_value="/fmu/out/vehicle_status_v4"),
        DeclareLaunchArgument("handoff_duration_sec", default_value="2.0"),
        DeclareLaunchArgument("handoff_position_offset_m", default_value="0.6"),
        DeclareLaunchArgument("handoff_yaw_offset_rad", default_value="0.35"),
        DeclareLaunchArgument("command_delay_sec", default_value="2.0"),
        DeclareLaunchArgument("command_repeat_sec", default_value="1.0"),
        DeclareLaunchArgument("status_timeout_sec", default_value="1.0"),
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
                "require_vehicle_status": ParameterValue(require_vehicle_status, value_type=bool),
                "require_vehicle_local_position": ParameterValue(require_vehicle_local_position, value_type=bool),
                "require_hb_cmd_source_dds": ParameterValue(require_hb_cmd_source_dds, value_type=bool),
                "hummingbird_status_topic": hummingbird_status_topic,
                "manual_control_topic": manual_control_topic,
                "vehicle_local_position_topic": vehicle_local_position_topic,
                "vehicle_status_topic": vehicle_status_topic,
                "handoff_duration_sec": ParameterValue(handoff_duration_sec, value_type=float),
                "handoff_position_offset_m": ParameterValue(handoff_position_offset_m, value_type=float),
                "handoff_yaw_offset_rad": ParameterValue(handoff_yaw_offset_rad, value_type=float),
                "command_delay_sec": ParameterValue(command_delay_sec, value_type=float),
                "command_repeat_sec": ParameterValue(command_repeat_sec, value_type=float),
                "status_timeout_sec": ParameterValue(status_timeout_sec, value_type=float),
            }],
        ),
        Node(
            package="px4_hummingbird_cmd",
            executable="px4_servo_to_gz",
            name="px4_servo_to_gz",
            output="screen",
            parameters=[{
                "model_name": model_name,
            }],
        ),
    ])
