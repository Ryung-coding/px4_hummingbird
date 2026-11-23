from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


AVP_SHARE = Path(get_package_share_directory('px4_hummingbird_avp'))


def generate_launch_description():
    auto_offboard = LaunchConfiguration('auto_offboard')
    auto_arm = LaunchConfiguration('auto_arm')
    enable_logger = LaunchConfiguration('enable_logger')
    return LaunchDescription([
        DeclareLaunchArgument('auto_offboard', default_value='true'),
        DeclareLaunchArgument('auto_arm', default_value='false'),
        DeclareLaunchArgument('position_mode', default_value='vision'),
        DeclareLaunchArgument('enable_logger', default_value='false'),
        DeclareLaunchArgument('standoff_m', default_value='1.0'),
        DeclareLaunchArgument('detect_range_m', default_value='2.0'),
        DeclareLaunchArgument('min_altitude_m', default_value='1.0'),
        DeclareLaunchArgument('radius_m', default_value='1.5'),
        DeclareLaunchArgument('max_delta_m', default_value='1.0'),
        DeclareLaunchArgument('out_speed_mps', default_value='0.35'),
        DeclareLaunchArgument('in_speed_mps', default_value='0.12'),
        DeclareLaunchArgument('min_sweep_speed', default_value='0.35'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(AVP_SHARE / 'launch' / 'avp_indoor.launch.py')),
            launch_arguments={
                'mode': 'sim',
                'enable_planner': 'true',
                'require_path_ready': 'true',
                'planner_config': str(AVP_SHARE / 'config' / 'avp_outdoor.yaml'),
                'enable_logger': enable_logger,
                'enable_executor': 'false',
                'enable_cmd': 'false',
                'enable_rviz': 'true',
            }.items(),
        ),
        Node(
            package='px4_hummingbird_avp', executable='avp_radius_sweep.py',
            output='screen', parameters=[{'use_sim_time': True,
                                          **{name: ParameterValue(LaunchConfiguration(name), value_type=float)
                                             for name in ('standoff_m', 'detect_range_m', 'min_altitude_m', 'radius_m', 'max_delta_m',
                                                          'out_speed_mps', 'in_speed_mps', 'min_sweep_speed')}}],
        ),
        Node(
            package='px4_hummingbird_cmd', executable='px4_position_cmd',
            name='px4_position_cmd', output='screen',
            parameters=[{
                'path': 'ext',
                'position_mode': LaunchConfiguration('position_mode'),
                'auto_offboard': ParameterValue(auto_offboard, value_type=bool),
                'auto_arm': ParameterValue(auto_arm, value_type=bool),
            }],
        ),
    ])
