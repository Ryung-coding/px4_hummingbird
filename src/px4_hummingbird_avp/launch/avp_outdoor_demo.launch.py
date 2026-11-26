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
        DeclareLaunchArgument('enable_logger', default_value='false'),
        DeclareLaunchArgument('tilting_mode', default_value='off'),
        DeclareLaunchArgument('standoff_m', default_value='1.0'),
        DeclareLaunchArgument('max_altitude_m', default_value='5.0'),
        DeclareLaunchArgument('total_sweep_s', default_value='80.0'),
        DeclareLaunchArgument('level_sweep_s', default_value='8.0'),
        DeclareLaunchArgument('tilt_angle_deg', default_value='30.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(AVP_SHARE / 'launch' / 'avp_indoor.launch.py')),
            launch_arguments={
                'enable_planner': 'false',
                'enable_logger': enable_logger,
                'record_tilting_mode': LaunchConfiguration('tilting_mode'),
                'enable_executor': 'false',
                'enable_cmd': 'false',
                'enable_rviz': 'false',
            }.items(),
        ),
        Node(
            package='px4_hummingbird_avp', executable='avp_outdoor_demo.py',
            output='screen', parameters=[{'use_sim_time': False,
                                          'tilting_mode': ParameterValue(LaunchConfiguration('tilting_mode'), value_type=str),
                                          **{name: ParameterValue(LaunchConfiguration(name), value_type=float)
                                             for name in ('standoff_m', 'max_altitude_m', 'total_sweep_s', 'level_sweep_s', 'tilt_angle_deg')}}],
        ),
        Node(
            package='px4_hummingbird_cmd', executable='px4_position_cmd',
            name='px4_position_cmd', output='screen',
            parameters=[{
                'path': 'ext',
                'auto_offboard': ParameterValue(auto_offboard, value_type=bool),
                'auto_arm': ParameterValue(auto_arm, value_type=bool),
            }],
        ),
    ])
