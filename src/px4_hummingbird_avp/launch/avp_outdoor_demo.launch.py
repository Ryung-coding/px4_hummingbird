from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


AVP_SHARE = Path(get_package_share_directory('px4_hummingbird_avp'))


def generate_launch_description():
    auto_offboard = LaunchConfiguration('auto_offboard')
    enable_logger = LaunchConfiguration('enable_logger')
    return LaunchDescription([
        DeclareLaunchArgument('auto_offboard', default_value='true'),
        DeclareLaunchArgument('reference_only', default_value='false'),
        DeclareLaunchArgument('position_mode', default_value='gps'),
        DeclareLaunchArgument('enable_logger', default_value='false'),
        DeclareLaunchArgument('tilting_mode', default_value='off'),
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
                                          'reference_only': ParameterValue(LaunchConfiguration('reference_only'), value_type=bool),
                                          'tilting_mode': ParameterValue(LaunchConfiguration('tilting_mode'), value_type=str),
                                          'position_mode': ParameterValue(LaunchConfiguration('position_mode'), value_type=str)}],
        ),
        Node(
            package='px4_hummingbird_cmd', executable='px4_position_cmd',
            name='px4_position_cmd', output='screen',
            condition=UnlessCondition(LaunchConfiguration('reference_only')),
            parameters=[{
                'path': 'ext',
                'position_mode': ParameterValue(LaunchConfiguration('position_mode'), value_type=str),
                'start_at_current': True,
                'auto_offboard': ParameterValue(auto_offboard, value_type=bool),
                'auto_arm': False,
            }],
        ),
    ])
