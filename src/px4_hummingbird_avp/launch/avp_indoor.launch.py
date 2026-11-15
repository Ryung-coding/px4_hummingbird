import fcntl
import os
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_launch_lock = None
CONFIG_DIR = Path(get_package_share_directory('px4_hummingbird_avp')) / 'config'


def _hold_launch_lock():
    global _launch_lock
    domain_id = os.environ.get('ROS_DOMAIN_ID', '0')
    lock_path = Path('/tmp') / f'px4_hummingbird_avp_{os.getuid()}_{domain_id}.lock'
    _launch_lock = os.open(lock_path, os.O_CREAT | os.O_WRONLY | os.O_CLOEXEC, 0o600)
    try:
        fcntl.flock(_launch_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        os.close(_launch_lock)
        raise RuntimeError('AVP is already running. Stop the previous AVP launch before starting another.') from exc


def _camera_nodes(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('sim', 'real'):
        raise RuntimeError(f'Unsupported AVP mode: {mode}. Use sim or real.')

    if mode == 'sim':
        return [
            Node(
                package='ros_gz_bridge', executable='parameter_bridge',
                name='avp_gz_bridge', output='screen',
                parameters=[{'config_file': str(CONFIG_DIR / 'bridge.yaml')}],
            ),
            Node(
                package='px4_hummingbird_avp', executable='avp_bridge.py',
                output='screen',
                parameters=[{'use_sim_time': True,
                             'enable_ev': ParameterValue(LaunchConfiguration('enable_ev'),
                                                         value_type=bool)}],
            ),
        ]

    oak_config_file = str(CONFIG_DIR / 'oak_d_pro.yaml')
    with open(oak_config_file, encoding='utf-8') as config_file:
        mount = yaml.safe_load(config_file)['/avp_oak']['ros__parameters']
    if mount['parent_frame'] != 'base_link':
        raise ValueError('OAK parent_frame must be base_link for the current AVP TF tree.')
    pos = mount['position_m']
    rpy = mount['rpy_rad']
    driver_share = get_package_share_directory('depthai_ros_driver_v3')
    driver_launch = str(Path(driver_share) / 'launch' / 'driver.launch.py')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch),
            launch_arguments={
                'name': 'oak',
                'params_file': oak_config_file,
                'parent_frame': mount['parent_frame'],
                'cam_pos_x': str(pos['x']),
                'cam_pos_y': str(pos['y']),
                'cam_pos_z': str(pos['z']),
                'cam_roll': str(rpy['roll']),
                'cam_pitch': str(rpy['pitch']),
                'cam_yaw': str(rpy['yaw']),
                'use_rviz': 'false',
                'publish_tf_from_calibration': 'true',
            }.items(),
        ),
        Node(
            package='px4_hummingbird_avp', executable='oak_pointcloud.py',
            output='screen',
            parameters=[{'depth_image_topic': mount['depth_image_topic'],
                         'camera_info_topic': mount['camera_info_topic'],
                         'sample_step': int(mount['sample_step']),
                         'max_depth_m': float(mount['max_depth_m'])}],
        ),
        Node(
            package='px4_hummingbird_avp', executable='avp_bridge.py',
            output='screen',
            parameters=[{'use_sim_time': False,
                         'enable_ev': False,
                         'publish_camera_tf': False}],
        ),
    ]


def _flight_recorder_nodes(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode == 'sim':
        color_topic = '/rgbd_camera/image'
        depth_topic = '/rgbd_camera/depth_image'
        camera_info_topic = '/rgbd_camera/camera_info'
        video_fps = 10.0
        depth_max_m = 6.0
    else:
        with (CONFIG_DIR / 'oak_d_pro.yaml').open(encoding='utf-8') as config_file:
            mount = yaml.safe_load(config_file)['/avp_oak']['ros__parameters']
        color_topic = mount['color_image_topic']
        depth_topic = mount['depth_image_topic']
        camera_info_topic = mount['camera_info_topic']
        video_fps = float(mount['video_fps'])
        depth_max_m = float(mount['max_depth_m'])

    return [Node(
        package='px4_hummingbird_avp', executable='record_avp_flight.py', output='screen',
        condition=IfCondition(LaunchConfiguration('enable_flight_record')),
        parameters=[{'mode': mode,
                     'use_sim_time': mode == 'sim',
                     'color_topic': color_topic,
                     'depth_topic': depth_topic,
                     'camera_info_topic': camera_info_topic,
                     'video_fps': video_fps,
                     'depth_max_m': depth_max_m}],
    )]


def generate_launch_description():
    _hold_launch_lock()
    enable_planner = LaunchConfiguration('enable_planner')
    enable_logger = LaunchConfiguration('enable_logger')
    enable_executor = LaunchConfiguration('enable_executor')
    enable_cmd = LaunchConfiguration('enable_cmd')
    enable_rviz = LaunchConfiguration('enable_rviz')
    auto_offboard = LaunchConfiguration('auto_offboard')
    auto_arm = LaunchConfiguration('auto_arm')
    sim_time = ParameterValue(PythonExpression([
        "'", LaunchConfiguration('mode'), "' == 'sim'"]), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim'),
        DeclareLaunchArgument('enable_ev', default_value='false'),
        DeclareLaunchArgument('enable_planner', default_value='true'),
        DeclareLaunchArgument('enable_logger', default_value='false'),
        DeclareLaunchArgument('enable_flight_record', default_value='false'),
        DeclareLaunchArgument('enable_executor', default_value='false'),
        DeclareLaunchArgument('enable_cmd', default_value='false'),
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        DeclareLaunchArgument('auto_offboard', default_value='false'),
        DeclareLaunchArgument('auto_arm', default_value='false'),
        Node(
            package='px4_hummingbird_avp', executable='avp_trajectory_logger.py',
            output='screen', condition=IfCondition(enable_logger),
            parameters=[{'use_sim_time': sim_time}],
        ),
        OpaqueFunction(function=_camera_nodes),
        OpaqueFunction(function=_flight_recorder_nodes),

        Node(
            package='px4_hummingbird_avp', executable='lics_avp_planner.py',
            output='screen', condition=IfCondition(enable_planner),
            parameters=[str(CONFIG_DIR / 'avp_indoor.yaml'), {'use_sim_time': sim_time}],
        ),
        Node(
            package='px4_hummingbird_avp', executable='lics_corridor_executor.py',
            output='screen', condition=IfCondition(enable_executor),
            parameters=[str(CONFIG_DIR / 'avp_indoor.yaml'), {'use_sim_time': sim_time}],
        ),

        Node(
            package='px4_hummingbird_cmd', executable='px4_position_cmd',
            name='px4_position_cmd', output='screen', condition=IfCondition(enable_cmd),
            parameters=[{
                'path': 'ext',
                'auto_offboard': ParameterValue(auto_offboard, value_type=bool),
                'auto_arm': ParameterValue(auto_arm, value_type=bool),
            }],
        ),
        Node(
            package='rviz2', executable='rviz2', name='avp_rviz',
            output='screen', condition=IfCondition(enable_rviz),
            arguments=['-d', str(CONFIG_DIR / 'avp_indoor.rviz')], parameters=[{'use_sim_time': sim_time}],
        ),
    ])
