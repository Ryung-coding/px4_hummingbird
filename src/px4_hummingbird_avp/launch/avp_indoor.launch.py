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
                parameters=[{'config_file': LaunchConfiguration('bridge_config')}],
            ),
            Node(
                package='px4_hummingbird_avp', executable='avp_bridge.py',
                output='screen',
                parameters=[{'use_sim_time': True,
                             'enable_ev': ParameterValue(LaunchConfiguration('enable_ev'),
                                                         value_type=bool)}],
            ),
        ]

    oak_config_file = LaunchConfiguration('oak_config_file').perform(context)
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


def generate_launch_description():
    _hold_launch_lock()
    enable_planner = LaunchConfiguration('enable_planner')
    enable_logger = LaunchConfiguration('enable_logger')
    enable_executor = LaunchConfiguration('enable_executor')
    enable_cmd = LaunchConfiguration('enable_cmd')
    cmd_path = LaunchConfiguration('cmd_path')
    enable_rviz = LaunchConfiguration('enable_rviz')
    auto_offboard = LaunchConfiguration('auto_offboard')
    auto_arm = LaunchConfiguration('auto_arm')
    require_control = LaunchConfiguration('require_control')
    package_config = Path(get_package_share_directory('px4_hummingbird_avp')) / 'config'
    bridge_config = str(package_config / 'bridge.yaml')
    rviz_config = str(package_config / 'avp_indoor.rviz')
    sim_time = ParameterValue(PythonExpression([
        "'", LaunchConfiguration('mode'), "' == 'sim'"]), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim'),
        DeclareLaunchArgument('bridge_config', default_value=bridge_config),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),
        DeclareLaunchArgument('oak_config_file',
                              default_value=str(package_config / 'oak_d_pro.yaml')),
        DeclareLaunchArgument('enable_ev', default_value='false'),
        DeclareLaunchArgument('enable_planner', default_value='true'),
        DeclareLaunchArgument('enable_logger', default_value='false'),
        DeclareLaunchArgument('log_dir', default_value='~/Desktop/px4_hummingbird/src/px4_hummingbird_avp/logs'),
        DeclareLaunchArgument('enable_executor', default_value='false'),
        DeclareLaunchArgument('enable_cmd', default_value='false'),
        DeclareLaunchArgument('cmd_path', default_value='ext'),
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        DeclareLaunchArgument('auto_offboard', default_value='false'),
        DeclareLaunchArgument('auto_arm', default_value='false'),
        DeclareLaunchArgument('require_control', default_value='true'),
        DeclareLaunchArgument('setpoint_topic', default_value='/avp/setpoint'),
        Node(
            package='px4_hummingbird_avp', executable='avp_trajectory_logger.py',
            output='screen', condition=IfCondition(enable_logger),
            parameters=[{'use_sim_time': sim_time, 'log_dir': LaunchConfiguration('log_dir')}],
        ),
        OpaqueFunction(function=_camera_nodes),

        # ============================================================
        # 1. AVP Planner
        # RGB-D PointCloud → Voxel Map → SQC 평가 → 관측 시점 생성
        # 다음 관측 시점(Viewpoint) 후보 생성
        # ============================================================

        Node(
            package='px4_hummingbird_avp', executable='lics_avp_planner.py',
            output='screen', condition=IfCondition(enable_planner),
            parameters=[{
                'use_sim_time': sim_time,

                # [A] ROI: 관측 대상 영역
                # 이 영역 내부의 표면을 대상으로 관측 품질을 평가 / 비행 가능 영역과는 별개
                'roi_cx': 0.0,         # 관측 대상(나무) 중심 X [m], ENU
                'roi_cy': 0.0,         # 관측 대상(나무) 중심 Y [m], ENU

                'roi_r': 1.2,          # ROI 를 구로 생성하였을때 관찰 반경 [m]

                'roi_z_min': 0.5,      # ROI 최소 높이 [m]
                'roi_z_max': 2.5,      # ROI 최대 높이 [m]

                # [B] 관측 거리 및 센서 설정
                'd_opt': 1.0,         # 관측 시점과 표면 사이 목표 거리 [m]
                'd_min': 1.2,         # SQC 거리 평가의 최소 기준 [m]
                'd_max': 1.7,         # SQC 거리 평가의 최대 기준 [m]
                'max_range': 3.5,     # PointCloud 최대 사용 거리 [m] 이후에는 고려 X
                'occ_pub_r_max': 3.0, # 중심 기준 실행기 장애물 전달 범위 [m]
                'max_pitch_deg': 30.0, # 시점 선정 시 허용할 기체 Pitch 절댓값 [deg]
                'vp_safety_margin': 0.5, # 관측 시점 주변 장애물 안전 여유 [m]

                # [C] SQC 가중치
                # SQC = w_dist*Cd + w_angle*Ca + w_base*Cb + w_frontier*Cu
                # Max부분으로 표면 패치를 우선하여 시점 생성
                'w_dist': 1.0,       # 거리 가중치      | ↑ 적정 거리 밖의 표면 우선, ↓ 거리 오차 영향 감소
                'w_angle': 1.0,      # 각도 가중치      | ↑ 비스듬히 관측한 표면 우선, ↓ 관측 각도 영향 감소
                'w_base': 1.0,       # 관측 방향 다양성  | ↑ 다양한 방향에서 덜 관측된 표면 우선, ↓ 방향 다양성 영향 감소
                'w_frontier': 20.0,  # 미관측 경계      | ↑ 미관측 영역 주변 우선, ↓ 기존 관측 영역의 품질 개선에 상대적으로 집중

                'sqc_min': 1.5,      # SQC 후보 기준    | ↑ 후보 감소(높은 점수만), ↓ 후보 증가(낮은 점수도 허용)
                'min_cluster': 8,    # 최소 패치 크기    | ↑ 작은 패치 제외, ↓ 작은 패치도 허용(노이즈 가능성 증가)

                # [D] 경로 재계획
                'plan_period_s': 2.0,  # AVP 계획 계산 주기 [s] / 낮추면 자주 iteration 진행

                # [E] 관측 시점 생성 허용 영역 (Corridor) => Executor의 설정과 동일하게 유지할 것
                'corridor_cx': 0.0,   # 코리더 중심 X [m]
                'corridor_cy': 0.0,   # 코리더 중심 Y [m]

                'corridor_theta_min_deg': 0.0,    # 시작 각도 [deg]
                'corridor_theta_max_deg': 360.0,  # 종료 각도 [deg]

                'corridor_r_min': 1.2,  # 시점 생성 허용 최소 반경 [m]
                'corridor_r_max': 1.7,  # 시점 생성 허용 최대 반경 [m]

                'corridor_z_min': 0.5,  # 시점 생성 허용 최소 높이 [m]
                'corridor_z_max': 2.0,  # 시점 생성 허용 최대 높이 [m]
            }],
        ),

        # ============================================================
        # 2. Corridor Executor
        # 기본 스윕 궤적 생성 + AVP Viewpoint 방문
        # 기체의 위치/속도/자세를 PX4에 전송하여 경로제시
        # ============================================================

        Node(
            package='px4_hummingbird_avp', executable='lics_corridor_executor.py',
            output='screen', condition=IfCondition(enable_executor),
            parameters=[{
                'use_sim_time': sim_time,

                # [A] Planner의 [E] corridor와 일치시킬 것
                'center_x': 0.0,       # 원형 궤적 중심 X [m], ENU
                'center_y': 0.0,       # 원형 궤적 중심 Y [m], ENU

                'theta_min_deg': 0.0,   # 비행 시작 각도 [deg]
                'theta_max_deg': 360.0, # 비행 종료 각도 [deg]

                'r_min': 1.2,         # 비행 허용 최소 반경 [m]
                'r_max': 1.7,         # 비행 허용 최대 반경 [m]

                'z_min': 0.5,         # 비행 허용 최소 높이 [m]
                'z_max': 2.0,         # 비행 허용 최대 높이 [m]

                # [B] 비행 궤적 생성
                'z_levels': [0.7], # 각 진행 경로별 Z [m] / 2개 이상이면 Z 레벨 순회
                'passes': 1,         # 기본 스윕 패스 횟수 (위와 개수맞춰야함)

                'cruise_speed': 0.3,  # 목표 순항 속도 [m/s]
                'accel': 0.3,         # 궤적 속도 가감속 크기 [m/s²]
                'replan_min_s': 15.0,  # 경로 재계획 최소 간격 [s], 기존 기본값 1.5초
                'dwell_s': 5.0,       # AVP 관측 시점 도착 후 정지 시간 [s]


                'cam_pitch_deg': 0.0, # 기본 순항 중 기체 Pitch 명령 [deg] *부호 반대임 주의요함
                'use_vp_pitch': True, # True: AVP 시점에서 계산한 Pitch 사용

                # [C] 장애물 대응 및 추종 안전
                'adaptive_corridor': True, # 장애물에 따라 최소 비행 반경을 자동 조정
                'obs_margin': 0.5,    # 관측된 수관 외측 안전 여유 [m]
                'clear_m': 0.5,       # Setpoint 주변 장애물 검사 반경 [m]
                'arrival_tol_m': 0.25, # 목표 위치 도착 판정 오차 [m]
                'tracking_tol_m': 0.5, # 추종 오차 초과 시 경로 진행 일시 정지

                # [D] PX4 연결
                'setpoint_topic': LaunchConfiguration('setpoint_topic'), # 기본값: /avp/setpoint

                'require_control': ParameterValue(require_control, value_type=bool),
            }],
        ),

        Node(
            package='px4_hummingbird_cmd', executable='px4_position_cmd',
            name='px4_position_cmd', output='screen', condition=IfCondition(enable_cmd),
            parameters=[{
                'path': cmd_path,
                'auto_offboard': ParameterValue(auto_offboard, value_type=bool),
                'auto_arm': ParameterValue(auto_arm, value_type=bool),
            }],
        ),
        Node(
            package='rviz2', executable='rviz2', name='avp_rviz',
            output='screen', condition=IfCondition(enable_rviz),
            arguments=['-d', LaunchConfiguration('rviz_config')], parameters=[{'use_sim_time': sim_time}],
        ),
    ])
