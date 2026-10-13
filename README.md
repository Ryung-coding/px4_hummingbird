# PX4 HummingBird

PX4 v1.17 기반 HummingBird 비행제어 및 Gazebo SITL 환경입니다. Underactuated / Fully Actuated 제어와 RC / DDS 명령 입력을 지원합니다.

## 1. Simulation 실행

`HB_v1.17` 브랜치의 **PX4 프로젝트 루트 디렉터리**에서 실행합니다.

```bash
make px4_sitl gz_hummingbird
```

별도 터미널에서 **QGroundControl (QGC)** 을 실행하여 기체 상태를 확인합니다.

## 2. ROS 2 연동

[ros2-ws 브랜치](https://github.com/Ryung-coding/px4_hummingbird/tree/ros2-ws)를 별도로 다운로드하고 빌드합니다. ROS 2 Jazzy 환경에서 다음을 각각 실행합니다.

**DDS Agent** (`ros2-ws` 루트):

```bash
./run_agent.sh
```

**ROS 2 Command + Gazebo Servo Bridge** (`ros2-ws` 환경 설정 후):

```bash
source install/setup.bash
ros2 launch px4_hummingbird_cmd auto_run_gz.launch.py path:=pos
```

지원하는 `path` 옵션:

| Path | 설명 |
| --- | --- |
| `pos` | Position Trajectory |
| `att` | Attitude Trajectory |
| `step_att` | Attitude Step Response |

실행 예시:

```bash
ros2 launch px4_hummingbird_cmd auto_run_gz.launch.py path:=att
ros2 launch px4_hummingbird_cmd auto_run_gz.launch.py path:=step_att
```

ROS 2 Workspace는 PX4의 **EKF Position**을 수신하고, **Position / Attitude Command**를 전달하며, **Servo Angle**을 받아 Gazebo로 출력합니다.

## 3. HummingBird Parameters

QGC의 **Parameters** 메뉴에서 `HB_`를 검색하여 설정합니다.

| Parameter | 기능 |
| --- | --- |
| `HB_CTRL_MODE` | Underactuated / Fully Actuated 제어 선택 |
| `HB_CMD_SOURCE` | RC / DDS 명령 입력 선택 |

- **Underactuated:** 일반 Quadrotor처럼 자세를 기울여 위치를 제어합니다.
- **Fully Actuated:** Tilt Servo를 활용하여 위치와 자세를 독립적으로 제어합니다.

기본 설정은 `HB_CTRL_MODE = 0`, `HB_CMD_SOURCE = 0`입니다.

## 4. DDS Command

ROS 2에서 사용하는 주요 PX4 Topic은 다음과 같습니다.

| Topic | 기능 |
| --- | --- |
| `/fmu/in/offboard_control_mode` | Offboard 제어 모드 지정 |
| `/fmu/in/trajectory_setpoint6dof` | Position / Attitude Setpoint 전달 |
| `/fmu/in/vehicle_command` | Arm / Mode 변경 |
| `/fmu/out/vehicle_local_position` | EKF 추정 위치 수신 |
| `/fmu/out/actuator_servos` | Servo 출력 수신 |

`px4_position_cmd.launch.py`는 Command만 실행하며, SITL에서는 Command와 Servo Bridge를 함께 실행하는 `auto_run_gz.launch.py`를 사용합니다.
