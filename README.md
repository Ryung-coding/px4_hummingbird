# HummingBird ROS 2 Workspace

PX4 HummingBird와 ROS 2 Jazzy를 연결하는 Workspace입니다. DDS를 통해 EKF Position을 수신하고, Position/Attitude Command를 전달하며, Servo Angle을 Gazebo로 출력합니다.

PX4 SITL은 [HB_v1.17](https://github.com/Ryung-coding/px4_hummingbird/tree/HB_v1.17) 브랜치를 사용합니다.

## 1. Build

`ros2-ws` 브랜치를 별도 디렉터리에 다운로드한 후 Workspace 루트에서 실행합니다.

```bash
git clone -b ros2-ws https://github.com/Ryung-coding/px4_hummingbird.git
cd px4_hummingbird
sudo apt install ros-jazzy-gz-msgs-vendor ros-jazzy-gz-transport-vendor
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. 실행

PX4 프로젝트 루트에서 SITL을 실행하고 QGroundControl (QGC)을 켭니다.

```bash
make px4_sitl gz_hummingbird
```

ROS 2 Workspace에서 별도 터미널로 DDS Agent와 Launch를 실행합니다.

```bash
# Terminal 1: DDS Agent (UDP 8888)
./run_agent.sh

# Terminal 2: ROS 2 Command + Gazebo Servo Bridge
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch px4_hummingbird_cmd auto_run_gz.launch.py path:=pos
```

`path` 옵션:

| Path | 동작 |
|---|---|
| `pos` | Position Trajectory |
| `att` | Attitude Trajectory |
| `step_att` | Attitude Step Response |

예를 들어 `path:=att` 또는 `path:=step_att`로 변경할 수 있습니다. Command만 실행할 때는 `px4_position_cmd.launch.py`를 사용합니다.

## 3. 주요 기능 및 DDS Topic

| 방향 | Topic | 역할 |
|---|---|---|
| PX4 → ROS 2 | `/fmu/out/vehicle_local_position` | EKF Position 수신 |
| ROS 2 → PX4 | `/fmu/in/trajectory_setpoint6dof` | Position/Attitude Command 전송 |
| ROS 2 → PX4 | `/fmu/in/offboard_control_mode` | Offboard Mode 설정 |
| ROS 2 → PX4 | `/fmu/in/vehicle_command` | Arm / Mode 전환 |
| PX4 → ROS 2 | `/fmu/out/actuator_servos` | Servo 출력 수신 |

`px4_servo_to_gz`는 PX4의 Servo 출력을 Gazebo Tilt Joint에 전달합니다. Motor 출력은 PX4 Gazebo Bridge를 사용합니다.