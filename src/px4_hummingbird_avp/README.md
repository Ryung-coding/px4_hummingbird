# HummingBird AVP: OAK-D Pro

## 설치

ROS 2 Jazzy와 이 작업공간을 준비한 뒤 카메라용 패키지를 설치합니다.

```bash
sudo apt update
sudo apt install ros-jazzy-depthai-ros-v3 ros-jazzy-sensor-msgs-py python3-numpy python3-yaml
cd /home/ryung/Desktop/px4_hummingbird
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select px4_hummingbird_avp
source install/setup.bash
```

## USB 권한

현재 카메라의 물리 포트는 `3-7`입니다. 이 포트만 현재 사용자에게 허용하려면 다음을 실행하고 카메라를 다시 연결합니다. 포트가 바뀌면 규칙의 `KERNEL=="3-7"`도 바꿉니다.

```bash
sudo ln -sfn /dev/null /etc/udev/rules.d/60-ros-jazzy-depthai-ros-v3.rules
printf 'SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", KERNEL=="3-7", ATTR{idVendor}=="03e7", OWNER="%s", MODE="0600"\n' "$USER" | sudo tee /etc/udev/rules.d/80-oak-port-3-7.rules >/dev/null
sudo udevadm control --reload-rules
```

장치 MXID, 장착 위치·자세, 깊이 토픽은 `config/oak_d_pro.yaml`에서 설정합니다. `physical_usb_port`는 기록용이며 장치 선택에는 MXID를 사용합니다.

## 실행

각 터미널에서 `source /opt/ros/jazzy/setup.bash`와 `source install/setup.bash`를 먼저 실행합니다.

```bash
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=sim
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=real
```

실기체 AVP 실행에는 PX4 uXRCE-DDS Agent와 odometry 연결이 필요합니다.

## 런치 인자

`ros2 launch px4_hummingbird_avp avp_indoor.launch.py 인자:=값` 형식으로 지정합니다. `enable_*`와 `require_control`은 `true` 또는 `false`를 사용합니다.

| 인자 | 기본값 | 용도 |
| --- | --- | --- |
| `mode` | `sim` | `sim` 또는 `real` |
| `bridge_config` | `config/bridge.yaml` | 시뮬레이션 Gazebo 브리지 설정 |
| `oak_config_file` | `config/oak_d_pro.yaml` | 실카메라 설정 |
| `rviz_config` | `config/avp_indoor.rviz` | RViz 화면 설정 |
| `enable_ev` | `false` | 시뮬레이션 외부 비전 입력 |
| `enable_planner` | `true` | AVP 플래너 실행 |
| `enable_executor` | `false` | 코리더 실행기 실행 |
| `enable_cmd` | `false` | PX4 명령 노드 실행 |
| `cmd_path` | `ext` | PX4 명령 노드의 경로 모드 |
| `enable_logger` | `false` | 궤적 로거 실행 |
| `log_dir` | `~/Desktop/px4_hummingbird/src/px4_hummingbird_avp/logs` | 궤적 로그 저장 위치 |
| `enable_rviz` | `false` | RViz 실행 |
| `auto_offboard` | `false` | 명령 노드의 자동 Offboard 전환 |
| `auto_arm` | `false` | 명령 노드의 자동 Arm |
| `require_control` | `true` | 실행기의 제어권 확인 |
| `setpoint_topic` | `/avp/setpoint` | 실행기 목표값 출력 토픽 |

## 카메라 테스트

PX4 없이 카메라, 점군, TF, RViz를 확인합니다.

```bash
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=real enable_planner:=false enable_rviz:=true rviz_config:=$(ros2 pkg prefix px4_hummingbird_avp)/share/px4_hummingbird_avp/config/oak_d_pro.rviz
```

다른 터미널에서 실행합니다.

```bash
ros2 run px4_hummingbird_avp test_oak_d_pro.py --duration 10
```

정상이라면 유효한 XYZ 점군과 `TF(base_link <- cloud)=True`가 출력됩니다.
