# HummingBird AVP

## 카메라 설치

`mode:=sim`은 Gazebo RGB-D 카메라를 쓰므로 OAK 설치가 필요 없습니다. `mode:=real`은 OAK-D Pro를 시작합니다.

```bash
sudo apt update
sudo apt install ros-jazzy-depthai-ros-v3 ros-jazzy-sensor-msgs-py python3-numpy python3-opencv python3-yaml
cd ~/Desktop/px4_hummingbird
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select px4_hummingbird_avp
source install/setup.bash
```

현재 OAK 물리 포트는 `3-7`입니다. 기존 DepthAI udev 규칙과 충돌하면 해당 포트에만 권한을 주고 카메라를 다시 연결합니다. 포트가 바뀌면 `KERNEL=="3-7"`도 바꿉니다.

```bash
sudo ln -sfn /dev/null /etc/udev/rules.d/60-ros-jazzy-depthai-ros-v3.rules
printf 'SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", KERNEL=="3-7", ATTR{idVendor}=="03e7", OWNER="%s", MODE="0600"\n' "$USER" | sudo tee /etc/udev/rules.d/80-oak-port-3-7.rules >/dev/null
sudo udevadm control --reload-rules
```

OAK 장치 ID, 장착 위치·자세, 영상 토픽은 `config/oak_d_pro.yaml`에서 설정합니다. AVP 플래너·코리더 수치는 `config/avp_indoor.yaml`, Gazebo 토픽 브리지는 `config/bridge.yaml`에서 설정합니다. 플래너의 `corridor_*`와 실행기의 중심·반경·고도 범위는 함께 맞춰야 합니다. 설정을 수정했으면 위 빌드를 다시 실행합니다.

## 실행

PX4, DDS Agent 및 필요한 서보 브리지는 기존 방식으로 먼저 켜 둡니다. 모든 터미널의 ROS 도메인을 일치시키고, AVP 런치는 같은 도메인에서 하나만 실행합니다. `mode:=sim`은 Gazebo RGB-D 센서를, `mode:=real`은 OAK-D Pro를 연결합니다. 아래 직접 실행 명령은 모두 ROS 환경을 준비한 터미널에서 사용합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/px4_hummingbird/install/setup.bash
```

### 런치 인자

`ros2 launch px4_hummingbird_avp avp_indoor.launch.py 인자:=값` 형식입니다. `enable_*`와 `auto_*`는 `true` 또는 `false`를 사용합니다. 표의 기본값은 **직접 실행** 기준이며, `avp`/`avp_log` alias는 일부 값을 미리 `true`로 지정합니다.

| 인자 | 기본값 | `true` 또는 선택한 값의 동작 |
| --- | --- | --- |
| `mode` | `sim` | `sim`: Gazebo 카메라·브리지, 시뮬레이션 시간 사용. `real`: OAK 카메라·점군 노드, 실시간 사용. |
| `enable_planner` | `true` | 점군으로 맵을 만들고 AVP 관측 시점을 생성. `false`이면 계획하지 않음. |
| `enable_executor` | `false` | 코리더 경로와 관측 시점 목표를 `/avp/setpoint`로 발행. `false`이면 비행 경로를 생성하지 않음. |
| `enable_cmd` | `false` | `px4_position_cmd`를 `path=ext`로 실행해 `/avp/setpoint` 목표를 PX4 명령으로 전달. `false`이면 이 런치가 비행 명령을 보내지 않음. |
| `auto_offboard` | `false` | 명령 노드가 자동 Offboard 전환을 시도. `enable_cmd:=true`일 때만 의미 있음. |
| `auto_arm` | `false` | 명령 노드가 자동 Arm을 시도. `enable_cmd:=true`일 때만 의미 있음. |
| `enable_rviz` | `false` | AVP 맵·경로 확인용 RViz 실행. |
| `enable_logger` | `false` | 궤적 JSONL 로거 실행(`logs/avp_*.jsonl`). 카메라 영상은 저장하지 않음. |
| `enable_flight_record` | `false` | RGB 영상, 원본 깊이, 카메라 정보, 수신한 odometry·목표값을 `logs/flight_날짜/`에 저장. |
| `enable_ev` | `false` | `sim`에서 Gazebo odometry를 PX4 외부 비전 입력으로 전달. `real`에서는 사용하지 않음. |

플래너·실행기의 거리, 고도, 속도 및 `require_control`은 런치 인자가 아니라 `config/avp_indoor.yaml`에서 설정합니다. 실기체에서 자동 Arm이 필요 없으면 `auto_arm:=false`인 직접 실행 명령을 사용합니다.

### AVP 비행

현재 `~/.bashrc`의 `avp` alias는 `enable_executor`, `enable_cmd`, `enable_rviz`, `auto_offboard`, `auto_arm`을 `true`로 지정합니다. 플래너는 기본값 `true`, 두 로거는 기본값 `false`입니다. 실기체에서는 자동 Arm/Offboard 동작을 확인한 뒤 사용합니다.

Alias (`~/.bashrc`를 불러온 터미널):

```bash
avp mode:=sim
avp mode:=real
```

동일한 직접 실행:

```bash
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=sim enable_executor:=true enable_cmd:=true enable_rviz:=true auto_offboard:=true auto_arm:=true
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=real enable_executor:=true enable_cmd:=true enable_rviz:=true auto_offboard:=true auto_arm:=true
```

`mode` 생략 시 `sim`입니다. 이미 `gz_cmd path:=avp` 등으로 명령 노드를 실행 중이라면 `avp`와 중복 실행하지 않습니다.

### RViz에서 실시간 보기

`avp mode:=sim`과 `avp mode:=real`은 이미 `enable_rviz:=true`라서 AVP와 함께 RViz가 열립니다. 비행 명령 없이 화면만 확인하려면 다음처럼 실행합니다. `enable_planner`는 기본값 `true`이므로 맵과 관측 시점도 계산하지만, `enable_executor`와 `enable_cmd`는 `false`여서 이 런치가 경로나 비행 명령을 내지 않습니다.

```bash
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=sim enable_rviz:=true
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=real enable_rviz:=true
```

이미 AVP 런치가 실행 중이면 두 번째 AVP 런치를 켜지 말고, 다른 터미널에서 RViz만 엽니다. 시뮬레이션에서는 `use_sim_time`도 켭니다.

```bash
# sim
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix px4_hummingbird_avp)/share/px4_hummingbird_avp/config/avp_indoor.rviz" --ros-args -p use_sim_time:=true
# real
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix px4_hummingbird_avp)/share/px4_hummingbird_avp/config/avp_indoor.rviz"
```

기본 RViz 설정의 `Fixed Frame`은 `map`입니다. 주요 표시 항목은 카메라 점군(`/rgbd_camera/points`), 점유 맵(`/avp/occ_voxels`), 관측 시점(`/avp/viewpoints`), 코리더·경로(`/avp/safety_region`, `/avp/path`), 목표 자세(`/avp/base_setpoint`, `/avp/setpoint`), 기체 위치(`/lics/odom`)입니다. Planner가 꺼져 있으면 맵·시점이, Executor가 꺼져 있으면 경로·목표값이 나타나지 않습니다. PX4 odometry가 없으면 `map -> base_link` TF가 없으므로 카메라만 확인할 때는 RViz의 `Fixed Frame`을 `base_link`로 바꿉니다.

기본 RViz 설정에는 RGB 영상 패널이 없습니다. 영상도 보려면 RViz에서 `Add` -> `Image`를 선택하고 토픽을 sim은 `/rgbd_camera/image`, real은 `/oak/rgb/image_raw`로 지정합니다. 카메라 영상만 필요하면 Planner와 비행 노드를 끄고 실행할 수 있습니다.

```bash
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=sim enable_planner:=false enable_rviz:=true
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=real enable_planner:=false enable_rviz:=true
```

### 기록만

AVP 알고리즘과 비행 명령을 실행하지 않고 RGB 영상, 원본 깊이, 카메라 보정값, 수신한 오도메트리·목표값을 저장합니다. 이미 다른 노드가 경로를 보내고 있다면 그 목표값도 기록됩니다.

기록 전용 alias는 현재 없으므로 직접 실행합니다:

```bash
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=sim enable_planner:=false enable_flight_record:=true
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=real enable_planner:=false enable_flight_record:=true
```

위 명령은 둘 중 하나만 실행합니다. `enable_executor`와 `enable_cmd`는 기본값 `false`이므로 새 경로나 Arm 명령을 내지 않습니다. PX4가 연결되지 않았다면 해당 odometry·목표값 행은 저장되지 않습니다.

### AVP 비행과 기록

`avp_log` alias는 `avp`와 같은 비행 노드에 `enable_logger:=true`를 더합니다. 카메라·PX4 데이터까지 기록하려면 `enable_flight_record:=true`를 추가합니다.

Alias:

```bash
avp_log mode:=sim enable_flight_record:=true
avp_log mode:=real enable_flight_record:=true
```

동일한 직접 실행:

```bash
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=sim enable_executor:=true enable_cmd:=true enable_rviz:=true auto_offboard:=true auto_arm:=true enable_logger:=true enable_flight_record:=true
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=real enable_executor:=true enable_cmd:=true enable_rviz:=true auto_offboard:=true auto_arm:=true enable_logger:=true enable_flight_record:=true
```

비행 없이 플래너의 맵·시점 생성과 카메라만 기록하려면 실행기·명령 노드가 기본적으로 꺼진 직접 런치를 사용합니다:

```bash
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=sim enable_flight_record:=true
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=real enable_flight_record:=true
```

궤적 JSONL만 필요하면 `avp_log mode:=sim` 또는 `avp_log mode:=real`을 사용합니다. `enable_flight_record` 없이는 카메라 영상이 저장되지 않습니다. 시뮬레이션에서 외부 비전 입력이 필요할 때만 `enable_ev:=true`를 명시합니다.

## 테스트

### 카메라·브리지

실카메라만 확인할 때는 PX4 없이 다음 런치를 켜고, 다른 터미널에서 점군과 TF를 확인합니다. 비행 명령 노드는 켜지지 않습니다.

```bash
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=real enable_planner:=false
ros2 run px4_hummingbird_avp test_oak_d_pro.py --duration 10
```

두 명령은 각각 다른 터미널에서 실행합니다. 정상이라면 유효한 XYZ 점군과 `TF(base_link <- cloud)=True`가 출력됩니다. 시뮬 카메라 확인은 SITL/Gazebo를 켠 뒤 아래처럼 합니다.

```bash
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=sim enable_planner:=false
ros2 topic hz /rgbd_camera/points
```

모드 전환 전 기존 AVP 런치를 `Ctrl+C`로 종료합니다.

### 기록 파일

위의 "기록만" 또는 "AVP 비행과 기록" 명령으로 수 초간 기록한 뒤 해당 터미널에서 `Ctrl+C`를 눌러 MP4를 닫습니다. 파일은 `~/Desktop/px4_hummingbird/src/px4_hummingbird_avp/logs/flight_날짜/`에 저장됩니다.

- `rgb.mp4`: RGB 영상.
- `depth.mp4`: 보기 위한 깊이 색상 영상. 거리 복원용이 아닙니다.
- `depth_raw/*.npy`: 원본 16UC1(mm) 또는 32FC1(m) 깊이 배열.
- `events.jsonl`: 각 프레임의 시각·인코딩·깊이 단위·원본 파일 경로와 카메라 보정값, AVP 경로·시점.
- `setpoints.csv`: AVP 목표(map/ENU)와 실제 PX4 6DoF 명령(local_ned), 속도·가속도·각속도.
- `odometry.csv`: 수신한 `/lics/odom`.

```bash
run=$(find ~/Desktop/px4_hummingbird/src/px4_hummingbird_avp/logs -maxdepth 1 -type d -name 'flight_*' | sort | tail -1)
ls "$run"/rgb.mp4 "$run"/depth.mp4 "$run"/depth_raw/000001.npy
grep -m 1 'camera_info' "$run"/events.jsonl
head -n 3 "$run"/setpoints.csv
python3 -c 'import numpy as np, sys; a=np.load(sys.argv[1]); print(a.shape, a.dtype, np.nanmin(a), np.nanmax(a))' "$run"/depth_raw/000001.npy
```


`setpoints.csv`의 `topic`과 `frame_id`를 보고 좌표계를 구분합니다. 명령 노드가 없으면 PX4 6DoF 명령 행은 생기지 않습니다. `avp_log`의 궤적 기록은 별도의 `logs/avp_*.jsonl`이며, 기존 `avp_logger <파일명>`은 이 궤적 JSONL을 표시합니다. `flight_날짜` 폴더용 재생 명령은 아닙니다.
