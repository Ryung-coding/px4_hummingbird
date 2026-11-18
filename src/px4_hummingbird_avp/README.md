# HummingBird AVP

## 준비

`mode:=sim`은 Gazebo RGB-D, `mode:=real`은 OAK-D Pro를 사용합니다. PX4와 DDS Agent는 기존 방식으로 먼저 실행하고, 모든 터미널에서 같은 `ROS_DOMAIN_ID`를 사용합니다. AVP 런치는 한 번만 실행합니다.

```bash
sudo apt install ros-jazzy-depthai-ros-v3 ros-jazzy-sensor-msgs-py python3-numpy python3-opencv python3-yaml
cd ~/Desktop/px4_hummingbird
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select px4_hummingbird_avp
source install/setup.bash
```

OAK 장치 ID·장착 TF·영상 토픽은 `config/oak_d_pro.yaml`, AVP 경계와 게인은 `config/avp_indoor.yaml`에서 설정합니다. 플래너의 `corridor_*`와 실행기의 비행 경계는 함께 맞춥니다. 실행기 `z_levels: [1.0, 1.0, 1.0, 1.0]`는 1 m에서 +Y→-Y→+Y→-Y→+Y로 두 번 왕복하며, 플래너 `sweep_z_levels: [1.0]`는 시점 후보 높이만 지정합니다. OAK USB 포트는 현재 `3-7`이며, 권한 오류 시 udev 규칙과 연결 포트를 확인합니다.

## 실행 조합

아래는 `mode:=real` 기준입니다. 시뮬레이션에서는 `mode:=sim`으로 바꿉니다. 실기체 비전 입력은 기존 OptiTrack 노드가 담당하고, 시뮬레이션은 Gazebo pose를 PX4 비전 입력으로 항상 전달합니다. `enable_planner` 기본값만 `true`이며 나머지 `enable_*`, `auto_*`는 `false`입니다.

`enable_planner`는 맵·시점 계산, `enable_executor`는 코리더·목표 생성, `enable_cmd`는 PX4로 목표 전달을 뜻합니다. `enable_rviz`는 화면만 켭니다. `enable_logger:=true`는 RGB 영상과 프레임별 카메라 TF·원본 깊이·odometry·목표를 `logs/flight_날짜/`에 기록하고, `enable_executor:=true`인 조합에서는 궤적 `logs/avp_*.jsonl`도 기록합니다. `auto_offboard`와 `auto_arm`은 `enable_cmd:=true`일 때만 유효합니다.

**카메라·TF 확인:** 플래너·실행기·명령 노드 없이 영상과 TF만 표시합니다. PX4 odometry가 없으면 RViz `Fixed Frame`을 `base_link`로 바꿉니다.

```bash
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=real enable_planner:=false enable_executor:=false enable_cmd:=false enable_rviz:=true
```

**알고리즘만 확인:** 맵·시점을 만들되 경로나 PX4 명령은 내지 않습니다. 이 조합에서는 `/avp/path_ready` 게이트를 끕니다. `map -> base_link` TF를 위해 PX4 odometry와 DDS Agent는 필요합니다.

```bash
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=real require_path_ready:=false enable_executor:=false enable_cmd:=false enable_rviz:=true
```

**비행 없이 카메라 기록:** 맵 계산·경로·명령 없이 RGB·깊이·프레임별 TF와 수신한 odometry/목표를 저장합니다. 아래는 실카메라 기준이며, SITL은 `mode:=sim`으로 바꿉니다.

```bash
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=real enable_planner:=false enable_executor:=false enable_cmd:=false enable_logger:=true
```

**알고리즘 확인 + 기록:** 맵·시점을 만들면서 영상·깊이를 저장합니다. 실행기가 꺼져 있어 궤적 JSONL은 만들지 않습니다.

```bash
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=real require_path_ready:=false enable_executor:=false enable_cmd:=false enable_rviz:=true enable_logger:=true
```

**AVP 비행:** 맵·시점과 경로를 만들어 PX4로 보냅니다. `require_path_ready`는 기본 `true`로 두고, 아래 명령은 자동 Arm을 하지 않습니다.

```bash
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=real enable_executor:=true enable_cmd:=true enable_rviz:=true auto_offboard:=true
```

**AVP 비행 + 기록:** 위 비행 명령에 `enable_logger:=true`를 추가하면 영상·깊이·카메라 TF·odometry·목표와 궤적 JSONL을 함께 저장합니다. 자동 Arm이 필요한 경우에만 `auto_arm:=true`를 추가합니다. 다른 `px4_position_cmd`를 이미 실행 중이면 `enable_cmd:=true`로 중복 실행하지 않습니다. `.bashrc`의 `avp_log mode:=sim` 또는 `avp_log mode:=real`도 같은 비행 기록 런치이지만 `auto_arm:=true`이므로 기록만 할 때는 사용하지 않습니다.

```bash
ros2 launch px4_hummingbird_avp avp_indoor.launch.py mode:=real enable_executor:=true enable_cmd:=true enable_rviz:=true auto_offboard:=true enable_logger:=true
```

## 비행 없는 점검

위의 **알고리즘만 확인** 런치를 켠 뒤 다른 터미널에서 다음을 실행합니다. 10초간 점군 Hz, `/lics/odom` Hz, `map -> base_link -> optical` TF, 점군의 map 높이와 OAK 축 방향을 검사합니다. `RESULT: READY`가 나와야 합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/px4_hummingbird/install/setup.bash
ros2 run px4_hummingbird_avp lics_check.py --ros-args -p mode:=real
```

시뮬레이션 점검에는 `-p mode:=sim -p use_sim_time:=true`를 대신 사용합니다. 나무 ROI 통계가 필요하면 `-p roi_cx:=0.0 -p roi_cy:=0.0 -p roi_r:=1.2`를 추가합니다. **바닥이 카메라 시야에 있고 map의 z=0이 바닥일 때만** `-p expect_floor:=true`로 높이 판정을 켭니다. OAK 축 검사는 발행된 TF를 검증하며, 실물 장착 방향은 물체를 앞·위·오른쪽에 놓아 RViz 점군 방향까지 확인해야 합니다.

PX4 없이 OAK 점군과 `base_link` TF만 확인할 때는 **카메라·TF만 확인** 런치와 다음 명령을 사용합니다.

```bash
ros2 run px4_hummingbird_avp test_oak_d_pro.py --duration 10
```

## 기록 파일과 확인

`enable_logger:=true`로 만든 `src/px4_hummingbird_avp/logs/flight_*/`에는 다음이 들어갑니다.

- `rgb.mp4`: RGB 영상. `rgb_frames.jsonl`: **영상 프레임 번호(1부터 시작)**, 촬영 `stamp_ns`, 카메라 `frame_id`, `map`(ENU) 기준 카메라 위치·자세 `pose_map_camera=[x,y,z,qw,qx,qy,qz]`. 영상 보간으로 복제된 프레임은 `duplicated`와 `source_video_frame`으로 표시합니다. TF를 얻지 못한 프레임은 `tf_status=missing`이고 pose가 `null`입니다.
- `events.jsonl`: RGB·깊이 프레임 이벤트, RGB/깊이 `CameraInfo`의 내부 파라미터, AVP 시점·경로.
- `depth_raw/*.npy`: 미터 단위 복원이 가능한 원본 깊이. `depth.mp4`는 거리 색상 미리보기입니다.
- `odometry.csv`, `setpoints.csv`: 실제 위치와 명령 목표.

같은 `logs/` 디렉터리의 별도 파일 `avp_*.jsonl`은 실행기를 켰을 때만 생성되며, 기본 스윕 `/avp/base_setpoint`와 AVP 목표 `/avp/setpoint` 궤적을 담습니다.

예를 들어 저장된 영상을 열고 해당 프레임의 TF를 확인합니다. 아래 `flight_날짜`는 실제 폴더명으로 바꿉니다.

```bash
xdg-open ~/Desktop/px4_hummingbird/src/px4_hummingbird_avp/logs/flight_날짜/rgb.mp4
head -n 1 ~/Desktop/px4_hummingbird/src/px4_hummingbird_avp/logs/flight_날짜/rgb_frames.jsonl
```

`avp_logger avp_파일명`은 궤적 JSONL의 기본 경로와 AVP 목표를 3D·위치·자세 그래프로 표시합니다. `.jsonl` 확장자는 생략합니다. `avp_log_headless`는 **기록 명령이 아니라 재생 명령**입니다. `flight_*` 폴더만 있어도 영상·깊이·실제 위치·목표를 재생하며, 같은 시각의 `~/Desktop/view_log/*_headless.jsonl`이 있으면 각속도·모터·서보 등의 신호를 추가합니다. 헤드리스 파일은 이 AVP 런치가 생성하지 않습니다.

```bash
avp_logger avp_기록시각
avp_log_headless flight_기록시각 --check
avp_log_headless flight_기록시각
avp_log_headless flight_기록시각 같은시각_headless
avp_log_headless flight_기록시각 --no-rviz
avp_log_headless flight_기록시각 --no-planner
```

위의 `기록시각`과 `같은시각_headless`는 실제 파일명으로 바꿉니다. `avp_log_headless`에 인자를 주지 않으면 가장 최근 `flight_*` 폴더를 엽니다. 시점별 영상은 Python 창, 기체 TF·경로와 기록된 시점은 RViz에서 봅니다. 시간 슬라이더와 재생 속도는 두 창에 공통으로 적용됩니다. `--no-planner`는 저장된 시점·경로만 보여주고 지도 재계산을 생략합니다. 재생은 기본적으로 별도 `ROS_DOMAIN_ID=113`에서 실행되고 PX4에 명령을 보내지 않습니다.

11/15 기록에는 `/avp/setpoint`가 없어 목표로 `/fmu/in/trajectory_setpoint6dof`를 사용합니다. RGB 프레임의 **촬영 시각 TF 분석**에는 재생 창의 기체 TF를 추정해 쓰지 말고 `rgb_frames.jsonl`의 해당 프레임 pose를 사용합니다. 픽셀에서 3D 위치를 계산할 때는 `events.jsonl`의 RGB `CameraInfo`와 이미지 프레임의 광학축 정의도 함께 확인합니다.
