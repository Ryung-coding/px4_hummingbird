# 1. 전체 구조

전체 흐름은 다음과 같다.

```text
ROS 2 px4_position_cmd
        |
        |  position command / offboard mode / arm command 전달
        v
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
        |
        |  DDS 통신
        v
MicroXRCEAgent
        |
        |  ROS 2 DDS <-> PX4 uORB bridge
        v
PX4 uxrce_dds_client
        |
        |  PX4 내부 uORB topic으로 변환
        v
PX4 control stack
        |
        |  위치제어 / 자세제어 / rate 제어 / allocation 수행
        v
actuator_motors / actuator_servos
        |
        |-------------------------------|
        |                               |
        v                               v
PX4 gz_bridge                 ROS 2 px4_servo_to_gz
        |                               |
        |  motor speed 전달             |  servo angle 전달
        v                               v
/model/hummingbird_0/command/   /model/hummingbird_0/servo_0~7
motor_speed
        |                               |
        v                               v
Gazebo motor joint            Gazebo tilt servo joint
```


## 2-1. ROS 2 -> PX4

`px4_position_cmd`가 PX4로 명령을 보낸다.

```text
/fmu/in/offboard_control_mode
```

- Offboard mode에서 어떤 setpoint를 사용할지 알려줌
- 현재는 position command 사용

```text
/fmu/in/trajectory_setpoint
```

- PX4로 보낼 위치 명령
- x, y, z 위치 setpoint 전달

```text
/fmu/in/vehicle_command
```

- PX4 mode 변경 명령
- Offboard 전환, arm 명령 등을 전달


## 2-2. PX4 -> ROS 2

PX4가 계산한 액추에이터 출력을 ROS 2에서 받는다.

```text
/fmu/out/actuator_servos
```

- PX4에서 계산된 서보 출력
- beta, alpha 틸트 서보 명령으로 사용


## 2-3. PX4 -> Gazebo motor

모터 명령은 PX4 기본 Gazebo bridge를 사용한다.

```text
PX4 actuator_motors
        -> PX4 gz_bridge
        -> /model/hummingbird_0/command/motor_speed
```

## 2-4. PX4 -> ROS 2 -> Gazebo servo

서보 명령은 이 워크스페이스의 `px4_servo_to_gz`가 전달한다.

```text
PX4 actuator_servos
        -> DDS
        -> ROS 2 px4_servo_to_gz
        -> /model/hummingbird_0/joint_beta1~4, joint_alpha1~4
```


# 3. Gazebo 모델 이름

Gazebo에서 HummingBird 모델 이름은 다음과 같다.

```text
hummingbird_0
```

`px4_position_cmd.launch.py`는 더 이상 `model_name`을 받지 않는다.
서보 bridge와 viewer/logging은 position launch와 분리해서 필요할 때 별도로 실행한다.


# 4. 액추에이터 번호별 beta / alpha 매핑

PX4 `actuator_servos`는 다음 순서로 사용한다.

```text
control[0] = beta_1
control[1] = beta_2
control[2] = alpha_1
control[3] = alpha_2
control[4] = alpha_3
control[5] = alpha_4
```

Gazebo joint는 beta 조인트가 4개라서 ROS bridge에서 앞/뒤 쌍으로 복제한다.

```text
actuator_servos[0] -> /model/hummingbird_0/joint_beta1
actuator_servos[0] -> /model/hummingbird_0/joint_beta4
actuator_servos[1] -> /model/hummingbird_0/joint_beta2
actuator_servos[1] -> /model/hummingbird_0/joint_beta3

actuator_servos[2] -> /model/hummingbird_0/joint_alpha1
actuator_servos[3] -> /model/hummingbird_0/joint_alpha2
actuator_servos[4] -> /model/hummingbird_0/joint_alpha3
actuator_servos[5] -> /model/hummingbird_0/joint_alpha4
```


# 5. 좌표계 규칙

HummingBird helper에서는 altitude-up 명령 convention을 사용한다.

```text
cmd.z > 0  = 위로 이동
```

하지만 PX4 `TrajectorySetpoint.position`은 local NED 좌표계를 사용한다.

```text
PX4 NED에서 z < 0 = 위로 이동
```

따라서 `px4_position_cmd`에서는 z 부호를 바꿔서 publish한다.

```text
position = [cmd.x, cmd.y, -cmd.z]
```

그림으로 보면 다음과 같다.

```text
Helper command frame

              +z
              ^
              |   up
              |
              o------> +x
             /
            /
          +y

cmd.z > 0 means up
```


```text
PX4 local NED frame

              -z
              ^
              |   up
              |
              o------> +x
             /
            /
          +y

PX4에서 위로 가려면 position.z는 음수
```


```text
변환 규칙

helper command:
    cmd = [x, y, z_up]

PX4 trajectory_setpoint:
    position = [x, y, -z_up]
```


# 6. 빌드 방법

```bash
alias QGC='cd ~/Applications && ./QGroundControl-x86_64.AppImage'
alias agent='~/Desktop/px4_hummingbird/run_agent.sh'
alias px4='cd ~/Desktop/PX4-Autopilot && make px4_sitl gz_hummingbird'
alias px4_ros='ros2 launch px4_hummingbird_cmd px4_position_cmd.launch.py'
```

각 alias의 역할:

```text
QGC      : QGroundControl 실행
agent    : Micro XRCE DDS Agent 실행, 기본 포트 udp4 8888
px4      : PX4 SITL + Gazebo HummingBird 실행
px4_ros  : ROS 2 position command node launch 실행
```

# 7. path 이름


`px4_position_cmd`에서 사용할 수 있는 path 이름:

```text
pos
att
step_att
```

현재 alias 기준 실행 예시:

```bash
px4_ros path:=pos
px4_ros path:=att
px4_ros path:=step_att
```

launch 명령을 직접 쓸 경우:

```bash
ros2 launch px4_hummingbird_cmd px4_position_cmd.launch.py path:=step_att
```
