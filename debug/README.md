# Orin DDS and Servo Diagnostics

이 모니터는 Opti 원본부터 PX4 visual-odometry 입력, PX4 상태·서보 출력까지 수신 간격과 원본 timestamp를 같은 시간축으로 기록한다. Agent와 서보 노드 PID 변화도 0.2초 간격으로 감시한다.

## Run

Agent와 서보 노드가 이미 켜져 있다면 중복 실행하지 않고 관찰한다.

```bash
cd ~/Desktop/px4_hummingbird
bash debug/run_dds_stability.sh --expect-opti --expect-servos --duration 0
```

서보 노드가 꺼져 있고 이 스크립트에서 로그를 잡아 실행하려면 `--start-servo`를 추가한다. Agent도 꺼져 있을 때만 `--start-agent`를 추가한다. 이미 실행 중인 Agent나 서보 노드는 감지 후 그대로 둔다.

```bash
bash debug/run_dds_stability.sh --start-servo --expect-opti --expect-servos --duration 0
bash debug/run_dds_stability.sh --start-agent --start-servo --expect-opti --expect-servos --duration 0
```

Agent 기본 연결은 `/dev/dds`, 921600 baud다. 다르게 연결했다면 `DDS_SERIAL_DEVICE`와 `DDS_AGENT_BAUD` 환경 변수를 지정한다. `--duration`의 기본값은 600초이며 0은 Ctrl-C까지 기록한다. 스크립트가 시작한 프로세스는 Ctrl-C 때만 SIGINT로 종료한다. 측정 시간이 끝나면 시작한 프로세스는 계속 실행된다.

## Recorded Data

- JSONL 수신 로그: `debug/logs/dds_<UTC>.jsonl`
- 스크립트가 직접 시작한 Agent 출력: `debug/logs/agent_<UTC>.log`
- 스크립트가 직접 시작한 서보 노드 출력: `debug/logs/servo_<UTC>.log`

이미 다른 터미널에서 시작한 프로세스의 이전 stdout/stderr는 가져올 수 없다. 이 경우 PID 종료는 즉시 기록되지만 구체적 오류는 해당 실행 터미널 또는 ROS 로그에서 확인한다. 다음 실험에서 오류 전문을 보려면 `--start-agent`, `--start-servo`로 스크립트가 직접 띄운다.

기본 `--gap 0.05`는 50ms 이상 구간을 기록하는 모니터 기준이다. 서보 노드의 0도 복귀 기준인 0.5초와는 별개다. 각 GAP 이벤트는 ROS 구독 수신 간격과 메시지 원본 timestamp 간격을 함께 보여준다.

## Locate the Gap

- `/nokov_raw/Hummingbird/pose`도 멈추면 Opti 원본 입력 단계부터 확인한다.
- Nokov 입력은 이어지는데 `/opti_raw`가 멈추면 mocap 변환 노드 단계를 확인한다.
- `/opti_raw`는 이어지는데 `/fmu/in/vehicle_visual_odometry`가 멈추면 ROS→PX4 DDS 입력을 확인한다.
- FMU 입력은 이어지는데 PX4 odometry·attitude·servo 출력 timestamp가 멈추면 PX4 추정·제어 발행 단계를 확인한다.
- PX4 source timestamp 간격은 정상인데 Orin 수신 간격만 커지면 Agent·serial·ROS 수신 경로를 확인한다. Agent PID가 살아 있다는 사실만으로 DDS 데이터가 계속 흐른다는 뜻은 아니다.

Sync Write 성공은 직렬 패킷 전송 성공을 뜻하며, 개별 서보의 실제 위치 응답을 확인하지 않는다. 이 드라이버는 시작 때 present position을 읽어 초기 LPF를 만들고 운용 중 위치를 반복해서 읽지 않는다.
