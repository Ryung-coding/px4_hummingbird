# Orin DDS stability check

Agent가 **이미 실행 중인 상태**에서 PX4 DDS 출력이 실제로 끊기는지 연속 측정한다. 이 진단은 Agent를 중단하거나 재시작하지 않고, Opti/cmd/viewer/서보 노드를 새로 실행하지 않으며, PX4에 아무 명령도 발행하지 않는다.

## Run

Orin의 `a603` 작업본에서 한 명령만 실행한다. 스크립트가 ROS Jazzy와 이 작업본의 `install/setup.bash`를 source하고 `ROS_DOMAIN_ID=13`을 설정한다.

```bash
cd ~/Desktop/px4_hummingbird
bash debug/run_dds_stability.sh
```

기본 측정 시간은 10분이다. 더 길게 보려면 `--duration 1800`, Ctrl-C까지 계속 보려면 `--duration 0`을 붙인다. 기본 공백 기준은 서보 노드의 DDS 수신 타임아웃과 같은 0.2초이며 `--gap 0.1`처럼 조정할 수 있다.

무장 해제 상태에서는 `/fmu/out/vehicle_attitude`만으로 DDS 연속성을 확인할 수 있다. `/fmu/out/actuator_servos`와 `/fmu/out/actuator_motors`는 PX4가 실제로 발행할 때 함께 측정된다. 무장 해제 중 서보 0 Hz는 고장이 아니다. 서보 명령이 반드시 나와야 하는 별도 지상 시험에서만 `--expect-servos`를 사용한다. 모터·서보를 동작시키는 시험은 프로펠러를 제거하고 기체를 고정한 상태에서 한다.

## Read Results

- 5초마다 출력되는 `att`, `servo`의 Hz, 마지막 메시지 나이(`age`), 최근 2000개 간격의 `p99`, 누적 공백 수(`gaps`), 최대 공백(`max`)을 본다.
- `SILENCE`는 마지막 수신 후 0.2초 이상 새 메시지가 없다는 뜻이다. 다시 메시지가 오면 전체 중단 길이를 `GAP`으로 기록한다.
- `NO DATA /fmu/out/vehicle_attitude`는 시작 후 3초 동안 기준 출력조차 들어오지 않았다는 뜻이다.
- `agent=DOWN`은 Orin에서 Agent 실행 파일을 찾지 못한 상태다. `servo_node=off`는 서보 실행 파일이 없는 상태이며, 서보 토픽의 수신 자체와는 별개의 정보다.
- 종료 시 토픽별 전체 샘플 수, 공백 횟수, 최대 공백, p99를 출력한다. 모든 메시지의 수신 시각, PX4 timestamp, 서보 control, 상태와 공백 이벤트는 `debug/logs/dds_<UTC>.jsonl`에 저장된다. 이 로그 폴더는 Git에서 제외한다.

이 측정은 **Orin의 ROS 구독자가 받은 시각**을 기준으로 한다. 공백만으로 PX4 내부 발행 중단과 Agent/serial 손실을 단정할 수 없다. 공백이 발생하면 PX4 콘솔의 `listener vehicle_attitude`, `listener actuator_servos`, `uxrce_dds_client status`와 비교한다. PX4 listener는 계속 갱신되는데 Orin만 공백이라면 DDS/Agent/serial/Orin 구독 경로를 우선 조사한다. Agent의 `Payload rx: 0 B/s`는 Opti와 cmd 입력이 없다면 정상이다.
