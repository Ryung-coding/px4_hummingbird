#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${DDS_DOMAIN_ID:-13}"

LOG_DIR="$ROOT/debug/logs"
STAMP="$(date -u +%Y%m%d_%H%M%S)"
AGENT_DEVICE="${DDS_SERIAL_DEVICE:-/dev/dds}"
AGENT_BAUD="${DDS_AGENT_BAUD:-921600}"
START_AGENT=0
START_SERVO=0
PYTHON_ARGS=()
STARTED_AGENT_PIDS=""
STARTED_SERVO_PIDS=""

for arg in "$@"; do
  case "$arg" in
    --start-agent) START_AGENT=1 ;;
    --start-servo) START_SERVO=1 ;;
    *) PYTHON_ARGS+=("$arg") ;;
  esac
done

mkdir -p "$LOG_DIR"

agent_pids() {
  pgrep -f '[M]icroXRCEAgent' || true
}

servo_pids() {
  pgrep -f '[p]x4_servo_to_dynamixel' || true
}

stop_started_processes() {
  trap - INT TERM
  if [[ -n "$STARTED_SERVO_PIDS" ]]; then
    while read -r pid; do
      [[ -n "$pid" ]] && kill -INT "$pid" 2>/dev/null || true
    done <<< "$STARTED_SERVO_PIDS"
  fi
  if [[ -n "$STARTED_AGENT_PIDS" ]]; then
    while read -r pid; do
      [[ -n "$pid" ]] && sudo kill -INT "$pid" 2>/dev/null || true
    done <<< "$STARTED_AGENT_PIDS"
  fi
}
trap stop_started_processes INT TERM

AGENT_PIDS="$(agent_pids)"
if [[ -n "$AGENT_PIDS" ]]; then
  printf 'MicroXRCEAgent already running: %s\n' "$(tr '\n' ' ' <<< "$AGENT_PIDS")"
elif (( START_AGENT )); then
  [[ -e "$AGENT_DEVICE" ]] || { printf 'DDS serial device not found: %s\n' "$AGENT_DEVICE" >&2; exit 1; }
  sudo -v
  AGENT_LOG="$LOG_DIR/agent_$STAMP.log"
  sudo env ROS_DOMAIN_ID="$ROS_DOMAIN_ID" MicroXRCEAgent serial --dev "$AGENT_DEVICE" -b "$AGENT_BAUD" >"$AGENT_LOG" 2>&1 &
  sleep 1
  STARTED_AGENT_PIDS="$(agent_pids)"
  if [[ -z "$STARTED_AGENT_PIDS" ]]; then
    printf 'MicroXRCEAgent failed to start. Log: %s\n' "$AGENT_LOG" >&2
    tail -n 60 "$AGENT_LOG" >&2 || true
    exit 1
  fi
  printf 'Started MicroXRCEAgent pid=%s log=%s\n' "$(tr '\n' ' ' <<< "$STARTED_AGENT_PIDS")" "$AGENT_LOG"
else
  printf 'MicroXRCEAgent is not running. Use --start-agent to start and capture it.\n'
fi

SERVO_PIDS="$(servo_pids)"
if [[ -n "$SERVO_PIDS" ]]; then
  printf 'px4_servo_to_dynamixel already running: %s\n' "$(tr '\n' ' ' <<< "$SERVO_PIDS")"
elif (( START_SERVO )); then
  SERVO_LOG="$LOG_DIR/servo_$STAMP.log"
  stdbuf -oL -eL ros2 run px4_hummingbird_cmd px4_servo_to_dynamixel >"$SERVO_LOG" 2>&1 &
  sleep 1
  STARTED_SERVO_PIDS="$(servo_pids)"
  if [[ -z "$STARTED_SERVO_PIDS" ]]; then
    printf 'px4_servo_to_dynamixel failed to start. Log: %s\n' "$SERVO_LOG" >&2
    tail -n 60 "$SERVO_LOG" >&2 || true
    exit 1
  fi
  printf 'Started px4_servo_to_dynamixel pid=%s log=%s\n' "$(tr '\n' ' ' <<< "$STARTED_SERVO_PIDS")" "$SERVO_LOG"
else
  printf 'px4_servo_to_dynamixel is not running. Use --start-servo to start and capture it.\n'
fi

python3 "$ROOT/debug/dds_stability.py" "${PYTHON_ARGS[@]}"
