#!/usr/bin/env bash
set -e

WS_DIR="$HOME/Desktop/px4_hummingbird"
PX4_DIR="$HOME/Desktop/PX4-Autopilot"
QGC_DIR="$HOME/Applications"
QGC_APP="$QGC_DIR/QGroundControl-x86_64.AppImage"
LOCAL_AGENT_BIN="$WS_DIR/run_agent.sh"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-13}"

start_qgc()
{
  if pgrep -f "QGroundControl-x86_64.AppImage" >/dev/null; then
    echo "QGC already running."
    return
  fi

  if [ ! -x "$QGC_APP" ]; then
    echo "QGC not found or not executable: $QGC_APP"
    return
  fi

  echo "Starting QGC..."
  (cd "$QGC_DIR" && nohup ./QGroundControl-x86_64.AppImage >/tmp/qgc.log 2>&1 &)
}

start_px4_sitl()
{
  echo "Starting PX4 SITL in a new terminal..."
  gnome-terminal -- bash -lc "cd '$PX4_DIR' && source /opt/ros/jazzy/setup.bash && make px4_sitl gz_hummingbird; exec bash" &
}

start_dds_agent_if_available()
{
  local agent_bin=""
  if command -v MicroXRCEAgent >/dev/null 2>&1; then
    agent_bin="MicroXRCEAgent"
  elif [ -x "$LOCAL_AGENT_BIN" ]; then
    agent_bin="$LOCAL_AGENT_BIN"
  else
    echo "MicroXRCEAgent not found. Skipping external DDS agent."
    return
  fi

  if pgrep -f "MicroXRCEAgent udp4 -p 8888" >/dev/null; then
    echo "MicroXRCEAgent already running."
    return
  fi

  echo "Starting MicroXRCEAgent in a new terminal..."
  gnome-terminal -- bash -lc "$agent_bin udp4 -p 8888; exec bash" &
}

start_qgc
start_dds_agent_if_available
start_px4_sitl

if [ "$#" -eq 0 ]; then
  set -- auto_offboard:=true auto_arm:=true
fi

case " $* " in
  *" model_name:="*) ;;
  *) set -- "$@" model_name:=hummingbird_0 ;;
esac

echo "Starting px4_hummingbird_cmd in a new terminal..."
gnome-terminal -- bash -lc "cd '$WS_DIR' && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch px4_hummingbird_cmd px4_position_cmd.launch.py $*; exec bash" &
