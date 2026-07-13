#!/usr/bin/env bash
set -e

AGENT_ROOT="$HOME/Desktop/px4_hummingbird/Micro-XRCE-DDS-Agent"
AGENT_BUILD="$AGENT_ROOT/build"
AGENT_BIN="$AGENT_BUILD/MicroXRCEAgent"

AGENT_LIB_PATH="$AGENT_BUILD"
AGENT_LIB_PATH="$AGENT_LIB_PATH:$AGENT_BUILD/temp_install/fastdds-3.1/lib"
AGENT_LIB_PATH="$AGENT_LIB_PATH:$AGENT_BUILD/temp_install/fastcdr-2.2.4/lib"
AGENT_LIB_PATH="$AGENT_LIB_PATH:$AGENT_BUILD/temp_install/microxrcedds_client-3.0.0/lib"
AGENT_LIB_PATH="$AGENT_LIB_PATH:$AGENT_BUILD/temp_install/microcdr-2.0.1/lib"

export LD_LIBRARY_PATH="$AGENT_LIB_PATH${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

if [ "$#" -eq 0 ]; then
  set -- udp4 -p 8888
fi

exec "$AGENT_BIN" "$@"
