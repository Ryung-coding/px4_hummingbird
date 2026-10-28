#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${DDS_DOMAIN_ID:-13}"

exec python3 "$ROOT/debug/dds_stability.py" "$@"
