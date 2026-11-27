#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=40

if [ -f "docker/docker-compose.yaml" ]; then
  docker compose -f docker/docker-compose.yaml up -d || echo "[WARN] Docker GPU start skipped/failed"
fi

source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/cobot2_ws/install/setup.bash" ]; then
  source "$HOME/cobot2_ws/install/setup.bash"
fi
if [ -f "$WS_DIR/install/setup.bash" ]; then
  source "$WS_DIR/install/setup.bash"
fi

ros2 launch store_bringup store_bot.launch.py
