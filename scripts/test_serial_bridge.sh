#!/usr/bin/env bash
set -e
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TASK_BUILD_ROOT="${DUAL_ARM_BUILD_ROOT:-/tmp/dual-arm-ros2}"
source "$TASK_BUILD_ROOT/install/setup.bash"
export ROS_DOMAIN_ID=166 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR="$TASK_BUILD_ROOT/serial_bridge_logs"
exec python3 "$TASK_ROOT/scripts/serial_bridge_integration.py"
