#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export AMENT_PREFIX_PATH="/opt/ros/jazzy:${AMENT_PREFIX_PATH}"
export ROS_DOMAIN_ID=43
export GZ_PARTITION=dual_arm_pipeline_demo
exec python3 scripts/demo_transfer_pipeline.py "$@"
