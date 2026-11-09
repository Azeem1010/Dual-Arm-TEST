#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# This workspace does not override ROS-distribution packages. Resolve those
# from Linux storage first to avoid thousands of negative stats over /mnt/*.
export AMENT_PREFIX_PATH="/opt/ros/jazzy:${AMENT_PREFIX_PATH}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
export GZ_PARTITION="${GZ_PARTITION:-dual_arm_pipeline_demo}"
case "${1:-gui}" in
  gui) exec ros2 launch dual_arm_bringup sim.launch.py gz_args:=-r ;;
  headless) exec ros2 launch dual_arm_bringup sim.launch.py 'gz_args:=-r -s' ;;
  mock) exec ros2 launch dual_arm_bringup mock.launch.py rviz:=false ;;
  *) echo 'Usage: bash scripts/start_sim.sh gui|headless|mock' >&2; exit 2 ;;
esac
