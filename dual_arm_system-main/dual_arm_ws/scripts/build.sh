#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --parallel-workers 2 --event-handlers console_direct+ 2>&1 | tee docs/evidence/colcon_build.log
source install/setup.bash
python3 scripts/validate_model.py 2>&1 | tee docs/evidence/model_validation.log
# Install generated adjacency collision exclusions.
colcon build --base-paths src --packages-select dual_arm_moveit_config --event-handlers console_direct+ 2>&1 | tee docs/evidence/moveit_rebuild.log
