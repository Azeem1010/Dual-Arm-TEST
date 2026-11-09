#!/usr/bin/env bash
set -e
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# rosidl's CMake file(STRINGS) handling breaks on non-ASCII build paths.
# Keep generated artifacts in an ASCII path; source files stay in the workspace.
TASK_BUILD_ROOT="${DUAL_ARM_BUILD_ROOT:-/tmp/dual-arm-ros2}"
# Stage the read-only vision source: setuptools and Python may generate files
# beside sources, including when the installed package uses symlinks.
python3 - "$TASK_ROOT" "$TASK_BUILD_ROOT" <<'PYTHON'
from pathlib import Path
import shutil
import sys
root, build = map(Path, sys.argv[1:])
source = root / 'three-camera-vision-ros2-main/vision_pkg'
staged = build / 'vision_source/vision_pkg'
if staged.exists():
    shutil.rmtree(staged)
staged.parent.mkdir(parents=True, exist_ok=True)
shutil.copytree(source, staged, ignore=shutil.ignore_patterns('__pycache__', '*.egg-info', '.pytest_cache'))
PYTHON
source /opt/ros/jazzy/setup.bash
colcon --log-base "$TASK_BUILD_ROOT/log" build \
  --base-paths "$TASK_ROOT/src" "$TASK_BUILD_ROOT/vision_source/vision_pkg" \
  --build-base "$TASK_BUILD_ROOT/build" \
  --install-base "$TASK_BUILD_ROOT/install" --symlink-install --executor sequential
echo "Built. Source: $TASK_BUILD_ROOT/install/setup.bash"
