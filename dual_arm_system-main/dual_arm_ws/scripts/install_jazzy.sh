#!/usr/bin/env bash
set -euo pipefail
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 ]] || { echo 'Requires Ubuntu 24.04'; exit 1; }
[[ $(id -u) == 0 ]] || { echo 'Run as root'; exit 1; }
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y curl ca-certificates gnupg software-properties-common
add-apt-repository -y universe
curl --fail --location https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list
apt-get update
apt-get install -y ros-jazzy-desktop ros-jazzy-moveit ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers ros-jazzy-ros2controlcli ros-jazzy-xacro ros-jazzy-usb-cam ros-jazzy-ament-cmake-gtest libyaml-cpp-dev python3-colcon-common-extensions python3-rosdep python3-opencv python3-pytest python3-yaml python3-numpy python3-scipy python3-venv build-essential cmake
