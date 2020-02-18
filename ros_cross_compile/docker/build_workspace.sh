#!/bin/bash
# No -u because setup.bash for ROS has unset variables
# No -x because it prints all of setup.bash steps, which are numerous and noisy
set -eo pipefail

usage() {
  echo "Usage: build_workspace.sh ROS_DISTRO TARGET_ARCH OWNER_USER"
  echo "ROS_DISTRO: ROS (1 or 2) distribution to build against."
  echo "TARGET_ARCH: architecture we are building for, used for directory namespacing."
  echo "OWNER_USER: ID of the calling user, used to chown after building. Since this is run in a container that may not have the username, provide the ID number."
}

if [ $# != 3 ]; then
  usage
  exit 1
fi

ROS_DISTRO=$1
TARGET_ARCH=$2
OWNER_USER=$3

# Touch the setup scripts, in case ROS is not actually installed, so that we can source
mkdir -p /opt/ros/"${ROS_DISTRO}"
touch /opt/ros/"${ROS_DISTRO}"/setup.bash

source /opt/ros/"${ROS_DISTRO}"/setup.bash
colcon build --mixin "${TARGET_ARCH}"-docker \
  --build-base build_"${TARGET_ARCH}" \
  --install-base install_"${TARGET_ARCH}"
chown -R "${OWNER_USER}" .
