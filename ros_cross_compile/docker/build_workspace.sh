#!/bin/bash
set -eo pipefail

[ "${ROS_DISTRO}" ] || (echo "ROSDISTRO var unset" && exit 1)
[ "${TARGET_ARCH}" ] || (echo "TARGET_ARCH var unset" && exit 1)
[ "${OWNER_USER}" ] || (echo "OWNER_USER var unset" && exit 1)

source /opt/ros/"${ROS_DISTRO}"/setup.bash
colcon build --mixin "${TARGET_ARCH}"-docker \
  --build-base build_"${TARGET_ARCH}" \
  --install-base install_"${TARGET_ARCH}"
chown -R "${OWNER_USER}" .
