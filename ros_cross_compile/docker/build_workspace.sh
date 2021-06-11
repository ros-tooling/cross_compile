#!/bin/bash
set -euxo pipefail

cleanup() {
  chown -R "${OWNER_USER}" .
}

trap 'cleanup' EXIT

mkdir -p /opt/ros/"${ROS_DISTRO}"
touch /opt/ros/"${ROS_DISTRO}"/setup.bash

set +ux
# shellcheck source=/dev/null
source /opt/ros/"${ROS_DISTRO}"/setup.bash
if [ -f /custom-data/setup.bash ]; then
    # shellcheck source=/dev/null
    source /custom-data/setup.bash
fi
set -ux
colcon build --mixin "${TARGET_ARCH}"-docker \
  --build-base build_"${TARGET_ARCH}" \
  --install-base install_"${TARGET_ARCH}"

# Runs user-provided post-build logic (file is present and empty if it wasn't specified)
/user-custom-post-build
