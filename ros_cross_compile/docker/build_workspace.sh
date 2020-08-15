#!/bin/bash
set -euxo pipefail

cleanup() {
  chown -R "${OWNER_USER}" .
}

trap 'cleanup' EXIT

export SYSROOT=/ros_ws/cc_internals/sysroot
export ROS_WS_INSTALL_PATH=/ros_ws/install_${TARGET_ARCH}
export ROS_WS_BUILD_PATH=/ros_ws/build_${TARGET_ARCH}

rosdir=${SYSROOT}/opt/ros/${ROS_DISTRO}

# It's possible that the workspace does not require ROS binary dependencies
# so this could not have been created. Instead of checking, lazily touch it
mkdir -p ${rosdir}
touch ${rosdir}/setup.bash

export TRIPLE=aarch64-linux-gnu
rsync -a ${SYSROOT}/usr/lib/${TRIPLE}/ /usr/lib/${TRIPLE}/
rsync -a ${SYSROOT}/usr/include/ /usr/include/

set +ux
# shellcheck source=/dev/null
source ${rosdir}/setup.bash
set -ux
# export MAKEFLAGS="-j1"
colcon build \
  --build-base ${ROS_WS_BUILD_PATH} \
  --install-base ${ROS_WS_INSTALL_PATH} \
  --cmake-args -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON -DCMAKE_TOOLCHAIN_FILE=/toolchains/${TARGET_ARCH}-gnu.cmake --no-warn-unused-cli
# Runs user-provided post-build logic (file is present and empty if it wasn't specified)
/user-custom-post-build
