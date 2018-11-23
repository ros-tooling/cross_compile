# Copyright (c) 2018, ARM Limited.
# SPDX-License-Identifier: Apache-2.0

#!/bin/bash -e

# 3. Prepare the sysroot
if [ ! -d "$CC_WS/sysroot_docker" ]; then
    # Build sysroot image
    docker build -t aarch64_ros2:latest -f ./ros2_ws/src/ros2/cross_compile/sysroot/Dockerfile_ubuntu_arm64 .
    docker run --name aarch64_sysroot aarch64_ros2:latest

    # Export sysroot image
    docker container export -o sysroot_docker.tar aarch64_sysroot
    mkdir sysroot_docker

    tar -C sysroot_docker -xf sysroot_docker.tar lib usr
fi

# 4. Build
# Export toolchain-file variable
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu

export CC=/usr/bin/$TARGET_TRIPLE-gcc
export CXX=/usr/bin/$TARGET_TRIPLE-g++
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

export SYSROOT=$CC_WS/sysroot_docker
export PYTHON_SOABI=cpython-36m-$TARGET_TRIPLE
export ROS2_INSTALL_PATH=$CC_WS/ros2_ws/install


# Hack to build class-loader
## This is temporarily required because this libraires are not found on the
## specified SYSROOT
ln -s `pwd`/sysroot_docker/lib/$TARGET_TRIPLE/libz.so.1 /usr/lib/$TARGET_TRIPLE/libz.so
ln -s `pwd`/sysroot_docker/lib/$TARGET_TRIPLE/libpcre.so.3 /usr/lib/$TARGET_TRIPLE/libpcre.so

# Ignore some package
touch \
    ros2_ws/src/ros2/rviz/COLCON_IGNORE \
    ros2_ws/src/ros2/demos/intra_process_demo/COLCON_IGNORE \
    ros2_ws/src/ros2/demos/image_tools/COLCON_IGNORE \
    ros2_ws/src/ros2/robot_state_publisher/COLCON_IGNORE

cd ros2_ws

# Trigger a build
colcon build --merge-install \
    --cmake-force-configure \
    --cmake-args \
        -DCMAKE_VERBOSE_MAKEFILE=ON \
        -DCMAKE_TOOLCHAIN_FILE="$(pwd)/src/ros2/cross_compile/cmake-toolchains/generic_linux.cmake" \
        -DSECURITY=ON
