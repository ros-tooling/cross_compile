#!/bin/bash

# This script is intended to be used by the Jenkins night builds.
# But it still possible to use manually setting up this variables:
#
# TARGET=[raspbian, ...]
# ROS2_DISTRO=[ardent,bouncy,crystal,master, ...]

# Any subsequent command which fail will cause the script to exit
set -e

# Check that all variables are defined before start
if [[ -z "$TARGET" || -z "$ROS2_DISTRO" ]]
then
  echo "Error: environment variables are not defined!"
  echo "Set values to: TARGET / ROS2_DISTRO"
  exit 1
fi

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

# Ensure that the cross-compilation environment is built and up-to-date
BUILD_SCRIPT=$THIS_DIR/build.sh
bash $BUILD_SCRIPT

# Prepare cross-compilation environment
SOURCE_SCRIPT=$THIS_DIR/env.sh
source $SOURCE_SCRIPT $TARGET

# Get sysroot
SYSROOT_SCRIPT=$THIS_DIR/get_sysroot.sh
bash $SYSROOT_SCRIPT

# Remove ROS2 old cross-compilation workspace and get a new one
CC_WS_DIR=~/ros2_cc_ws
sudo rm -rf $CC_WS_DIR
mkdir -p $CC_WS_DIR/src
wget -O $CC_WS_DIR/ros2.repos https://raw.githubusercontent.com/ros2/ros2/"$ROS2_DISTRO"/ros2.repos
vcs import $CC_WS_DIR/src < $CC_WS_DIR/ros2.repos
echo "Created CC_WS_DIR=$CC_WS_DIR"

# Get short SHA of HEAD for the corresponding branch
HEAD=$(git ls-remote git://github.com/ros2/ros2 "$ROS2_DISTRO" | cut -c1-7)

# Create install directory for the cross-compilation results
RESULTS_DIR=~/ros2_install/"$ROS2_DISTRO"_"$HEAD"_"$TARGET"
mkdir -p $RESULTS_DIR
echo "Created RESULTS_DIR=$RESULTS_DIR"

# Save the current packages versions
ROS2_EXACT_REPOS_FILE=$RESULTS_DIR/ros2_exact.repos
vcs export --exact $CC_WS_DIR/src > $ROS2_EXACT_REPOS_FILE

# Cross-compiling ROS2
IGNORE_SCRIPT=$THIS_DIR/ignore_pkgs.sh
bash $IGNORE_SCRIPT $CC_WS_DIR $ROS2_DISTRO

# Run the cross-compilation and check the return code
CC_SCRIPT=$THIS_DIR/cc_workspace.sh
CC_CMD="bash $CC_SCRIPT $CC_WS_DIR"
if $CC_CMD; then
  # If the build was succesful, copy results to specified directory
  cp -r $CC_WS_DIR/install $RESULTS_DIR
else
  exit 1
fi
