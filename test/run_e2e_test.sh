#!/bin/bash
# This end-to-end test runs the entire cross-compilation pipeline for ROS2 packages.
# This test does the following:
# 1. Mocks a sysroot directory in a temporary directory.
# 2. Runs the cross compilation script against the mock sysroot, compiling all the ROS2 packages.
# 3. Runs a Docker container using the image built from cross compilation script.
# 4. Executes a ROS2 C++ example demo publisher node in the container.
# 5. Closes the Docker container.
# The test passes if none of the above actions fail to complete.

set -uxo pipefail

# Terminal output colors
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[0;33m'
readonly CYAN='\033[0;36m'
readonly NORM='\033[0m'
# Defaults
arch=aarch64  # or "armhf"
os=ubuntu     # or "debian"
distro=dashing
rmw=fastrtps
result=1        # Default to failure

# Loggers
log(){
  printf "%b%s%b\n" "$CYAN" "$1" "$NORM"
}

warning(){
  printf "%b%s%b\n" "$YELLOW" "$1" "$NORM"
}

error(){
  printf "%b%s%b\n" "$RED" "$1" "$NORM"
}

success(){
  printf "%b%s%b\n" "$GREEN" "$1" "$NORM"
}

# Utilities
cleanup(){
  # Usage: cleanup CONTAINER_NAME RESULT
  if [[ "$2" -eq 0 ]]; then
    success PASS
  else
    error FAIL
  fi
  rm -rf "$temp_path/sysroot";
  docker stop -t 2 "$1"
  docker rm "$1" >/dev/null 2>/dev/null;
}

setup(){
  # Usage: setup CONTAINER_NAME
  temp_path=$(mktemp -d)
  mkdir "$temp_path/sysroot"
  mkdir -p "$temp_path/sysroot/ros2_ws/src/"
  # Get full directory name of the script no matter where it is being called from
  dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
  # Copy dummy pkg
  cp -r "$dir/dummy_pkg" "$temp_path/sysroot/ros2_ws/src"
  # Copy QEMU binaries
  mkdir -p "$temp_path/sysroot/qemu-user-static"
  cp /usr/bin/qemu-* "$temp_path/sysroot/qemu-user-static"

  if [[ $(docker inspect -f "{{.State.Running}}" "$CONTAINER_NAME") == "true" ]]; then
    warning "Container named $CONTAINER_NAME already running. Stopping it before proceeding."
    docker stop -t 2 "$CONTAINER_NAME"
  fi
}

# Argparser
while [[ $# -gt 0 ]]
do
  key="$1"

  case $key in
      -a|--arch)
      arch="$2"
      shift 2
      ;;
      -o|--os)
      os="$2"
      shift 2
      ;;
      -d|--distro)
      distro="$2"
      shift 2
      ;;
      -r|--rmw)
      rmw="$2"
      shift 2
      ;;
      *)
      error "Unrecognized option $1"
      exit 2
      shift
      ;;
  esac
done

# Expected name of the container
readonly CONTAINER_NAME="cc-$arch-$os-$rmw-$distro"
readonly CONTAINER_TAG="$USER/$arch-$os-$rmw-$distro:latest"
# Create trap to make sure all artifacts are removed on exit
trap 'cleanup $CONTAINER_NAME $result' EXIT

# Testing starts here
setup "$CONTAINER_NAME"

# Check if the ros2 command exists
log "Checking for ros2 command in PATH..."
if ! type ros2 > /dev/null; then
  error "Unable to find ros2 in PATH. Make sure you have sourced your environment/workspace properly."
  result=1
  exit 1
fi

# Run the cross compilation script
log "Executing cross compilation script..."
ros2 run cross_compile cross_compile --arch "$arch" --os "$os" --distro "$distro" --rmw "$rmw" \
                                        --sysroot-path "$temp_path"
CC_SCRIPT_STATUS=$?
if [[ "$CC_SCRIPT_STATUS" -ne 0 ]]; then
  error "Failed to run cross compile script."
  result=1
  exit 1
fi

# Check the container was created
log "Running Docker container..."
docker run --name "$CONTAINER_NAME" -td "$CONTAINER_TAG" /bin/bash
IS_CONTAINER_RUNNING=$(docker inspect -f "{{.State.Running}}" "$CONTAINER_NAME")
if [[ "$IS_CONTAINER_RUNNING" != "true" ]]; then
  error "Container was not created."
  result=1
  exit 1
fi

log "Executing node in Docker container..."
docker exec -t "$CONTAINER_NAME" bash -c 'source /ros2_ws/install/local_setup.bash && dummy_binary'
IS_PUBLISHER_RUNNING=$?
if [[ "IS_PUBLISHER_RUNNING" -ne 0 ]]; then
  error "Failed to run the dummy binary in the Docker container."
  result=1
  exit 1
fi
log "Stopping Docker container..."
docker stop -t 2 "$CONTAINER_NAME"

result=0
exit 0
