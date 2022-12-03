#!/bin/bash
# This end-to-end test runs the entire cross-compilation pipeline for this tool
# This test does the following:
# 1. Mocks a sysroot directory in a temporary directory.
# 2. Runs the cross compilation script against the mock sysroot, compiling all the packages.
# 3. Runs a Docker container using the image built from cross compilation script.
# 4. Executes a C++ example demo node in the container.
# 5. Closes the Docker container.
# The test passes if none of the above actions fail to complete.

set -euxo pipefail

# Terminal output colors
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[0;33m'
readonly CYAN='\033[0;36m'
readonly NORM='\033[0m'
# Defaults
arch=aarch64  # or "armhf"
os=ubuntu     # or "debian"
distro=foxy
result=1        # Default to failure

RUNTIME_IMAGE_TAG="$(whoami)/$arch-$os-$distro:e2e-runtime"
readonly RUNTIME_IMAGE_TAG

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

panic() {
  error "$1"
  result=1
  exit 1
}

success(){
  printf "%b%s%b\n" "$GREEN" "$1" "$NORM"
}

# Utilities
cleanup(){
  # Usage: cleanup RESULT
  if [[ "$1" -eq 0 ]]; then
    success PASS
  else
    error FAIL
  fi
  rm -rf "$test_sysroot_dir";
}

setup(){
  if [[ "$distro" =~ ^(melodic|noetic)$ ]]; then
    ros_version=ros
  else
    ros_version=ros2
  fi

  test_sysroot_dir=$(mktemp -d)
  mkdir -p "$test_sysroot_dir/src"
  # Get full directory name of the script no matter where it is being called from
  dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

  # Copy correct dummy test pkg for the current argument set
  if [ "$ros_version" == "ros2" ] && [ "$os" == "ubuntu" ]; then
    # ROS2 + Debian info
    # Debian is a Tier 3 package for current ROS 2 distributions, so it doesn't have apt releases
    # Therefore we can't resolve the rclcpp dependency of the ros2 package, so we build the empty project
    cp -r "$dir/dummy_pkg_ros2_cpp" "$test_sysroot_dir/src"
    cp -r "$dir/dummy_pkg_ros2_py" "$test_sysroot_dir/src"
    target_package="dummy_pkg_ros2_cpp"
  fi

  cp -r "$dir/dummy_pkg" "$test_sysroot_dir/src"
  target_package="dummy_pkg"

  custom_setup_script=${test_sysroot_dir}/custom-setup.bash
  echo "#!/bin/bash" > "$custom_setup_script"
  if [ "$arch" == "armhf" ]; then
    if [ "$distro" == "foxy" ] || \
       [ "$distro" == "galactic" ] || \
       [ "$distro" == "humble" ] || \
       [ "$distro" == "rolling" ]; then
      error "Foxy, Galactic, Humble and Rolling do not have armhf binaries available"
      exit 0
    fi
    if [ "$os" == "debian" ]; then
      if [ "$distro" == "noetic" ]; then
        error "Noetic does not have armhf binaries available to debian"
        exit 0
      fi
    fi
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
      -d|--rosdistro)
      distro="$2"
      shift 2
      ;;
      *)
      panic "Unrecognized option $1"
      ;;
  esac
done

# Create trap to make sure all artifacts are removed on exit
trap 'cleanup $result' EXIT

# Testing starts here
setup

# Run the cross compilation script
log "Executing cross compilation script..."
python3 -m ros_cross_compile "$test_sysroot_dir" \
  --arch "$arch" --os "$os" --rosdistro "$distro" \
  --runtime-tag "$RUNTIME_IMAGE_TAG" \
  --custom-setup-script "$custom_setup_script" --custom-metric-file "${arch}_${os}_${distro}_a"
CC_SCRIPT_STATUS=$?
if [[ "$CC_SCRIPT_STATUS" -ne 0 ]]; then
  panic "Failed to run cross compile script."
fi

install_dir=$test_sysroot_dir/install_$arch
build_dir=$test_sysroot_dir/build_$arch

log "Checking that install directory was output to the correct place..."
if [ ! -d "$install_dir" ]; then
  panic "The install directory was not where it should have been output"
fi

log "Checking that extraction didn't overwrite our workspace"
if [ ! -d "$test_sysroot_dir/src" ]; then
  panic "The user's source tree got deleted by the build"
fi

log "Checking that the binary output is in the correct architecture..."
if [ "$arch" = 'armhf' ]; then
  expected_binary_bits='ELF 32-bit'
elif [ "$arch" = 'aarch64' ]; then
  expected_binary_bits='ELF 64-bit'
elif [ "$arch" = 'x86_64' ]; then
  expected_binary_bits='ELF 64-bit'
fi

if [ "$arch" = 'armhf' ]; then
  expected_binary_architecture=', ARM'
elif [ "$arch" = 'aarch64' ]; then
  expected_binary_architecture=', ARM aarch64'
elif [ "$arch" = 'x86_64' ]; then
  expected_binary_architecture=', x86-64'
fi
binary_file_info=$(file "$install_dir/$target_package/bin/dummy_binary")
if [[ "$binary_file_info" != *"$expected_binary_bits"*"$expected_binary_architecture"* ]]; then
  panic "The binary output was not of the expected architecture"
fi

log "Running example node in the runtime container..."
docker run --rm \
  "$RUNTIME_IMAGE_TAG" \
  -c "source /root/.bashrc && dummy_binary"
RUN_RESULT=$?
if [[ "$RUN_RESULT" -ne 0 ]]; then
  panic "Failed to run the dummy binary in the Docker container."
fi

log "Rerunning build with package selection..."
rm -rf "$install_dir"
rm -rf "$build_dir"
cp -r "$dir/dummy_pkg_ros2_cpp" "$test_sysroot_dir/src"
cat > "$test_sysroot_dir/defaults.yaml" <<EOF
list:
  packages-select: [dummy_pkg]
build:
  packages-select: [dummy_pkg]
  event-handlers: ["console_cohesion+","console_package_list+"]
EOF

python3 -m ros_cross_compile "$test_sysroot_dir" \
  --arch "$arch" --os "$os" --rosdistro "$distro" \
  --colcon-defaults "$test_sysroot_dir/defaults.yaml" \
  --custom-metric-file "${arch}_${os}_${distro}_b"
CC_SCRIPT_STATUS=$?
if [[ "$CC_SCRIPT_STATUS" -ne 0 ]]; then
  panic "Failed to run cross compile script."
fi

if [ ! -d "$install_dir/dummy_pkg" ]; then
  panic "Didn't build the cpp package when selected"
fi
if [ -d "$install_dir/dummy_pkg_ros2_cpp" ]; then
  panic "Built the python package when deselected"
fi

if [ -z "$(ls -A "$test_sysroot_dir"/cc_internals/metrics/)" ]; then
  panic "Failed to write time series data to file"
fi

if [ -z "${METRICS_OUT_DIR-}" ]; then
  :
else
  \cp -rf "$test_sysroot_dir/cc_internals/metrics"/* "$METRICS_OUT_DIR"
fi

result=0
exit 0
