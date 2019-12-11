# ROS2 Cross Compile

![License](https://img.shields.io/github/license/ros-tooling/cross_compile)
[![Documentation Status](https://readthedocs.org/projects/cross_compile/badge/?version=latest)](https://cross_compile.readthedocs.io/en/latest/?badge=latest)

A tool to automate ROS2 packages compilation to non-native architectures.

:construction: `cross_compile` relies on running emulated builds
using QEmu, #69 tracks progress toward enabling cross-compilation.

## Installation

### Prerequisites

This tool requires:

- Docker
- Python 3.5, or newer.
- QEmu

#### Installing system dependencies on Ubuntu

On Ubuntu Bionic (18.04), run the following commands to install system
dependencies:

```bash
# Install requirements for Docker and qemu-user-static
sudo apt update && sudo apt install -y curl qemu-user-static

# Full instructions on installing Docker may be found here:
# https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-docker-engine---community
curl -fsSL https://get.docker.com | sudo sh -

# Allow the current user to invoke Docker CLI.
sudo usermod -aG docker $USER

# Reload the group permissions in the current shell. Otherwise logout and login again to apply permissions
newgrp docker

# Verify current user can run Docker
docker run hello-world
```

### Installing from source

#### Latest (unstable development - `master` branch)

Please follow those instructions if you plan to contribute to this repository.

* Install all software dependencies required for ROS 2 development by following
  the [ROS 2 documentation][ros2_dev_setup]
* Checkout the source code and compile it as follows

```bash
mkdir -p ~/ros2_cross_compile_ws/src
cd ros2_cross_compile_ws

# Use vcs to clone all required repositories
curl -fsSL https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos | vcs import src/
curl -fsSL https://raw.githubusercontent.com/ros-tooling/cross_compile/master/cross_compile.repos | vcs import src/

# Install all required system dependencies
# Some packages may fail to install, this is expected on an unstable branch,
# and is generally OK.
sudo rosdep init
rosdep update
rosdep install -r -y --rosdistro=eloquent --ignore-packages-from-source --from-paths src/

# Use colcon to compile cross_compile code and all its dependencies
# ros2run is required to run the cross_compile script, but may be installed elsewhere on the host.
colcon build --packages-up-to cross_compile ros2run

# If you use bash or zsh, source .bash or .zsh, instead of .sh
source install/local_setup.sh
```

## Usage

This script requires a `sysroot` directory containing the ROS 2 workspace, and
the toolchain.

The following instructions explain how to create a `sysroot` directory.

#### Create the directory structure
```bash
mkdir -p sysroot/qemu-user-static
mkdir -p sysroot/ros2_ws/src
```

#### Copy the QEMU Binaries

```bash
cp /usr/bin/qemu-*-static sysroot/qemu-user-static/
```

#### Prepare ros2_ws

```bash
# Copy in your ros2_ws
cp -r <full_path_to_your_ros_ws>/src sysroot/ros2_ws/src

# Use vcs to checkout the required ROS2 version.
# Substitute `master` for `release-latest` or a specific release like `dashing`.
curl -fsSL https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos | vcs import src/
```

#### Run the cross compilation script

In the end your `sysroot` directory should look like this:

```bash
sysroot/
 +-- qemu-user-static/
 |   +-- qemu-*-static
 +-- ros2_ws/
     +-- src/
          |-- (ros2 packages)
          +-- ...
```

Then run the tool:

```bash
ros2 run cross_compile cross_compile --sysroot-path /absolute/path/to/sysroot \
                                     --arch aarch64 \
                                     --os ubuntu
```

### Custom Setup Script

Your ROS application may have build needs that aren't covered by `rosdep install`.
If this is the case (for example you need to add extra apt repos), we provide the option `--custom-setup-script` to execute arbitrary code in the sysroot container.

The path provided may be absolute, or relative to the current directory.

Keep in mind
* It is up to the user to determine whether the script is compatible with chosen base platform
* Make sure to specify non-interactive versions of commands, e.g. `apt-get install -y`, or the script may hang waiting for input
* You cannot make any assumptions about the state of the apt cache, so run `apt-get update` before installing packages
* You will be running as root user in the container, so you don't need `sudo`

Below is an example script for an application that installs some custom Raspberry Pi libraries.

```
apt-get update
apt-get install -y software-properties-common

# Install Raspberry Pi library that we have not provided a rosdep rule for
add-apt-repository ppa:rpi-distro/ppa
apt-get install -y pigpio
```

## License

This library is licensed under the Apache 2.0 License.

## Build status

| ROS 2 Release | Branch Name     | Development | Source Debian Package | X86-64 Debian Package | ARM64 Debian Package | ARMHF Debian package |
| ------------- | --------------- | ----------- | --------------------- | --------------------- | -------------------- | -------------------- |
| Latest        | `master`        | [![Test Pipeline Status](https://github.com/ros-tooling/cross_compile/workflows/Test%20cross_compile/badge.svg)](https://github.com/ros-tooling/cross_compile/actions) | N/A                   | N/A                   | N/A                  | N/A                  |
| Dashing       | `dashing-devel` | [![Build Status](http://build.ros2.org/buildStatus/icon?job=Ddev__cross_compile__ubuntu_bionic_amd64)](http://build.ros2.org/job/Ddev__cross_compile__ubuntu_bionic_amd64) | [![Build Status](http://build.ros2.org/buildStatus/icon?job=Dsrc_uB__cross_compile__ubuntu_bionic__source)](http://build.ros2.org/job/Dsrc_uB__cross_compile__ubuntu_bionic__source) | [![Build Status](http://build.ros2.org/buildStatus/icon?job=Dbin_uB64__cross_compile__ubuntu_bionic_amd64__binary)](http://build.ros2.org/job/Dbin_uB64__cross_compile__ubuntu_bionic_amd64__binary) | N/A | N/A |


[ros2_dev_setup]: https://index.ros.org/doc/ros2/Installation/Latest-Development-Setup/
