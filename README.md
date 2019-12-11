# ROS2 Cross Compile

![License](https://img.shields.io/github/license/ros-tooling/cross_compile)
[![Documentation Status](https://readthedocs.org/projects/cross_compile/badge/?version=latest)](https://cross_compile.readthedocs.io/en/latest/?badge=latest)

A tool for easily compiling ROS2 packages to non-native architectures.

:construction: Note: while this tool is named `cross_compile`, it currently relies on running emulated builds using QEmu, [this ticket](https://github.com/ros-tooling/cross_compile/issues/69) tracks our progress toward enabling cross-compilation.

## Installation

### Prerequisites

#### Python
Following the [REP-2000 Dashing Diademata](https://www.ros.org/reps/rep-2000.html#dashing-diademata-may-2019-may-2021) guideline, Python 3.5 and above are supported.

#### Ubuntu
Qemu and Docker have to be installed for cross_compile to work.
The following instructions have been tested on Ubuntu Xenial (16.04) and Bionic (18.04).

```bash
# Install requirements for Docker and qemu-user-static
sudo apt update && sudo apt install -y \
    curl \
    qemu-user-static

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

* Install all software dependencies required for ROS 2 development by following the [ROS 2 documentation](https://index.ros.org/doc/ros2/Installation/Latest-Development-Setup/)
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

We need to setup a `sysroot` directory for the docker artifacts. Docker can only copy from a specific
context so all these assets need to be copied relative to the `Dockerfile` path.

#### 1. Create the directory structure
```bash
mkdir -p sysroot/qemu-user-static
mkdir -p sysroot/ros2_ws/src
```

#### 2. Copy the QEMU Binaries

```bash
cp /usr/bin/qemu-*-static sysroot/qemu-user-static/
```

#### 3. Prepare ros2_ws
```bash
# Copy in your ros2_ws
cp -r <full_path_to_your_ros_ws>/src sysroot/ros2_ws/src

# Use vcs to checkout the required ROS2 version.
# Substitute `master` for `release-latest` or a specific release like `dashing`.
curl -fsSL https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos | vcs import src/
```

#### 4. Run the cross compilation script

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

### Including Non-rosdep Dependencies

Your ROS application may have dependencies that cannot be resolved by `rosdep` because they only exist in nonstandard APT repositories.
To add additional sources and install arbitrary packages from them, you may use the `--extra-dependencies` commandline option.
This option takes in a relative or absolute path to a YAML specification file.
The format of that file is as follows.

```
# 'apt_sources' [optional] specifies nonstandard apt sources
apt_sources:
    # 'manual_repos' [optional] is a list of non-PPA apt repositories to add to a custom sources.list, and fetch a key for it
    manual_repos:
        # all keys 'url', 'keyserver', and 'recv_key' are required
        - url: <Repository URL, e.g. https://packages.customproject.com/ubuntu/something>
          keyserver: <Keyserver URL e.g. hkp://ha.pool.sks-keyservers.net:80>
          recv_key: <Key ID, hex string>
    # 'ppas' is a list of PPAs to add via `add-apt-repository`
    ppas:
        - <PPA, e.g. ppa:ubuntu-pi-flavour-makers/ppa>
# 'apt_packages' [optional] is a list of packages to install via apt after the above sources have been added
apt_packages:
    - <apt package name, e.g. libraspberrypi0>
```

Here is an example input file


```
# my-extra-deps.yaml
apt_sources:
  manual_repos:
    - url: https://packages.ubiquityrobotics.com/ubuntu/ubiquity
      keyserver: hkp://ha.pool.sks-keyservers.net:80
      recv_key: C3032ED8
  ppas:
    - ppa:ubuntu-pi-flavour-makers/ppa
apt_packages:
  - libraspberrypi0
```

And the corresponding invocation

```
ros2 run cross_compile cross_compile \
    --sysroot-path /path/to/my/workspace \
    --arch aarch64 \
    --os ubuntu \
    --extra-dependencies my-extra-deps.yaml
```

## Build Status

| ROS 2 Release | Branch Name     | Development | Source Debian Package | X86-64 Debian Package | ARM64 Debian Package | ARMHF Debian package |
| ------------- | --------------- | ----------- | --------------------- | --------------------- | -------------------- | -------------------- |
| Latest        | `master`        | [![Test Pipeline Status](https://github.com/ros-tooling/cross_compile/workflows/Test%20cross_compile/badge.svg)](https://github.com/ros-tooling/cross_compile/actions) | N/A                   | N/A                   | N/A                  | N/A                  |
| Dashing       | `dashing-devel` | [![Build Status](http://build.ros2.org/buildStatus/icon?job=Ddev__cross_compile__ubuntu_bionic_amd64)](http://build.ros2.org/job/Ddev__cross_compile__ubuntu_bionic_amd64) | [![Build Status](http://build.ros2.org/buildStatus/icon?job=Dsrc_uB__cross_compile__ubuntu_bionic__source)](http://build.ros2.org/job/Dsrc_uB__cross_compile__ubuntu_bionic__source) | [![Build Status](http://build.ros2.org/buildStatus/icon?job=Dbin_uB64__cross_compile__ubuntu_bionic_amd64__binary)](http://build.ros2.org/job/Dbin_uB64__cross_compile__ubuntu_bionic_amd64__binary) | N/A | N/A |
