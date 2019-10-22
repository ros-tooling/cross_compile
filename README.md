# ROS2 Cross Compile

![License](https://img.shields.io/github/license/ros-tooling/cross_compile)
[![Documentation Status](https://readthedocs.org/projects/cross_compile/badge/?version=latest)](https://cross_compile.readthedocs.io/en/latest/?badge=latest)

A tool for cross compiling ROS2 packages.

## Installation

### Prerequisites

#### Ubuntu

The cross compilation toolchain and docker have to be installed.
The following instructions have been tested on Ubuntu Xenial (16.04) and Bionic (18.04).

```bash
# Install cross compilation toolchain
sudo apt-get update
sudo apt-get install -y build-essential cmake git wget curl lsb-core bash-completion \
    qemu-user-static g++-aarch64-linux-gnu g++-arm-linux-gnueabihf python3-pip htop
sudo python3 -m pip install -U  colcon-common-extensions rosdep vcstool

# Also install docker and make it available to the current user
sudo apt-get install -y apt-transport-https ca-certificates curl gnupg-agent software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo apt-key fingerprint 0EBFCD88
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt-get update
sudo apt-get install -y docker-ce
sudo usermod -aG docker $USER
newgrp docker # this reloads the group permissions in the current shell, unnecessary after relogin
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
curl https://raw.githubusercontent.com/ros2/ros2/dashing/ros2.repos | vcs import src/
curl https://raw.githubusercontent.com/ros-tooling/cross_compile/master/cross_compile.repos | vcs import src/

# Install all required system dependencies
# Some packages may fail to install, this is expected on an unstable branch,
# and is generally OK.
rosdep update
rosdep install -r --rosdistro=eloquent --ignore-packages-from-source --from-paths src/

# Use colcon to compile cross_compile code and all its dependencies
colcon build --packages-up-to cross_compile

# If you use bash or zsh, source .bash or .zsh, instead of .sh
source install/local_setup.sh
```

## Usage

We need to setup a `sysroot` directory for the docker artifacts. Docker can only copy from a specific
context so all these assets need to be copied relative to the `Dockerfile` path.

#### 1. Copy the QEMU Binaries

```bash
mkdir sysroot
cd sysroot
# Create a directory to store qemu assets
mkdir qemu-user-static
cp /usr/bin/qemu-* qemu-user-static
```

#### 2. Copy the ROS2 packages

If you want to cross compile specific local packages:

```
# Create a directory to store source packages
mkdir -p ros2_ws/src
cp -r <full_path_to_your_ros_ws>/src ros2_ws/src
```

If you want to cross compile the latest ROS2 release instead of your workspace:

```bash
mkdir -p ros2_ws/src
# Get ROS2 sources
wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
vcs import ros2_ws/src < ros2.repos
```

#### 3. Run the cross compilation script

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
ros2 run cross_compile cross_compile --arch aarch64 --os ubuntu --sysroot-path /directory/with/sysroot
```

#### Sample Docker images

You can use the [Official Dockerhub ROS Repo](https://hub.docker.com/_/ros) to find base images.

You can also use [OSRF's Dockerhub Repo](https://hub.docker.com/r/osrf/ros2) to obtain images as well.

## License
This library is licensed under the Apache 2.0 License.

## Troubleshooting

#### Lib Poco Issue
From the ROS2 Cross compilation docs:
> The Poco pre-built has a known issue where it is searching for libz and libpcre on the host system instead of SYSROOT.
> As a workaround for the moment, please link both libraries into the the host’s file-system.
> ```bash
> mkdir -p /usr/lib/$TARGET_TRIPLE
> ln -s `pwd`/sysroot_docker/lib/$TARGET_TRIPLE/libz.so.1 /usr/lib/$TARGET_TRIPLE/libz.so
> ln -s `pwd`/sysroot_docker/lib/$TARGET_TRIPLE/libpcre.so.3 /usr/lib/$TARGET_TRIPLE/libpcre.so
> ```
