# Colcon cc-build

Colcon plugin for cross-compilation

## Install prerequisites

### Ubuntu

The cross compilation toolchain and docker have to be installed. 
The following instructions have been tested on Ubuntu Xenial (16.04) and Bionic (18.04).

```bash
# Install cross compilation toolchain
sudo apt-get update
sudo apt-get install -y build-essential cmake git wget curl lsb-core bash-completion \
    qemu-user-static g++-aarch64-linux-gnu g++-arm-linux-gnueabihf python3-pip htop
sudo python3 -m pip install -U  colcon-common-extensions rosdep vcstool

# Also install docker and make it available to the current user: https://docs.docker.com/install/linux/docker-ce/ubuntu/
sudo apt-get install -y apt-transport-https ca-certificates curl gnupg-agent software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo apt-key fingerprint 0EBFCD88
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt-get update
sudo apt-get install -y docker-ce
sudo usermod -aG docker $USER
newgrp docker # this reloads the group permissions in the current shell, unnecessary after relogin
docker run hello-world

# Install dependencies required by the cross-compile tool
pip3 install -r sysroot_creator/requirements.txt
```

### Mac
The following instructions have been tested on Mac OS Mojave (10.14).

```bash
# Ensure your brew install is healthy
brew doctor
brew install cmake git wget curl bash-completion qemu
python3 -m pip install --user -U  colcon-common-extensions rosdep vcstool
```

[Install Docker toolbox](https://docs.docker.com/toolbox/toolbox_install_mac/)

## Usage

We need to setup our sysroot directory for the docker image. Docker can only copy from a specific
context so all these assets need to be copied relative to the `Dockerfile` path.
```bash
cd /path/to/sysroot
# Create a directory to store ROS/qemu assets
mkdir qemu-user-static ros2_ws
cp /usr/bin/qemu-* qemu-user-static 
# Copy ROS Sources
cp -r ~/ros2_ws/src ros2_ws
```

For the current setup,
```bash
# Create a directory to store qemu assets
mkdir -p sysroot_creator/scripts/sysroot/qemu-user-static
cp -r /usr/bin/qemu-* sysroot_creator/scripts/sysroot/qemu-user-static/

# Get ROS sources (if there is no existing source checkout)
# release-latest (dashing)
wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
# crystal
wget https://raw.githubusercontent.com/ros2/ros2/crystal/ros2.repos

# Copy ROS Sources
mkdir -p sysroot_creator/scripts/sysroot/ros2_ws/src

# If there is no existing source checkout
vcs import sysroot_creator/scripts/sysroot/ros2_ws/src < ros2.repos 

# From an existing workspace
cp -r ~/ros2_ws sysroot_creator/scripts/sysroot/ros2_ws
```

In the end your `sysroot` directory should look like this:
```bash
sysroot
 |-- Dockerfile_workspace
 +-- qemu-user-static
 |   +-- qemu-*-static
 +-- ros2_ws
     +-- src
          |-- (ros2 packages) 
          +-- ...
```

### Building a workspace

1. Setup the sysroot

Add --force-sysroot-build to force rebuilding the sysroot
```bash
python3 create_cc_sysroot.py --arch [armhf|aarch64] --os [ubuntu|debian]
```
2. Install the colcon mixins for cross-compilation
```bash
colcon mixin add cc_mixins file://<path_to_cross_compile_repo>/mixins/index.yaml
colcon mixin update cc_mixins 
# Check the mixins are installed by running
colcon mixin show
```
3. Launch cross compilation using the sysroot created and colcon mixin for target architecture
```bash
colcon build --mixin [armhf-generic_linux|aarch64-generic_linux]
  --packages-up-to examples_rclcpp_minimal_publisher
```

#### Sample Docker images

You can use the [Official Dockerhub ROS Repo](https://hub.docker.com/_/ros) to find base images.

You can also use [OSRF's Dockerhub Repo](https://hub.docker.com/r/osrf/ros2) to obtain images as well.

#### Assumptions

- The Docker image for `--sysroot-base-image` installs the ROS 2 distro at `/opt/ros/${distro}`.

### Troubleshooting

#### Debug

To manually build and/or run the workspace image

```bash
docker image build -f colcon_cc_build/colcon_cc_build/verb/sysroot/Dockerfile_workspace \
  --network host \
  -t ros2_benchmark_pipeline:latest \
  --build-arg ROS2_BASE_IMG=<your-base-image> \
  --build-arg ROS2_WORKSPACE=. --build-arg ROS_DISTRO=crystal --build-arg TARGET_TRIPLE=aarch64-linux-gnu \
  .

docker container run -it --rm --network=host --name test ros2_benchmark_pipeline:latest bash
```


#### Lib Poco Issue
From the ROS2 Cross compilation docs:
> The Poco pre-built has a known issue where it is searching for libz and libpcre on the host system instead of SYSROOT. 
> As a workaround for the moment, please link both libraries into the the host’s file-system.
> ```bash
> mkdir -p /usr/lib/$TARGET_TRIPLE
> ln -s `pwd`/sysroot_docker/lib/$TARGET_TRIPLE/libz.so.1 /usr/lib/$TARGET_TRIPLE/libz.so
> ln -s `pwd`/sysroot_docker/lib/$TARGET_TRIPLE/libpcre.so.3 /usr/lib/$TARGET_TRIPLE/libpcre.so
> ```
