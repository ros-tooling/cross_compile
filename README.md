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
mkdir -p ~/ros_cross_compile_ws/src
cd ros_cross_compile_ws

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
mkdir -p sysroot/ros_ws/src
```

#### Copy the QEMU Binaries

```bash
cp /usr/bin/qemu-*-static sysroot/qemu-user-static/
```


#### 3. Prepare ros_ws
Use [ROS](http://wiki.ros.org/ROS/Installation) or [ROS 2](https://index.ros.org/doc/ros2/Installation/) source installation guide to get the ROS repositories needed to cross compile.

Once you have the desired sources, copy them in the `sysroot` to use with the tool.
```bash
# Copy ros sources into the sysroot directory
cp -r <full_path_to_your_ros_ws>/src sysroot/ros_ws/src
```

#### Run the cross compilation script

In the end your `sysroot` directory should look like this:

```bash
sysroot/
 +-- qemu-user-static/
 |   +-- qemu-*-static
 +-- ros_ws/
     +-- src/
          |-- (ros packages)
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

### Custom Data Directory

Your Custom Setup Script (see above) may need some data that is not accessible within the sysroot creation environment.
For example, you have custom rosdep rules files that need to be installed to find your dependencies.
For this use case, you can use the option `--custom-data-dir` to point to an arbitrary path, which will be copied into the environment, and available for use by your custom setup script.
Within the custom setup script, the data is available at `./custom-data/`

**Example:**

Custom data directory (`/arbitrary/local/directory`)
```
/arbitrary/local/directory/
+-- my-data/
|   +-- something.txt
```

Setup Script (`/path/to/custom-setup.sh`)

```
#!/bin/bash
cat custom-data/something.txt
```

Tool invocation:

```
ros2 run cross_compile cross_compile --sysroot-path /absolute/path/to/sysroot \
                                     --arch aarch64 --os ubuntu \
                                     --custom-setup-script /path/to/custom-setup.sh \
                                     --custom-data-dir /arbitrary/local/directory
```

Now, during the sysroot creation process, you should see the contents of `something.txt` printed during the execution of the custom script.

NOTE: For trivial text files, as in the example above, you could just as easily create those files fully within the `--custom-setup-script`. However, for binary data such as precompiled libraries, this feature comes to the rescue.


## Tutorial

For a new user, this section walks you through a representative use case, step by step.

In this tutorial, we will cross-compile the File Talker tool https://github.com/ros-tooling/file_talker against ROS2 Dashing, to run on an ARM64 Ubuntu system.
This workflow can be generalized to any `.repos` file for your project.

NOTE: This tutorial currently assumes a Debian-based (including Ubuntu) Linux distribution as the host platform.

### Creating a cross-compilation workspace

1. Create a directory for your workspace
    * `mkdir cross_compile_ws`
    * `cd cross_compile_ws`
1. Create a `.repos` file for `vcs`
    * `file_talker.repos`
    ```
    repositories:
      file_talker:
        type: git
        url: https://github.com/ros-tooling/file_talker.git
        version: master
    ```
1. Set up the sysroot environment for the cross-compiler to use
    * `mkdir -p sysroot/qemu-user-static/`
    * `mkdir -p sysroot/ros_ws/src/`
    * `cp /usr/bin/qemu-*-static sysroot/qemu-user-static`
    * `vcs import ros_ws/src < file_talker.repos`

### Running the Cross-Compilation

```
ros2 run cross_compile cross_compile \  # Invoke the cross-compilation tool
  --sysroot-path $(pwd) \
  --rosdistro dashing \
  --arch aarch64 \
  --os ubuntu
```

Let's run through the arguments we passed to the script:

* `--sysroot-path $(pwd)`

We are pointing the `cross_compile` tool to the absolute path of the directory containing the `sysroot` directory we created.
You could run the tool from any directory, but in this case we were already in the directory that contained sysroot, hence `$(pwd)`

* `--rosdistro dashing`

Both ROS and ROS2 distros can be specified by name (e.g. kinetic).
`cross_compile -h` will print the supported distros for this option

* `--arch aarch64`

We are targeting the ARMv8 / ARM64 / aarch64 architecture (which are different names for the same thing).
`cross_compile -h` will print the supported architectures for this option.

* `--os ubuntu`

The target OS is Ubuntu - the version of the OS will be chosen automatically based on the ROS Distro's target OS.
In our case for ROS2 Dashing, it is 18.04 Bionic Beaver.

### Outputs of the build

Run the following command

```
ls sysroot/ros_ws/
```

If the build succeeded, the directory will look like this:

```
ros_ws/
+-- src/
    |-- file_talker
+-- install_aarch64/
    |-- ...
```

The created directory `install_aarch64` is the installation of your ROS workspace for your target architecture.
You can double check this for yourself:

```
$ file ros_ws/install_aarch64/lib/file_talker/file_talker                                                               0s
ros_ws/install_aarch64/lib/file_talker/file_talker: ELF 64-bit LSB shared object, ARM aarch64, version 1 (GNU/Linux), dynamically linked, interpreter /lib/ld-, for GNU/Linux 3.7.0, BuildID[sha1]=02ede8a648dfa6b5b30c03d54c6d87fd9151389e, not stripped
```

### Using the build on a target platform

Copy `install_aarch64` onto the target system into a location of your choosing. It contains all of the binaries for _your_ workspace.

If your workspace has any dependencies that are outside the source tree - that is, if `rosdep` had anything to install during the build - then you will still need to install these dependencies on the target system.

```
# Run this on the target system, which must have rosdep already installed
# remember `rosdep init`, `rosdep update`, `apt-get update` if you need them
rosdep install --from-paths install_aarch64/share --ignore-src --rosdistro dashing -y
```

Now you may use the ROS installation as you normally would

```
source install_aarch64/setup.bash
ros2 run file_talker file_talker my_text_file.txt
```


## License

This library is licensed under the Apache 2.0 License.

## Build status

| ROS 2 Release | Branch Name     | Development | Source Debian Package | X86-64 Debian Package | ARM64 Debian Package | ARMHF Debian package |
| ------------- | --------------- | ----------- | --------------------- | --------------------- | -------------------- | -------------------- |
| Latest        | `master`        | [![Test Pipeline Status](https://github.com/ros-tooling/cross_compile/workflows/Test%20cross_compile/badge.svg)](https://github.com/ros-tooling/cross_compile/actions) | N/A                   | N/A                   | N/A                  | N/A                  |
| Dashing       | `dashing-devel` | [![Build Status](http://build.ros2.org/buildStatus/icon?job=Ddev__cross_compile__ubuntu_bionic_amd64)](http://build.ros2.org/job/Ddev__cross_compile__ubuntu_bionic_amd64) | [![Build Status](http://build.ros2.org/buildStatus/icon?job=Dsrc_uB__cross_compile__ubuntu_bionic__source)](http://build.ros2.org/job/Dsrc_uB__cross_compile__ubuntu_bionic__source) | [![Build Status](http://build.ros2.org/buildStatus/icon?job=Dbin_uB64__cross_compile__ubuntu_bionic_amd64__binary)](http://build.ros2.org/job/Dbin_uB64__cross_compile__ubuntu_bionic_amd64__binary) | N/A | N/A |


[ros2_dev_setup]: https://index.ros.org/doc/ros2/Installation/Latest-Development-Setup/
