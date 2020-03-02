# ROS / ROS 2 Cross Compile Tool

![License](https://img.shields.io/github/license/ros-tooling/cross_compile)
[![Documentation Status](https://readthedocs.org/projects/cross_compile/badge/?version=latest)](https://cross_compile.readthedocs.io/en/latest/?badge=latest)

A tool to automate compiling ROS and ROS 2 workspaces to non-native architectures.

:construction: `ros_cross_compile` relies on running emulated builds
using QEmu, #69 tracks progress toward enabling cross-compilation.


## Supported targets

This tool supports compiling a workspace for all combinations of the following:

* Architecture: `armhf`, `aarch64`
* ROS Distro
  * ROS: `kinetic`, `melodic`
  * ROS 2: `dashing`, `eloquent`
* OS: `Ubuntu`, `Debian`

NOTE: ROS 2 supports Debian only as a Tier 3 platform.
This means that there are not `apt` repositories available for the ROS 2 Core on this platform.
Because of that, when targeting Debian for a ROS 2 workspace, you must also include the source for the core as well.
It is recommended to use a release branch of `ros2.repos` from https://github.com/ros2/ros2 to do so, rather than `master`, so that you are not affected by development branch bugs and API changes.

## Supported hosts

This tool officially supports running on the following host systems.
Note that many others likely work, but these are being thoroughly tested.

* Ubuntu 18.04 Bionic Beaver
* OSX Mojave

## Installation

### Prerequisites

This tool requires that you have already installed
* [Docker](https://docs.docker.com/install/)
  * Follow the instructions to add yourself to the `docker` group as well, so you can run containers as a non-root user
* Python 3.5 or higher

If you are using a Linux host, you must also install QEmu (Docker for OSX performs emulation automatically):

```sh
sudo apt-get install qemu-user-static
```

### Installing ros_cross_compile

To install the stable release,

```sh
pip3 install ros_cross_compile
```

If you would like the latest nightly build, you can get it from Test PyPI

```sh
pip3 install --index-url https://test.pypi.org/simple/ ros_cross_compile
```

## How it works, high level

1. Collect dependencies
    1. Create a Docker image that has `rosdep`
    1. Run the `rosdep` image against your target workspace to output a script that describes how to install its dependencies
1. Create "sysroot image" that has everything needed for building target workspace
    1. Use a base image for the target architecture (aarch64, armhf, ...)
    1. Install build tools (compilers, cmake, colcon, etc)
    1. Run the dependency installer script collected in Step 1 (if dependency list hasn't changed since last run, this uses the Docker cache)
1. Build
    1. Runs the "sysroot image" using QEmu emulation
    1. `colcon build`

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


#### Prepare `ros_ws`
Use [ROS](http://wiki.ros.org/ROS/Installation) or [ROS 2](https://index.ros.org/doc/ros2/Installation/) source installation guide to get the ROS repositories needed to cross compile.

Once you have the desired sources, copy them in the `sysroot` to use with the tool.
```bash
# Copy ros sources into the sysroot directory
cp -r <full_path_to_your_ros_ws>/src sysroot/ros_ws
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
python3 -m ros_cross_compile \
  --sysroot-path /absolute/path/to/sysroot \
  --arch aarch64 \
  --os ubuntu
  --rosdistro dashing
```

### Custom rosdep script

Your ROS application may need nonstandard rosdep rules.
If so, you have the option to provide a script to be run before the `rosdep install` command collects keys.

This script has access to the "Custom data directory" same as the "Custom setup script", see the following sections. If you need any extra files for setting up rosdep, they can be accessed via this custom data directory.

Note that:
1. Rosdeps are always collected in an Ubuntu Bionic container, so scripts must be compatible with that

Here is an example script for an application that adds extra rosdep source lists

```bash
cp ./custom-data/rosdep-rules/raspicam-node.yaml /etc/ros/rosdep/custom-rules/raspicam-node.yaml
echo "yaml file:/etc/ros/rosdep/custom-rules/raspicam-node.yaml" > /etc/ros/rosdep/sources.list.d/22-raspicam-node.list
echo "yaml https://s3-us-west-2.amazonaws.com/rosdep/python.yaml" > /etc/ros/rosdep/sources.list.d/18-aws-python.list
```

Tool invocation for this example:

```bash
python3 -m ros_cross_compile \
  --sysroot-path /absolute/path/to/directory/containing/sysroot
  --arch aarch64 --os ubuntu \
  --custom-rosdep-script /path/to/rosdep-script.sh \
  --custom-data-dir /arbitrary/local/directory
```

### Custom setup script

Your ROS application may have build needs that aren't covered by `rosdep install`.
If this is the case (for example you need to add extra apt repos), use the option `--custom-setup-script` to execute arbitrary code in the sysroot container.

The path provided may be absolute, or relative to the current directory.

Keep in mind
* It's up to the user to determine whether the script is compatible with chosen base platform
* Make sure to specify non-interactive versions of commands, for example `apt-get install -y`, or the script may hang waiting for input
* You cannot make any assumptions about the state of the apt cache, so run `apt-get update` before installing packages
* The script runs as root user in the container, so you don't need `sudo`

Below is an example script for an application that installs some custom Raspberry Pi libraries.

```bash
apt-get update
apt-get install -y software-properties-common

# Install Raspberry Pi library that we have not provided a rosdep rule for
add-apt-repository ppa:rpi-distro/ppa
apt-get install -y pigpio
```

### Custom data directory

Your custom setup or rosdep script (see preceding sections) may need some data that is not otherwise accessible.
For example, you need to copy some precompiled vendor binaries to a specific location, or provide custom rosdep rules files.
For this use case, you can use the option `--custom-data-dir` to point to an arbitrary path.
The sysroot build copies this directory into the build environment, where it's available for use by your custom setup script at `./custom-data/`.

**Example:**

Custom data directory (`/arbitrary/local/directory`)
```
/arbitrary/local/directory/
+-- my-data/
|   +-- something.txt
```

Setup Script (`/path/to/custom-setup.sh`)

```bash
#!/bin/bash
cat custom-data/something.txt
```

Tool invocation:

```bash
python3 -m ros_cross_compile \
  --sysroot-path /absolute/path/to/directory/containing/sysroot
  --arch aarch64 --os ubuntu \
  --custom-setup-script /path/to/custom-setup.sh \
  --custom-data-dir /arbitrary/local/directory
```

Now, during the sysroot creation process, you should see the contents of `something.txt` printed during the execution of the custom script.

NOTE: for trivial text files, as in the preceding example, you could have created those files fully within the `--custom-setup-script`. But for large or binary data such as precompiled libraries, this feature comes to the rescue.


## Tutorial

For a new user, this section walks you through a representative use case, step by step.

This tutorial demonstrates how to cross-compile the [File Talker tool](https://github.com/ros-tooling/file_talker) against ROS 2 Dashing, to run on an ARM64 Ubuntu system.
You can generalize this workflow to any `.repos` file for your project.

NOTE: this tutorial assumes a Debian-based (including Ubuntu) Linux distribution as the host platform.

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

### Running the cross-compilation

```bash
python3 -m ros_cross_compile \
  --sysroot-path $(pwd) \
  --rosdistro dashing \
  --arch aarch64 \
  --os ubuntu
```

Here is a detailed look at the arguments passed to the script:

* `--sysroot-path $(pwd)`

Point the `ros_cross_compile` tool to the absolute path of the directory containing the `sysroot` directory created earlier.
You could run the tool from any directory, but in this case the current working directory contains `sysroot`, hence `$(pwd)`

* `--rosdistro dashing`

You may specify both ROS and ROS 2 distributions by name, for example, `kinetic` (ROS) or `dashing` (ROS 2).
`ros_cross_compile -h` prints the supported distributions for this option

* `--arch aarch64`

Target the ARMv8 / ARM64 / aarch64 architecture (which are different names for the same thing).
`ros_cross_compile -h` prints the supported architectures for this option.

* `--os ubuntu`

The target OS is Ubuntu - the tool chooses the OS version automatically based on the ROS Distro's target OS.
In this case for ROS 2 Dashing - 18.04 Bionic Beaver.

### Outputs of the build

Run the following command

```bash
ls sysroot/ros_ws/
```

If the build succeeded, the directory looks like this:

```
ros_ws/
+-- src/
    |-- file_talker
+-- install_aarch64/
    |-- ...
```

The created directory `install_aarch64` is the installation of your ROS workspace for your target architecture.
You can verify this:

```bash
$ file ros_ws/install_aarch64/lib/file_talker/file_talker                                                               0s
ros_ws/install_aarch64/lib/file_talker/file_talker: ELF 64-bit LSB shared object, ARM aarch64, version 1 (GNU/Linux), dynamically linked, interpreter /lib/ld-, for GNU/Linux 3.7.0, BuildID[sha1]=02ede8a648dfa6b5b30c03d54c6d87fd9151389e, not stripped
```

### Using the build on a target platform

Copy `install_aarch64` onto the target system into a location of your choosing. It contains the binaries for _your_ workspace.

If your workspace has any dependencies that are outside the source tree - that is, if `rosdep` had anything to install during the build - then you still need to install these dependencies on the target system.

```bash
# Run this on the target system, which must have rosdep already installed
# remember `rosdep init`, `rosdep update`, `apt-get update` if you need them
rosdep install --from-paths install_aarch64/share --ignore-src --rosdistro dashing -y
```

Now you may use the ROS installation as you would on any other system

```bash
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
