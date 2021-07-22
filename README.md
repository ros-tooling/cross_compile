# ROS / ROS 2 Cross Compile Tool

![License](https://img.shields.io/github/license/ros-tooling/cross_compile)
[![Documentation Status](https://readthedocs.org/projects/cross_compile/badge/?version=latest)](https://cross_compile.readthedocs.io/en/latest/?badge=latest)

A tool to automate compiling ROS and ROS 2 workspaces to non-native architectures.

:construction: `ros_cross_compile` relies on running emulated builds
using QEmu, #69 tracks progress toward enabling cross-compilation.


## Supported targets

This tool supports compiling a workspace for all combinations of the following:

* Architecture: `armhf`, `aarch64`, `x86_64`
* ROS Distro
  * ROS: `melodic`, `noetic`
  * ROS 2: `dashing`, `foxy`, `galactic`, `rolling`
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
* Python 3.6 or higher

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
1. (Optional) Create runtime image
    1. Creates a docker image that can be used on the target platform to run the build. See "Runtime Image" section.

## Usage

This package installs the `ros_cross_compile` command.
The command's first argument is the path to your ROS workspace.

Here is a simple invocation for a standard workflow.

```bash
ros_cross_compile /path/to/my/workspace --arch aarch64 --os ubuntu --rosdistro foxy
```

For information on all available options, run `ros_cross_compile -h`.
See the following sections for information on the more complex options.

### Package Selection and Build Customization

To choose which packages to install dependencies for, this tool runs `colcon list` on your workspace.
To build, it runs `colcon build`.

You can provide arbitrary arguments to these commands via the [colcon `defaults.yaml`](https://colcon.readthedocs.io/en/released/user/configuration.html#defaults-yaml).

You can either specify the name of this file via `ros_cross_compile --colcon-defaults /path/to/defaults.yaml`, or if not specified, a file called `defaults.yaml` will be used if present.

For example, there are repositories checked out in your workspace that contain packages that are not needed for your application - some repos provide many packages and you may only want one!
In this scenario there is a "bringup" package that acts as the entry point to your application:

```yaml
# my_workspace/defaults.yaml
list:
  # only install dependencies for source packages that my package depends on
  packages-up-to: [my_application_bringup]
build:
  # only build up to my package
  packages-up-to: [my_application_bringup]
  # example of a boolean commandline argument
  merge-install: true
```

Other configurations can be passed and used as command line args. Examples are CMake build arguments, like the build type or the verb configurations for the event handlers:

```yaml
# my_workspace/defaults.yaml
build:
  cmake-args: ["-DCMAKE_BUILD_TYPE=Release"]
  event-handlers: ["console_direct+"]
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
```

Tool invocation for this example:

```bash
ros_cross_compile /path/to/my/workspace --arch aarch64 --os ubuntu \
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

Additionally, a custom setup script may control the build environment by populating the `/custom-data/setup.bash` file which will be sourced before building.

### Custom post-build script

You may want to perform arbitrary post-processing on your build outputs, in the event of a sucessful build - use `--custom-post-build-script` for this.
Keep in mind that it is run at the root of the built workspace.

Following is an example setup that allows a user to run [colcon bundle](https://github.com/colcon/colcon-bundle) to create a portable bundle of the cross-compiled application.

Here are the contents of `./postbuild.sh`

```bash
#!/bin/bash
set -eux

apt-get update
apt-get install -y wget
wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

apt-get install -y python3-apt
pip3 install -u setuptools pip
pip3 install -U colcon-ros-bundle

colcon bundle \
  --build-base build_"${TARGET_ARCH}" \
  --install-base install_"${TARGET_ARCH}" \
  --bundle-base bundle_"${TARGET_ARCH}"
```

Now, run

```
ros_cross_compile /path/to/my/workspace --arch aarch64 --os ubuntu \
  --custom-post-build-script ./postbuild.sh
```

After the build completes, you should see the bundle outputs in `bundle_aarch64`


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
ros_cross_compile /path/to/my/workspace --arch aarch64 --os ubuntu \
  --custom-setup-script /path/to/custom-setup.sh \
  --custom-data-dir /arbitrary/local/directory
```

Now, during the sysroot creation process, you should see the contents of `something.txt` printed during the execution of the custom script.

NOTE: for trivial text files, as in the preceding example, you could have created those files fully within the `--custom-setup-script`. But for large or binary data such as precompiled libraries, this feature comes to the rescue.


### Runtime Image

`ros_cross_compile` can optionally create and tag a Docker image that contains the build output and its runtime dependencies.

The argument `--runtime-tag` takes a single value, which is the tag used for the output image.

```
OUTPUT_IMAGE=my_registry/image_name:image_tag
ros_cross_compile $workspace --runtime-tag $OUTPUT_IMAGE
```

One way to deploy this image is to push it to a registry, from where it can be pulled onto a target platform

```
docker push $OUTPUT_IMAGE
```

The image contains any necessary emulation binaries to run locally if desired for smoke testing.

```
docker run -it $OUTPUT_IMAGE
# In the shell inside the running container, the setup is already sourced for the default entrypoint
ros2 launch my_package my.launch.py
```

Note: Currently this feature is a thin layer on top of the image used for building, so it is not a fully minimal image - it contains build tools, build dependencies, and test dependencies in addition to the necessary runtime dependencies.
Future work is planned to slim down this output image to a properly minimal runtime.
This work is tracked in https://github.com/ros-tooling/cross_compile/issues/263.


## Tutorial

For a new user, this section walks you through a representative use case, step by step.

This tutorial demonstrates how to cross-compile the [ROS 2 Demo Nodes](https://github.com/ros-tooling/demos) against ROS 2 Foxy, to run on an ARM64 Ubuntu system.
You can generalize this workflow to use on any workspace for your project.

NOTE: this tutorial assumes a Debian-based (including Ubuntu) Linux distribution as the host platform.

### Creating a simple source workspace

Create a directory for your workspace and checkout the sources

```
mkdir -p cross_compile_ws/src
cd cross_compile_ws
git clone -b foxy https://github.com/ros2/demos src/demos
```

Create a file `defaults.yaml` in this directory with the following contents. This file narrows down the set of built packages, rather than building every single package in the source repository. This file is optional - see preceding section "Package Selection and Build Customization
" for more information.

```
build:
  # only build the demo_nodes_cpp package, to save time building all of the demos
  packages-up-to:
    - demo_nodes_cpp
```

### Running the cross-compilation

```bash
ros_cross_compile . --rosdistro foxy --arch aarch64 --os ubuntu
```

Here is a detailed look at the arguments passed to the script (`ros_cross_compile -h` will print all valid choices for each option):

* `.`
  * The first argument to `ros_cross_compile` is the directory of the workspace to be built. This could be any relative or absolute path, in this case it's just `.`, the current working directory.
* `--rosdistro foxy`
  * You may specify either a ROS and ROS 2 distribution by name, for example `noetic` (ROS) or `galactic` (ROS 2).
* `--arch aarch64`
  * Target the ARMv8 / ARM64 / aarch64 architecture (which are different names for effectively the same thing).
* `--os ubuntu`
  * The target OS is Ubuntu - the tool chooses the OS version automatically based on the ROS Distro's target OS. In this case for ROS 2 Foxy - Ubuntu 20.04 Focal Fossa.

### Outputs of the build

Run the following command

```bash
ls cross_compile_ws
```

If the build succeeded, the directory looks like this:

```
build_aarch64/
cc_internals/
defaults.yaml
install_aarch64/
log/
src/
tutorial.repos
```

* The created directory `install_aarch64` is the installation of your ROS workspace for your target architecture.
* `cc_internals` is used by `ros_cross_compile` to cache artifacts between builds - as a user you will not need to inspect it

You can verify that the build created binaries for the target architecture (note "ARM aarch64" in below output. Your `sha1` may differ):

```bash
$ file install_aarch64/demo_nodes_cpp/lib/demo_nodes_cpp/talker
install_aarch64/demo_nodes_cpp/lib/demo_nodes_cpp/talker: ELF 64-bit LSB shared object, ARM aarch64, version 1 (SYSV), dynamically linked, interpreter /lib/ld-linux-aarch64.so.1, BuildID[sha1]=f086db477d6f5f919414d63911366077f1051b80, for GNU/Linux 3.7.0, not stripped
```

### Using the build on a target platform

Copy `install_aarch64` onto the target system into a location of your choosing. It contains the binaries for _your_ workspace.

If your workspace has any dependencies that are outside the source tree - that is, if `rosdep` had anything to install during the build - then you still need to install these dependencies on the target system.

```bash
# Run this on the target system, which must have rosdep already installed
# remember `rosdep init`, `rosdep update`, `apt-get update` if you need them
rosdep install --from-paths install_aarch64/share --ignore-src --rosdistro foxy -y
```

Now you may use the ROS installation as you would on any other system

```bash
source install_aarch64/setup.bash
ros2 run demo_nodes_cpp talker

# and in a different shell
ros2 run demo_nodes_cpp listener
```

## Troubleshooting

If you are running in docker with `/var/run/docker.sock` mounted and see the following error:
> No src/ directory found at /ws, did you remember to mount your workspace?

You may need to try running in docker-in-docker. This approach is demonstrated to work in gitlab-ci with a privileged runner and the following `gitlab.yml` as an example:

```yaml
image: teracy/ubuntu:18.04-dind-19.03.3

services:
  - docker:19.03.3-dind

variables:
  # Disable TLS or we get SSLv1 errors. We shouldn't need this since we mount the /certs volume.
  # We also need to connect to the docker daemon via DOCKER_HOST.
  DOCKER_TLS_CERTDIR: ""
  DOCKER_HOST: tcp://docker:2375

build-stuff:
  stage: build
  tags:
    - ros
  before_script:
    # Install packages
    - apt update
    - apt install -qq -y qemu-user-static python3-pip rsync

    # Set up the workspace
    - cd ${CI_PROJECT_DIR}/..
    - rm -rf cross_compile_ws/src
    - mkdir -p cross_compile_ws/src
    - cp -r ${CI_PROJECT_DIR} cross_compile_ws/src/
    - rsync -a ${CI_PROJECT_DIR}/../cross_compile_ws ${CI_PROJECT_DIR}
    - cd ${CI_PROJECT_DIR}

    # Install ros_cross_compile
    - pip3 install ros_cross_compile
  script:
    - ros_cross_compile cross_compile_ws --arch aarch64 --os ubuntu --rosdistro melodic
  artifacts:
    paths:
      - $CI_PROJECT_DIR/cross_compile_ws/install_aarch64
    expire_in: 1 week
```

## License

This library is licensed under the Apache 2.0 License.

## Build status

| ROS 2 Release | Branch Name     | Development | Source Debian Package | X86-64 Debian Package | ARM64 Debian Package | ARMHF Debian package |
| ------------- | --------------- | ----------- | --------------------- | --------------------- | -------------------- | -------------------- |
| Latest        | `master`        | [![Test Pipeline Status](https://github.com/ros-tooling/cross_compile/workflows/Test%20cross_compile/badge.svg)](https://github.com/ros-tooling/cross_compile/actions) | N/A                   | N/A                   | N/A                  | N/A                  |
| Dashing       | `dashing-devel` | [![Build Status](http://build.ros2.org/buildStatus/icon?job=Ddev__cross_compile__ubuntu_bionic_amd64)](http://build.ros2.org/job/Ddev__cross_compile__ubuntu_bionic_amd64) | [![Build Status](http://build.ros2.org/buildStatus/icon?job=Dsrc_uB__cross_compile__ubuntu_bionic__source)](http://build.ros2.org/job/Dsrc_uB__cross_compile__ubuntu_bionic__source) | [![Build Status](http://build.ros2.org/buildStatus/icon?job=Dbin_uB64__cross_compile__ubuntu_bionic_amd64__binary)](http://build.ros2.org/job/Dbin_uB64__cross_compile__ubuntu_bionic_amd64__binary) | N/A | N/A |


[ros2_dev_setup]: https://index.ros.org/doc/ros2/Installation/Latest-Development-Setup/
