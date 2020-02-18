# This Dockerfile creates an image with all build tools and dependencies
# preinstalled for a target workspace.
#
# When `run` against mounted sources, it performs a build for the target OS and architecture.
# It uses qemu user-mode static emulation libraries to emulate the target platform for the build.
# Required build arguments:
#  - BASE_IMAGE: the base Docker image to start from, e.g. arm64v8/ubuntu:bionic
#  - DEPENDENCY_SCRIPT_PATH: a path (relative to the PWD of the `docker build` command) of a script that installs the source workspace's dependencies
#  - ROS_VERSION: "ros" or "ros2"

ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG DEPENDENCY_SCRIPT_PATH
ARG ROS_VERSION

SHELL ["/bin/bash", "-c"]

COPY qemu-user-static/ /usr/bin/

# Set timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -sf /usr/share/zoneinfo/Etc/UTC /etc/localtime

RUN apt-get update && apt-get install -y \
        tzdata \
        locales \
    && rm -rf /var/lib/apt/lists/*

# Set locale
RUN echo 'en_US.UTF-8 UTF-8' >> /etc/locale.gen && \
    locale-gen && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LC_ALL C.UTF-8

# Add the ros apt repo
RUN apt-get update && apt-get install -y \
        gnupg2 \
        lsb-release \
    && rm -rf /var/lib/apt/lists/*
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' \
    --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get clean && apt-get update

RUN echo "deb http://packages.ros.org/${ROS_VERSION}/ubuntu `lsb_release -cs` main" \
    > /etc/apt/sources.list.d/${ROS_VERSION}-latest.list

# ROS dependencies
RUN apt-get update && apt-get install -y \
      build-essential \
      cmake \
      git \
      python3-colcon-common-extensions \
      python3-colcon-mixin \
      python3-pip \
      wget \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install -U \
    setuptools

# Install some pip packages needed for testing ROS 2
RUN if [[ "${ROS_VERSION}" == "ros2" ]]; then \
    python3 -m pip install -U \
    argcomplete \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest \
    pytest-cov \
    pytest-runner \
    vcstool; \
  fi

# Install Fast-RTPS dependencies for ROS 2
RUN if [[ "${ROS_VERSION}" == "ros2" ]]; then \
    apt-get update && apt-get install --no-install-recommends -y \
        libasio-dev \
        libtinyxml2-dev \
    && rm -rf /var/lib/apt/lists/*; \
  fi

# Run arbitrary user setup (copy data and run script)
COPY user-custom-data/ custom-data/
COPY user-custom-setup .
RUN chmod +x ./user-custom-setup && \
    ./user-custom-setup && \
    rm -rf /var/lib/apt/lists/*

# Use generated rosdep installation script
COPY ${DEPENDENCY_SCRIPT_PATH} ${DEPENDENCY_SCRIPT_PATH}
RUN apt-get update && \
    ./"${DEPENDENCY_SCRIPT_PATH}" && \
    rm -rf /var/lib/apt/lists/*

# Set up build tools for the workspace
COPY mixins/ mixins/
RUN colcon mixin add cc_mixin file://$(pwd)/mixins/index.yaml && colcon mixin update cc_mixin
COPY build_workspace.sh /root
WORKDIR /ros_ws
ENTRYPOINT ["/root/build_workspace.sh"]