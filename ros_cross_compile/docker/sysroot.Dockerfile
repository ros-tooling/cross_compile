# This dockerfile takes ROS 1 or 2 source packages from ${ROS_WORKSPACE}/ros_ws/src
# and builds them for the specified target platform.
# It uses qemu user-mode static emulation libraries from ${ROS_WORKSPACE}/qemu-user-static/
# to emulate the target platform.

# Assumptions: ros_ws/src and qemu-user-static directories are present in ${ROS_WORKSPACE}.

ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG ROS_WORKSPACE
ARG ROS_VERSION
ARG ROS_DISTRO
ARG TARGET_ARCH

ENV ROS_DISTRO=${ROS_DISTRO}
ENV TARGET_ARCH=${TARGET_ARCH}

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
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list

# Buildtool dependencies
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
  ; fi

# Install Fast-RTPS dependencies for ROS 2
RUN if [[ "${ROS_VERSION}" == "ros2" ]]; then \
    apt-get update && apt-get install --no-install-recommends -y \
        libasio-dev \
        libtinyxml2-dev \
    && rm -rf /var/lib/apt/lists/* \
  ; fi

# Run arbitrary user setup (copy data and run script)
COPY user-custom-data/ custom-data/
COPY user-custom-setup .
RUN chmod +x ./user-custom-setup && \
    ./user-custom-setup && \
    rm -rf /var/lib/apt/lists/*

# Copy in ROS dependency installation script
WORKDIR /ros_ws
COPY ${ROS_WORKSPACE}/cc_internals .
# This script was created by the rosdep gathering step
RUN apt-get update && \
    ./install_rosdeps.sh && \
    rm -rf /var/lib/apt/lists/*

# Set up build tools for the workspace
COPY mixins/ mixins/
RUN colcon mixin add cc_mixin file://$(pwd)/mixins/index.yaml && colcon mixin update cc_mixin
# In case the workspace did not actually install any dependencies, add these for uniformity
RUN mkdir -p /opt/ros/${ROS_DISTRO} && \
    touch /opt/ros/${ROS_DISTRO}/setup.sh && \
    touch /opt/ros/${ROS_DISTRO}/setup.bash
COPY build_workspace.sh /root
ENTRYPOINT ["/root/build_workspace.sh"]
