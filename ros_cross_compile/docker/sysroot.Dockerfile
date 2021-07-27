# This file describes an image that has everything necessary installed to build a target ROS workspace
# It uses QEmu user-mode emulation to perform dependency installation and build
# Assumptions: qemu-user-static directory is present in docker build context

ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG ROS_VERSION

SHELL ["/bin/bash", "-c"]

COPY bin/* /usr/bin/

# Set timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -sf /usr/share/zoneinfo/Etc/UTC /etc/localtime

RUN apt-get update && apt-get install --no-install-recommends -y \
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
RUN apt-get update && apt-get install --no-install-recommends -y \
        ca-certificates \
        curl \
        dirmngr \
        gnupg2 \
        lsb-release \
    && rm -rf /var/lib/apt/lists/*
RUN if [[ "${ROS_VERSION}" == "ros2" ]]; then \
      curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg; \
      echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/${ROS_VERSION}/ubuntu `lsb_release -cs` main" | \
          tee /etc/apt/sources.list.d/ros2.list > /dev/null; \
    else \
      curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - ; \
      echo "deb http://packages.ros.org/${ROS_VERSION}/ubuntu `lsb_release -cs` main" \
          > /etc/apt/sources.list.d/${ROS_VERSION}-latest.list; \
    fi

# ROS dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
      build-essential \
      cmake \
      python3-colcon-common-extensions \
      python3-colcon-mixin \
      python3-dev \
      python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install -U \
    setuptools

# Install some pip packages needed for testing ROS 2
RUN if [[ "${ROS_VERSION}" == "ros2" ]]; then \
    python3 -m pip install -U \
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

# Use generated rosdep installation script
COPY install_rosdeps.sh .
RUN chmod +x install_rosdeps.sh
RUN export DEBIAN_FRONTEND=noninteractive && apt-get update && \
    ./install_rosdeps.sh && \
    rm -rf /var/lib/apt/lists/*

# Copy colcon defaults config and set COLCON_DEFAULTS_FILE
COPY defaults.yaml /root
ENV COLCON_DEFAULTS_FILE=/root/defaults.yaml

# Set up build tools for the workspace
COPY mixins/ mixins/
RUN colcon mixin add cc_mixin file://$(pwd)/mixins/index.yaml && colcon mixin update cc_mixin
# In case the workspace did not actually install any dependencies, add these for uniformity
COPY build_workspace.sh /root
WORKDIR /ros_ws
COPY user-custom-post-build /
RUN chmod +x /user-custom-post-build
ENTRYPOINT ["/root/build_workspace.sh"]
