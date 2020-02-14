# This Dockerfile describes a simple image with rosdep installed.
# When `run`, it outputs a script for installing dependencies for a give workspace
# Requirements:
#  * mount a colcon workspace at /root/ws
#  * set environment variable ROS_DISTRO (e.g. ROS_DISTRO=dashing)
#  * set environment variable TARGET_OS (this is passed directly to rosdep insall --os=, so it is in the format OS_NAME:OS_VERSION, e.g. "ubuntu:bionic")
FROM ubuntu:bionic

RUN apt-get update && apt-get install -y \
      gnupg2 \
    && rm -rf /var/lib/apt/lists/*
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get update && apt-get install -y \
      python-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init
COPY gather_rosdeps.sh /root/
RUN mkdir -p /root/ws
WORKDIR /root/ws
ENTRYPOINT ["/root/gather_rosdeps.sh"]
