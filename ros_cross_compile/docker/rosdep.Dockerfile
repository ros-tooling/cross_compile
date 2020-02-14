# This Dockerfile describes a layer on a base OS that has:
#  * Access to the ROS apt repositories
#  * rosdep installed
# When `run` against a mounted ROS workspace, output a script that installs its dependencies
# This image is intended to run in the host architecture for speed -
# rosdep doesn't differentiate architecture, just OS,
# so we can run natively on the same target OS and get the same results
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

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

RUN apt-get update && apt-get install -y \
      python-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init
COPY gather_rosdeps.sh /root/
RUN mkdir -p /root/ws
WORKDIR /root/ws
ENTRYPOINT ["/root/gather_rosdeps.sh"]
