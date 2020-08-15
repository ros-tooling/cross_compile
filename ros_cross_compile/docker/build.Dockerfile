FROM ubuntu:focal
ENV DEBIAN_FRONTEND=noninteractive

# Common for all
RUN apt-get update && apt-get install --no-install-recommends -q -y \
    build-essential \
    cmake \
    python3-pip \
    wget

RUN pip3 install colcon-common-extensions colcon-mixin

# Specific at the end (layer sharing)
RUN apt-get update && apt-get install --no-install-recommends -q -y \
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu

RUN apt-get update && apt-get install -q -y --no-install-recommends rsync

RUN pip3 install lark-parser numpy

# Fast and small, no optimization necessary
COPY mixins/ /mixins/
COPY build_workspace.sh /root
COPY toolchains/ /toolchains/
WORKDIR /ros_ws
ENTRYPOINT ["/root/build_workspace.sh"]
