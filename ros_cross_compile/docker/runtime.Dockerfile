# This dockerfile creates an image containing the runtime environment for the workspace.
# For now this is a thin layer on top of the sysroot/buildroot environment used to build.
# It re-sets the entrypoint to a shell instead of a build script, and copies in the install.
# The eventual plan for it is to only contain `<exec_depends>` of the workspace

ARG BASE_IMAGE
FROM $BASE_IMAGE

WORKDIR /ros_ws

RUN apt-get update && apt-get install -q -y --no-install-recommends less vim

ARG INSTALL_PATH
COPY $INSTALL_PATH/ install

RUN echo "source /ros_ws/install/setup.bash" >> ~/.bashrc
ENTRYPOINT /bin/bash
