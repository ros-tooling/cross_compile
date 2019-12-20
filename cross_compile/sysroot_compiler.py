# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Contains all classes used by the sysroot_compiler.py script."""

import getpass
import logging
import os
from pathlib import Path
import shutil
from string import Template
import tarfile
import tempfile
from typing import Optional

import docker

ROS_WS_DIR_ERROR_STRING = Template(
    """"$ros_ws" does not exist in the sysroot directory. Make """
    """sure you copy your packages as "$ros_ws/src" into the """
    """"sysroot" directory."""
    )
QEMU_DIR_ERROR_STRING = Template(
    """"$qemu_dir" does not exist in the sysroot directory. Make """
    """sure you copy the binaries from "/usr/bin/qemu-*" into the """
    """sysroot directory."""
)

QEMU_EMPTY_ERROR_STRING = Template(
    """"$qemu_dir" is empty. Make sure you copy the binaries from """
    """"/usr/bin/qemu-*" into "$qemu_dir".""")

COPY_DOCKER_WS_ERROR_STRING = Template(
    """Unable to copy the "$dockerfile" file. Make sure you """
    """have write permissions to the sysroot directory."""
)

SYSROOT_NOT_FOUND_ERROR_STRING = Template(
    """Sysroot directory not found at "$sysroot_dir". Make """
    """sure you specify the full path to the directory containing "sysroot"."""
)

SYSROOT_DIR_NAME = 'sysroot'  # type: str
QEMU_DIR_NAME = 'qemu-user-static'  # type: str
ROS_DOCKERFILE_NAME = 'Dockerfile_ros'  # type: str
DOCKER_CLIENT = docker.from_env()
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class Platform:
    """
    A class that represents platform specification for cross compiling.

    Includes:
    1. Target architecture
    2. Target operating system
    3. ROS2 distribution used
    4. RMW implementation used
    """

    SUPPORTED_ROS2_DISTROS = ['dashing', 'eloquent']
    SUPPORTED_ROS_DISTROS = ['kinetic', 'melodic']

    def __init__(self, arch: str, os: str, rosdistro: str, rmw: str):
        """Initialize platform parameters."""
        self.arch = arch
        self.os = os
        self.rosdistro = rosdistro
        self.rmw = rmw

        if self.arch == 'armhf':
            self.cc_toolchain = 'arm-linux-gnueabihf'
        elif self.arch == 'aarch64':
            self.cc_toolchain = 'aarch64-linux-gnu'

        if self.rosdistro in self.SUPPORTED_ROS2_DISTROS:
            self.ros_version = 'ros2'
        elif self.rosdistro in self.SUPPORTED_ROS_DISTROS:
            self.ros_version = 'ros'

    def __str__(self):
        """Return string representation of platform parameters."""
        return '-'.join((self.arch, self.os, self.rosdistro))

    def get_workspace_image_tag(self) -> str:
        """Generate docker image name and tag."""
        return getpass.getuser() + '/' + str(self) + ':latest'


class DockerConfig:
    """
    Represents docker build parameters used in creating sysroot.

    Includes:
    1. Base docker image to use for building sysroot
    2. Docker network mode
    3. Setting to enable/disable caching during docker build
    """

    def __init__(
        self, arch: str, os: str, rosdistro: str, sysroot_base_image: str,
        docker_network_mode: str, sysroot_nocache: bool
    ):
        base_image = {
            'armhf': 'arm32v7',
            'aarch64': 'arm64v8',
        }
        image_tag = {
            'kinetic': {
                'ubuntu': 'xenial',
                'debian': 'jessie',
            },
            'melodic': {
                'ubuntu': 'bionic',
                'debian': 'stretch',
            },
            'dashing': {
                'ubuntu': 'bionic',
                'debian': 'stretch',
            },
            'eloquent': {
                'ubuntu': 'bionic',
                'debian': 'buster',
            }
        }

        """Initialize docker configuration."""
        if sysroot_base_image is None:
            self.base_image = \
                self.base_image = '{}/{}:{}'.format(base_image[arch], os, image_tag[rosdistro][os])
        else:
            self.base_image = sysroot_base_image

        self.network_mode = docker_network_mode
        self.nocache = sysroot_nocache

    def __str__(self):
        """Return string representation of docker build parameters."""
        return 'Base Image: {}\nNetwork Mode: {}\nCaching: {}'.format(
            self.base_image, self.network_mode, self.nocache)


class SysrootCompiler:
    """Build Docker containers for cross-compiling ROS2 packages."""

    def __init__(
      self,
      cc_root_dir: str,
      ros_workspace_dir: str,
      platform: Platform,
      docker_config: DockerConfig,
      custom_setup_script_path: Optional[str],
    ) -> None:
        """
        Construct a SysrootCompiler object building ROS 2 Docker container.

        :param cc_root_dir: The directory containing the 'sysroot' directory
                            with the ROS2 workspace and QEMU binaries.
        :param ros_workspace_dir: The name of the directory containing the
                                  ROS2 packages (inside a 'src' directory).
        :param platform: A custom object used to specify the the platform for
                         cross-compilation.
        :param docker_config: A custom object used to specify the configuration
                              of the Docker image to build.
        :param custom_setup_script_path: Optional path to a custom setup script
                                         to run arbitrary commands
        """
        if not isinstance(cc_root_dir, str):
            raise TypeError('Argument `cc_root_dir` must be of type string.')
        if not isinstance(ros_workspace_dir, str):
            raise TypeError(
                'Argument `ros_workspace_dir` must be of type string.')
        if not isinstance(platform, Platform):
            raise TypeError('Argument `platform` must be of type Platform.')
        if not isinstance(docker_config, DockerConfig):
            raise TypeError(
                'Argument `docker_config` must be of type DockerConfig.')

        workspace_root = Path(cc_root_dir).resolve()
        self._target_sysroot = workspace_root / SYSROOT_DIR_NAME
        self._ros_workspace_relative_to_sysroot = ros_workspace_dir
        self._ros_workspace_dir = self._target_sysroot / self._ros_workspace_relative_to_sysroot
        self._qemu_directory = self._target_sysroot / QEMU_DIR_NAME
        self._dockerfile_directory = Path(__file__).parent / ROS_DOCKERFILE_NAME
        self._expected_dockerfile_directory = (
            self._target_sysroot / ROS_DOCKERFILE_NAME)
        self._system_setup_script_path = Path()
        self._build_setup_script_path = Path()
        self._platform = platform
        self._docker_config = docker_config
        self._setup_sysroot_dir(custom_setup_script_path)

    def get_system_setup_script_path(self) -> Path:
        """Return the path to the system setup script."""
        return self._system_setup_script_path

    def get_build_setup_script_path(self) -> Path:
        """Return the path to the build setup script."""
        return self._build_setup_script_path

    def _setup_sysroot_dir(self, custom_script: Optional[str]) -> None:
        """
        Check to make sure the sysroot directory is setup correctly.

        Raises FileNotFoundError if any of the components necessary for
        cross compilation are missing. Copies the Dockerfile to the
        'sysroot' directory in order to copy the assets to it.
        See https://docs.docker.com/engine/reference/builder/#copy
        """
        logger.info('Checking sysroot directory...')
        if not self._target_sysroot.exists():
            raise FileNotFoundError(SYSROOT_NOT_FOUND_ERROR_STRING.substitute(
                sysroot_dir=self._target_sysroot))

        logger.debug('Sysroot directory exists.')
        if not self._ros_workspace_dir.exists():
            raise FileNotFoundError(ROS_WS_DIR_ERROR_STRING.substitute(
                ros_ws=self._ros_workspace_dir))
        logger.debug('ROS workspace exists.')
        if not self._qemu_directory.exists():
            raise FileNotFoundError(
                QEMU_DIR_ERROR_STRING.substitute(qemu_dir=QEMU_DIR_NAME))
        if not os.listdir(str(self._qemu_directory.absolute())):
            raise FileNotFoundError(
                QEMU_EMPTY_ERROR_STRING.substitute(qemu_dir=QEMU_DIR_NAME))
        logger.debug('QEMU binaries exist')
        shutil.copy(
            str(self._dockerfile_directory), str(self._target_sysroot))
        if not self._expected_dockerfile_directory.exists():
            raise FileNotFoundError(COPY_DOCKER_WS_ERROR_STRING.substitute(
                dockerfile=ROS_DOCKERFILE_NAME))
        custom_script_dest = str(self._target_sysroot / 'user-custom-setup')
        if custom_script:
            shutil.copy(custom_script, custom_script_dest)
        else:
            with open(custom_script_dest, 'w') as custom_script_file:
                custom_script_file.write('#!/bin/sh\necho "No custom setup"\n')

    def build_workspace_sysroot_image(self) -> None:
        """Build the target sysroot docker image."""
        logger.info('Fetching sysroot base image: %s', self._docker_config.base_image)
        DOCKER_CLIENT.images.pull(self._docker_config.base_image)
        image_tag = self._platform.get_workspace_image_tag()
        buildargs = {
            'BASE_IMAGE': self._docker_config.base_image,
            'ROS_WORKSPACE': self._ros_workspace_relative_to_sysroot,
            'ROS_VERSION': self._platform.ros_version,
            'ROS_DISTRO': self._platform.rosdistro,
            'TARGET_TRIPLE': self._platform.cc_toolchain,
            'TARGET_ARCH': self._platform.arch,
        }
        logger.info('Building workspace image: %s', image_tag)

        # Switch to low-level API to expose build logs
        docker_client = docker.APIClient(base_url='unix://var/run/docker.sock')
        # Note the difference:
        # path – Path to the directory containing the Dockerfile
        # dockerfile – Path within the build context to the Dockerfile
        log_generator = docker_client.build(
            path=str(self._target_sysroot),
            dockerfile=str(self._expected_dockerfile_directory),
            tag=image_tag,
            buildargs=buildargs,
            quiet=False,
            nocache=self._docker_config.nocache,
            network_mode=self._docker_config.network_mode,
            decode=True)
        for chunk in log_generator:
            # There are two outputs we want to capture, stream and error.
            # We also want to remove newline (\n) and carriage returns (\r) to
            # avoid mangled output.
            error_line = chunk.get('error', None)
            if error_line:
                logger.exception(
                    'Error building sysroot image. The following error was caught:\n%s',
                    error_line)
                raise docker.errors.BuildError(reason=error_line, build_log=error_line)
            line = chunk.get('stream', '')
            line = line.rstrip().lstrip()
            if line:
                logger.info(line)

        logger.info('Successfully created sysroot docker image: %s', image_tag)

    def export_cross_compiled_build(self) -> None:
        """Done cross compiling, export the build into the ROS workspace."""
        logger.info('Exporting sysroot to path [%s]', self._target_sysroot)
        with tempfile.TemporaryFile() as install_tar_file:
            image_tag = self._platform.get_workspace_image_tag()
            logger.info(
                'Fetching cross-compiled install directory of image {} into tempfile'.format(
                    image_tag))

            sysroot_container = DOCKER_CLIENT.containers.run(
                image=image_tag, detach=True)
            install_stream, install_stat = sysroot_container.get_archive(
                '/ros_ws/install_{}'.format(self._platform.arch))
            install_tar_file.writelines(install_stream)
            sysroot_container.stop()

            logger.info('Extracting install tarball to {}'.format(self._ros_workspace_dir))
            # rewind so tarfile can read from the beginning
            install_tar_file.seek(0)
            with tarfile.open(fileobj=install_tar_file) as install_tar:
                install_tar.extractall(str(self._ros_workspace_dir))

        logger.info(
            'Successfully exported cross-compiled workspace install to path {}'.format(
                self._ros_workspace_dir / 'install'))

    def execute_cc_pipeline(self) -> bool:
        """Execute the entire cross compilation workflow."""
        self.build_workspace_sysroot_image()
        self.export_cross_compiled_build()
        return True
