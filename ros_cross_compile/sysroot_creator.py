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

import logging
import os
from pathlib import Path
import shutil
from string import Template
from typing import Optional

from ros_cross_compile.docker_client import DockerClient
from ros_cross_compile.platform import Platform

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

COPY_WS_FILE_ERROR_STRING = Template(
    """Unable to copy the "$filename" file. Make sure you """
    """have write permissions to the sysroot directory."""
)

SYSROOT_NOT_FOUND_ERROR_STRING = Template(
    """Sysroot directory not found at "$sysroot_dir". Make """
    """sure you specify the full path to the directory containing "sysroot"."""
)

SYSROOT_DIR_NAME = 'sysroot'  # type: str
QEMU_DIR_NAME = 'qemu-user-static'  # type: str
ROS_DOCKERFILE_NAME = 'sysroot.Dockerfile'  # type: str
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def _replace_tree(src: Path, dest: Path) -> None:
    """Delete dest and copy the directory src to that location."""
    shutil.rmtree(str(dest), ignore_errors=True)  # it may or may not exist already
    shutil.copytree(str(src), str(dest))


def _ensure_exists(src: Path) -> None:
    """Check that a path exists and raise a FileNotFoundError if not."""
    if not src.exists():
        raise FileNotFoundError(COPY_WS_FILE_ERROR_STRING.substitute(filename=src))


class SysrootCreator:
    """
    Create a sysroot with all dependencies for a workspace.

    Uses Docker to install dependencies on an emulated target, which produces
    a Docker image with everything installed.
    """

    def __init__(
      self,
      cc_root_dir: str,
      ros_workspace_dir: str,
      platform: Platform,
      custom_setup_script_path: Optional[str] = None,
      custom_data_dir: Optional[str] = None,
    ) -> None:
        """
        Construct a SysrootCreator object building ROS 2 Docker container.

        :param cc_root_dir: The directory containing the 'sysroot' directory
                            with the ROS2 workspace and QEMU binaries.
        :param ros_workspace_dir: The name of the directory containing the
                                  ROS2 packages (inside a 'src' directory).
        :param platform: A custom object used to specify the the platform for
                         cross-compilation.
        :param custom_setup_script_path: Optional path to a custom setup script
                                         to run arbitrary commands
        :param custom_data_dir: Optional path to a custom directory of data that
                                `custom_setup_script` can utilize
        """
        if not isinstance(cc_root_dir, str):
            raise TypeError('Argument `cc_root_dir` must be of type string.')
        if not isinstance(ros_workspace_dir, str):
            raise TypeError(
                'Argument `ros_workspace_dir` must be of type string.')
        if not isinstance(platform, Platform):
            raise TypeError('Argument `platform` must be of type Platform.')

        workspace_root = Path(cc_root_dir).resolve()
        self._target_sysroot = workspace_root / SYSROOT_DIR_NAME
        self._ros_workspace_relative_to_sysroot = ros_workspace_dir
        self._ros_workspace_dir = self._target_sysroot / self._ros_workspace_relative_to_sysroot
        self._qemu_directory = self._target_sysroot / QEMU_DIR_NAME
        self._final_dockerfile_path = (
            self._target_sysroot / ROS_DOCKERFILE_NAME)
        self._system_setup_script_path = Path()
        self._build_setup_script_path = Path()
        self._platform = platform
        self._setup_sysroot_dir(custom_setup_script_path, custom_data_dir)

    def get_system_setup_script_path(self) -> Path:
        """Return the path to the system setup script."""
        return self._system_setup_script_path

    def get_build_setup_script_path(self) -> Path:
        """Return the path to the build setup script."""
        return self._build_setup_script_path

    def _setup_sysroot_dir(self, custom_script: Optional[str], custom_data: Optional[str]) -> None:
        """
        Check to make sure the sysroot directory is setup correctly.

        Raises FileNotFoundError if any of the components necessary for
        cross compilation are missing. Copies the Dockerfile to the
        'sysroot' directory in order to copy the assets to it.
        See https://docs.docker.com/engine/reference/builder/#copy
        """
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

        package_path = Path(__file__).parent
        dockerfile_src = str(package_path / 'docker' / ROS_DOCKERFILE_NAME)
        shutil.copy(dockerfile_src, str(self._target_sysroot))
        _ensure_exists(self._final_dockerfile_path)
        logger.debug('Copied Dockerfile')

        build_script_src = str(package_path / 'docker' / 'build_workspace.sh')
        shutil.copy(build_script_src, str(self._target_sysroot))
        _ensure_exists(self._target_sysroot / 'build_workspace.sh')
        logger.debug('Copied builder script')

        custom_data_dest = str(self._target_sysroot / 'user-custom-data')
        shutil.rmtree(
            custom_data_dest, ignore_errors=True
        )  # should not be there unless explicitly specified
        if custom_data:
            shutil.copytree(custom_data, custom_data_dest)
            logger.debug('Custom data dir provided - copied')
        else:
            os.makedirs(custom_data_dest)
            logger.debug('No custom data dir provided - touched empty dir')

        mixins_src = package_path / 'mixins'
        mixins_dest = self._target_sysroot / 'mixins'
        _replace_tree(mixins_src, mixins_dest)
        if not mixins_dest.exists():
            raise FileNotFoundError('Mixins not properly copied to build context')
        logger.debug('Copied mixins')

        custom_script_dest = str(self._target_sysroot / 'user-custom-setup')
        if custom_script:
            shutil.copy(custom_script, custom_script_dest)
            logger.debug('Custom script provided - copied')
        else:
            with open(custom_script_dest, 'w') as custom_script_file:
                custom_script_file.write('#!/bin/sh\necho "No custom setup"\n')
            logger.debug('No custom script provided - created empty script')

    def create_workspace_sysroot_image(self, docker_client: DockerClient) -> None:
        """Build the target sysroot docker image."""
        image_tag = self._platform.sysroot_image_tag

        logger.info('Building sysroot image: %s', image_tag)
        docker_client.build_image(
            dockerfile_dir=self._target_sysroot,
            dockerfile_name=ROS_DOCKERFILE_NAME,
            tag=image_tag,
            buildargs={
                'BASE_IMAGE': self._platform.target_base_image,
                'ROS_WORKSPACE': self._ros_workspace_relative_to_sysroot,
                'ROS_VERSION': self._platform.ros_version,
                'ROS_DISTRO': self._platform.ros_distro,
                'TARGET_ARCH': self._platform.arch,
            }
        )
        logger.info('Successfully created sysroot docker image: %s', image_tag)
