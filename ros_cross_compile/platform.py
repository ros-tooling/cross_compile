# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
import getpass
from typing import Optional

# NOTE: when changing any following values, update README.md Supported Targets section
ARCHITECTURE_DOCKER_MAP = {
    'armhf': 'arm32v7',
    'aarch64': 'arm64v8',
}
SUPPORTED_ARCHITECTURES = tuple(ARCHITECTURE_DOCKER_MAP.keys())

SUPPORTED_ROS2_DISTROS = ('dashing', 'eloquent')
SUPPORTED_ROS_DISTROS = ('kinetic', 'melodic')

ROSDISTRO_OS_MAP = {
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
    },
}
# NOTE: when changing any preceding values, update README.md Supported Targets section


class Platform:
    """
    Represents the target platform for cross compiling.

    Includes:
    * Target architecture
    * Target operating system
    * Target ROS distribution
    """

    def __init__(
        self, arch: str, os_name: str, ros_distro: str, override_base_image: Optional[str] = None,
    ):
        """Initialize platform parameters."""
        self._arch = arch
        self._ros_distro = ros_distro
        self._os_name = os_name

        try:
            docker_org = ARCHITECTURE_DOCKER_MAP[arch]
        except KeyError:
            raise ValueError('Unknown target architecture "{}" specified'.format(arch))

        if self.ros_distro in SUPPORTED_ROS2_DISTROS:
            self._ros_version = 'ros2'
        elif self.ros_distro in SUPPORTED_ROS_DISTROS:
            self._ros_version = 'ros'
        else:
            raise ValueError('Unknown ROS distribution "{}" specified'.format(ros_distro))

        if self.os_name not in ROSDISTRO_OS_MAP[self.ros_distro]:
            raise ValueError(
                'OS "{}" not supported for ROS distro "{}"'.format(os_name, ros_distro))

        if override_base_image:
            self._docker_target_base = override_base_image
        else:
            self._os_distro = ROSDISTRO_OS_MAP[self.ros_distro][self.os_name]
            self._docker_target_base = '{}/{}:{}'.format(docker_org, self.os_name, self.os_distro)
            self._docker_native_base = '{}:{}'.format(self.os_name, self.os_distro)

    @property
    def arch(self):
        return self._arch

    @property
    def ros_distro(self):
        return self._ros_distro

    @property
    def os_name(self):
        return self._os_name

    @property
    def os_distro(self):
        return self._os_distro

    @property
    def ros_version(self):
        return self._ros_version

    def __str__(self):
        """Return string representation of platform parameters."""
        return '-'.join((self.arch, self.os_name, self.ros_distro))

    @property
    def sysroot_image_tag(self) -> str:
        """Generate docker image name and tag."""
        return getpass.getuser() + '/' + str(self) + ':latest'

    @property
    def target_base_image(self) -> str:
        """Name of the base OS Docker image for the target architecture."""
        return self._docker_target_base

    @property
    def native_base_image(self) -> str:
        """Name of the base OS Docker image for the host architecture."""
        return self._docker_native_base
