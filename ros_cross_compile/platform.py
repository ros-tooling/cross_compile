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
from typing import NamedTuple
from typing import Optional

ArchNameMapping = NamedTuple('ArchNameMapping', [('docker', str), ('qemu', str)])


# NOTE: when changing any following values, update README.md Supported Targets section
ARCHITECTURE_NAME_MAP = {
    'armhf': ArchNameMapping(docker='arm32v7', qemu='arm'),
    'aarch64': ArchNameMapping(docker='arm64v8', qemu='aarch64'),
    'x86_64': ArchNameMapping(docker='', qemu='x86_64'),
}
SUPPORTED_ARCHITECTURES = tuple(ARCHITECTURE_NAME_MAP.keys())

SUPPORTED_ROS2_DISTROS = ('dashing', 'foxy', 'galactic', 'rolling')
SUPPORTED_ROS_DISTROS = ('melodic', 'noetic')

ROSDISTRO_OS_MAP = {
    'melodic': {
        'ubuntu': 'bionic',
        'debian': 'stretch',
    },
    'noetic': {
        'ubuntu': 'focal',
        'debian': 'buster',
    },
    'dashing': {
        'ubuntu': 'bionic',
        'debian': 'stretch',
    },
    'foxy': {
        'ubuntu': 'focal',
        'debian': 'buster',
    },
    'galactic': {
        'ubuntu': 'focal',
        'debian': 'buster',
    },
    'rolling': {
        'ubuntu': 'focal',
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

        self._overridden_sysroot_image_tag = None

        try:
            docker_org = ARCHITECTURE_NAME_MAP[arch].docker
        except KeyError:
            raise ValueError('Unknown target architecture "{}" specified'.format(arch))

        if self.ros_distro in SUPPORTED_ROS2_DISTROS:
            self._ros_version = 'ros2'
        elif self.ros_distro in SUPPORTED_ROS_DISTROS:
            self._ros_version = 'ros'
        else:
            raise ValueError('Unknown ROS distribution "{}" specified'.format(ros_distro))

        os_map = ROSDISTRO_OS_MAP[self.ros_distro]
        if self.os_name not in os_map:
            raise ValueError(
                'OS "{}" not supported for ROS distro "{}". Supported values: {} '
                '(note that these are case sensitive)'.format(
                    os_name, ros_distro, list(os_map.keys())))

        if override_base_image:
            self._docker_target_base = override_base_image
        else:
            self._os_distro = ROSDISTRO_OS_MAP[self.ros_distro][self.os_name]
            native_base = '{}:{}'.format(self.os_name, self.os_distro)
            if docker_org:
                self._docker_target_base = '{}/{}'.format(docker_org, native_base)
            else:
                self._docker_target_base = native_base

    @property
    def arch(self):
        return self._arch

    @property
    def qemu_arch(self):
        return ARCHITECTURE_NAME_MAP[self.arch].qemu

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
        if (self._overridden_sysroot_image_tag):
            return self._overridden_sysroot_image_tag
        return getpass.getuser() + '/' + str(self) + ':latest'

    def override_sysroot_image_tag(self, tag):
        """Override the sysroot_image_tag with tag."""
        self._overridden_sysroot_image_tag = tag

    @property
    def target_base_image(self) -> str:
        """Name of the base OS Docker image for the target architecture."""
        return self._docker_target_base
