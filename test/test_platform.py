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


"""Unit tests for the `create_cc_sysroot.py` script."""

import getpass

import docker
import pytest

from ros_cross_compile.sysroot_creator import Platform


@pytest.fixture
def platform_config() -> Platform:
    return Platform(
        arch='aarch64',
        os_name='ubuntu',
        ros_distro='dashing')


def test_platform_argument_validation():
    p = Platform('armhf', 'ubuntu', 'dashing')
    assert p

    with pytest.raises(ValueError):
        # invalid arch
        p = Platform('mips', 'ubuntu', 'dashing')

    with pytest.raises(ValueError):
        # invalid distro
        p = Platform('armhf', 'ubuntu', 'ardent')

    with pytest.raises(ValueError):
        # invalid OS
        p = Platform('armhf', 'rhel', 'dashing')


def test_construct_x86():
    p = Platform('x86_64', 'ubuntu', 'foxy')
    assert p


def test_sysroot_image_tag(platform_config):
    """Make sure the image tag is created correctly."""
    image_tag = platform_config.sysroot_image_tag
    test_tag = '{}/{}:latest'.format(getpass.getuser(), str(platform_config))
    assert isinstance(image_tag, str)
    assert image_tag == test_tag


def verify_base_docker_images(arch, os_name, rosdistro, image_name):
    """Assert correct base image is generated."""
    platform = Platform(arch, os_name, rosdistro)
    assert platform.target_base_image == image_name


def test_get_docker_base_image():
    """Test that the correct base docker image is used for all arguments."""
    verify_base_docker_images('aarch64', 'ubuntu', 'dashing', 'arm64v8/ubuntu:bionic')
    verify_base_docker_images('aarch64', 'ubuntu', 'melodic', 'arm64v8/ubuntu:bionic')
    verify_base_docker_images('aarch64', 'ubuntu', 'noetic', 'arm64v8/ubuntu:focal')

    verify_base_docker_images('aarch64', 'debian', 'dashing', 'arm64v8/debian:stretch')
    verify_base_docker_images('aarch64', 'debian', 'melodic', 'arm64v8/debian:stretch')
    verify_base_docker_images('aarch64', 'debian', 'noetic', 'arm64v8/debian:buster')

    verify_base_docker_images('armhf', 'ubuntu', 'dashing', 'arm32v7/ubuntu:bionic')
    verify_base_docker_images('armhf', 'ubuntu', 'melodic', 'arm32v7/ubuntu:bionic')
    verify_base_docker_images('armhf', 'ubuntu', 'noetic', 'arm32v7/ubuntu:focal')

    verify_base_docker_images('armhf', 'debian', 'dashing', 'arm32v7/debian:stretch')
    verify_base_docker_images('armhf', 'debian', 'melodic', 'arm32v7/debian:stretch')
    verify_base_docker_images('armhf', 'debian', 'noetic', 'arm32v7/debian:buster')


def test_docker_py_version():
    # Explicitly check a known difference between apt and pip versions
    with pytest.raises(TypeError):
        # 1.20 (from pip, which we are not using) API has named arguments
        err = docker.errors.BuildError(reason='problem', build_log='stuff that happened')

    # 1.10 API (from apt which we are using) does not
    err = docker.errors.BuildError('problem')
    assert err


def test_ros_version_map():
    platform = Platform('armhf', 'ubuntu', 'dashing')
    assert platform.ros_version == 'ros2'
    platform = Platform('x86_64', 'ubuntu', 'foxy')
    assert platform.ros_version == 'ros2'
    platform = Platform('armhf', 'ubuntu', 'melodic')
    assert platform.ros_version == 'ros'
    platform = Platform('aarch64', 'ubuntu', 'noetic')
    assert platform.ros_version == 'ros'


def test_override_base():
    override = 'arm128v12/ubuntu:quintessential'
    platform = Platform('aarch64', 'ubuntu', 'dashing', override)
    assert platform.target_base_image == override


def test_override_build_image():
    build_image = 'rostooling/setup-ros-docker:ubuntu-focal-ros-foxy-ros-base-latest'
    platform = Platform('aarch64', 'ubuntu', 'dashing')
    platform.override_sysroot_image_tag(build_image)
    assert platform.sysroot_image_tag == build_image
