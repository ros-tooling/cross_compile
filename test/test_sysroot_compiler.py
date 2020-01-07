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


"""Unit tests for the `create_cc_sysroot.py` script."""

import getpass
import os
from pathlib import Path
from typing import Tuple

from cross_compile.sysroot_compiler import DockerConfig
from cross_compile.sysroot_compiler import Platform
from cross_compile.sysroot_compiler import QEMU_DIR_NAME
from cross_compile.sysroot_compiler import ROS_DOCKERFILE_NAME
from cross_compile.sysroot_compiler import SYSROOT_DIR_NAME
from cross_compile.sysroot_compiler import SysrootCompiler
import docker
import pytest

THIS_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def _default_docker_kwargs() -> dict:
    return {
        'arch': 'aarch64',
        'os': 'ubuntu',
        'rosdistro': 'dashing',
        'sysroot_base_image': '035662560449.dkr.ecr.us-east-2.amazonaws.com/cc-tool:'
                              'aarch64-bionic-dashing-fastrtps-prebuilt',
        'docker_network_mode': 'host',
        'sysroot_nocache': False,
    }


@pytest.fixture
def platform_config() -> Platform:
    return Platform(
        arch='aarch64',
        os='ubuntu',
        rosdistro='dashing',
        rmw='fastrtps')


@pytest.fixture
def docker_config() -> DockerConfig:
    return DockerConfig(**_default_docker_kwargs())


def setup_mock_sysroot(path: Path) -> Tuple[Path, Path]:
    """Create mock directories to correctly construct the SysrootCreator."""
    sysroot_dir = path / SYSROOT_DIR_NAME
    sysroot_dir.mkdir()
    ros_workspace_dir = sysroot_dir / 'ros_ws'
    ros_workspace_dir.mkdir()
    qemu_dir = sysroot_dir / QEMU_DIR_NAME
    qemu_dir.mkdir()
    qemu_binary_mock = qemu_dir / 'qemu'
    qemu_binary_mock.ensure()
    return sysroot_dir, ros_workspace_dir


def test_get_workspace_image_tag(platform_config):
    """Make sure the image tag is created correctly."""
    image_tag = platform_config.get_workspace_image_tag()
    test_tag = '{}/{}:latest'.format(getpass.getuser(), str(platform_config))
    assert isinstance(image_tag, str)
    assert image_tag == test_tag


def test_docker_config_args(docker_config):
    """Make sure the Docker configuration is setup correctly."""
    args = _default_docker_kwargs()
    test_config_string = (
        'Base Image: {}\n'
        'Network Mode: {}\n'
        'Caching: {}'
    ).format(
        args['sysroot_base_image'], args['docker_network_mode'], args['sysroot_nocache']
    )
    config_string = str(docker_config)
    assert isinstance(config_string, str)
    assert config_string == test_config_string


def test_sysroot_compiler_constructor(
        platform_config, docker_config, tmpdir):
    """Test the SysrootCompiler constructor assuming valid path setup."""
    # Create mock directories and files
    sysroot_dir, ros_workspace_dir = setup_mock_sysroot(tmpdir)
    sysroot_compiler = SysrootCompiler(
        str(tmpdir), 'ros_ws', platform_config, docker_config)

    assert isinstance(sysroot_compiler.get_build_setup_script_path(), Path)
    assert isinstance(sysroot_compiler.get_system_setup_script_path(), Path)


def test_custom_setup_script(platform_config, docker_config, tmpdir):
    sysroot_dir, ros_workspace_dir = setup_mock_sysroot(tmpdir)
    compiler = SysrootCompiler(
        str(tmpdir), 'ros_ws', platform_config, docker_config,
        custom_setup_script_path=os.path.join(THIS_SCRIPT_DIR, 'custom-setup.sh'))
    assert compiler
    assert (sysroot_dir / 'user-custom-setup').exists()


def test_custom_data_dir(platform_config, docker_config, tmpdir):
    sysroot_dir, ros_workspace_dir = setup_mock_sysroot(tmpdir)
    compiler = SysrootCompiler(
        str(tmpdir), 'ros_ws', platform_config, docker_config,
        custom_data_dir=os.path.join(THIS_SCRIPT_DIR, 'data'))
    assert compiler
    assert (sysroot_dir / 'user-custom-data' / 'arbitrary.txt').exists()


def test_sysroot_compiler_tree_validation(platform_config, docker_config, tmpdir):
    """
    Ensure that the SysrootCompiler constructor validates the workspace.

    Start with empty directory and add one piece at a time, expecting failures until
    all parts are present.
    """
    kwargs = {
        'cc_root_dir': str(tmpdir),
        'ros_workspace_dir': 'ros_ws',
        'platform': platform_config,
        'docker_config': docker_config,
        'custom_setup_script_path': None,
    }

    # There's no 'sysroot' at all yet
    with pytest.raises(FileNotFoundError):
        compiler = SysrootCompiler(**kwargs)

    sysroot_dir = tmpdir / SYSROOT_DIR_NAME
    sysroot_dir.mkdir()
    # ROS2 ws and qemu dirs are missing
    with pytest.raises(FileNotFoundError):
        compiler = SysrootCompiler(**kwargs)

    ros_workspace_dir = sysroot_dir / 'ros_ws'
    ros_workspace_dir.mkdir()
    # qemu dirs are missing
    with pytest.raises(FileNotFoundError):
        compiler = SysrootCompiler(**kwargs)

    qemu_dir = sysroot_dir / QEMU_DIR_NAME
    qemu_dir.mkdir()
    # the qemu binary is still missing
    with pytest.raises(FileNotFoundError):
        compiler = SysrootCompiler(**kwargs)

    qemu_binary_mock = qemu_dir / 'qemu'
    qemu_binary_mock.ensure()
    # everything is present now
    compiler = SysrootCompiler(**kwargs)
    assert compiler


def test_sysroot_compiler_tree_additions(platform_config, docker_config, tmpdir):
    sysroot_dir, ros_workspace_dir = setup_mock_sysroot(tmpdir)
    compiler = SysrootCompiler(str(tmpdir), 'ros_ws', platform_config, docker_config)
    assert compiler
    assert (sysroot_dir / ROS_DOCKERFILE_NAME).exists()
    assert (sysroot_dir / 'mixins' / 'cross-compile.mixin').exists()
    assert (sysroot_dir / 'mixins' / 'index.yaml').exists()


def verify_base_docker_images(arch, os, rosdistro, image_name):
    """Assert correct base image is generated."""
    sysroot_base_image = None
    docker_network_mode = 'host'
    sysroot_nocache = 'False'
    assert DockerConfig(
        arch, os, rosdistro, sysroot_base_image,
        docker_network_mode, sysroot_nocache).base_image == image_name


def test_get_docker_base_image():
    """Test that the correct base docker image is used for all arguments."""
    verify_base_docker_images('aarch64', 'ubuntu', 'dashing', 'arm64v8/ubuntu:bionic')
    verify_base_docker_images('aarch64', 'ubuntu', 'eloquent', 'arm64v8/ubuntu:bionic')
    verify_base_docker_images('aarch64', 'ubuntu', 'kinetic', 'arm64v8/ubuntu:xenial')
    verify_base_docker_images('aarch64', 'ubuntu', 'melodic', 'arm64v8/ubuntu:bionic')

    verify_base_docker_images('aarch64', 'debian', 'dashing', 'arm64v8/debian:stretch')
    verify_base_docker_images('aarch64', 'debian', 'eloquent', 'arm64v8/debian:buster')
    verify_base_docker_images('aarch64', 'debian', 'kinetic', 'arm64v8/debian:jessie')
    verify_base_docker_images('aarch64', 'debian', 'melodic', 'arm64v8/debian:stretch')

    verify_base_docker_images('armhf', 'ubuntu', 'dashing', 'arm32v7/ubuntu:bionic')
    verify_base_docker_images('armhf', 'ubuntu', 'eloquent', 'arm32v7/ubuntu:bionic')
    verify_base_docker_images('armhf', 'ubuntu', 'kinetic', 'arm32v7/ubuntu:xenial')
    verify_base_docker_images('armhf', 'ubuntu', 'melodic', 'arm32v7/ubuntu:bionic')

    verify_base_docker_images('armhf', 'debian', 'dashing', 'arm32v7/debian:stretch')
    verify_base_docker_images('armhf', 'debian', 'eloquent', 'arm32v7/debian:buster')
    verify_base_docker_images('armhf', 'debian', 'kinetic', 'arm32v7/debian:jessie')
    verify_base_docker_images('armhf', 'debian', 'melodic', 'arm32v7/debian:stretch')


def test_parse_docker_build_output(
        platform_config, docker_config, tmpdir):
    """Test the SysrootCompiler constructor assuming valid path setup."""
    # Create mock directories and files
    sysroot_dir, ros_workspace_dir = setup_mock_sysroot(tmpdir)
    sysroot_compiler = SysrootCompiler(
        str(tmpdir), 'ros_ws', platform_config, docker_config, None)

    log_generator_without_errors = [
        {'stream': ' ---\\u003e a9eb17255234\\n'},
        {'stream': 'Step 1 : VOLUME /data\\n'},
        {'stream': ' ---\\u003e Running in abdc1e6896c6\\n'},
        {'stream': ' ---\\u003e 713bca62012e\\n'},
        {'stream': 'Removing intermediate container abdc1e6896c6\\n'},
        {'stream': 'Step 2 : CMD [\\"/bin/sh\\"]\\n'},
        {'stream': ' ---\\u003e Running in dba30f2a1a7e\\n'},
        {'stream': ' ---\\u003e 032b8b2855fc\\n'},
        {'stream': 'Removing intermediate container dba30f2a1a7e\\n'},
        {'stream': 'Successfully built 032b8b2855fc\\n'},
    ]
    # Just expect it not to raise
    sysroot_compiler._parse_build_output(log_generator_without_errors)

    log_generator_with_errors = [
        {'stream': ' ---\\u003e a9eb17255234\\n'},
        {'stream': 'Step 1 : VOLUME /data\\n'},
        {'stream': ' ---\\u003e Running in abdc1e6896c6\\n'},
        {'stream': ' ---\\u003e 713bca62012e\\n'},
        {'stream': 'Removing intermediate container abdc1e6896c6\\n'},
        {'stream': 'Step 2 : CMD [\\"/bin/sh\\"]\\n'},
        {'error': ' ---\\COMMAND NOT FOUND\\n'},
    ]
    with pytest.raises(docker.errors.BuildError):
        sysroot_compiler._parse_build_output(log_generator_with_errors)


def test_docker_py_version():
    # Explicitly check a known difference between apt and pip versions
    with pytest.raises(TypeError):
        # 1.20 (from pip, which we are not using) API has named arguments
        err = docker.errors.BuildError(reason='problem', build_log='stuff that happened')

    # 1.10 API (from apt which we are using) does not
    err = docker.errors.BuildError('problem')
    assert err
