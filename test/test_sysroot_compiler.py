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

import os
from pathlib import Path
from types import SimpleNamespace
from typing import Tuple

from cross_compile.sysroot_compiler import DOCKER_WS_NAME
from cross_compile.sysroot_compiler import DockerConfig
from cross_compile.sysroot_compiler import Platform
from cross_compile.sysroot_compiler import QEMU_DIR_NAME
from cross_compile.sysroot_compiler import SYSROOT_DIR_NAME
from cross_compile.sysroot_compiler import SysrootCompiler

import pytest


def _default_args() -> SimpleNamespace:
    # TODO: Change base image to dockerhub repo image once we have access.

    args: SimpleNamespace = SimpleNamespace()
    args.arch = 'aarch64'
    args.os = 'ubuntu'
    args.distro = 'dashing'
    args.rmw = 'fastrtps'
    args.sysroot_base_image = (
        '035662560449.dkr.ecr.us-east-2.amazonaws.com/cc-tool:'
        'aarch64-bionic-dashing-fastrtps-prebuilt')
    args.docker_network_mode = 'host'
    args.sysroot_nocache = 'False'

    return args


@pytest.fixture
def platform_config() -> Platform:
    return Platform(_default_args())


@pytest.fixture
def docker_config() -> DockerConfig:
    return DockerConfig(_default_args())


def setup_mock_sysroot(path: Path) -> Tuple[Path, Path]:
    """Create mock directories to correctly construct the SysrootCreator."""
    sysroot_dir = path / SYSROOT_DIR_NAME
    sysroot_dir.mkdir()
    ros_workspace_dir = sysroot_dir / 'ros2_ws'
    ros_workspace_dir.mkdir()
    qemu_dir = sysroot_dir / QEMU_DIR_NAME
    qemu_dir.mkdir()
    qemu_binary_mock = qemu_dir / 'qemu'
    qemu_binary_mock.touch()
    docker_ws_dir = sysroot_dir / DOCKER_WS_NAME
    docker_ws_dir.touch()
    return sysroot_dir, ros_workspace_dir


def test_get_workspace_image_tag(platform_config):
    """Make sure the image tag is created correctly."""
    image_tag = platform_config.get_workspace_image_tag()
    test_tag = '{}/{}:latest'.format(os.getenv('USER'), str(platform_config))
    assert isinstance(image_tag, str)
    assert image_tag == test_tag


def test_docker_config_args(docker_config):
    """Make sure the Docker configuration is setup correctly."""
    args = _default_args()
    test_config_string = (
        'Base Image: {}\n'
        'Network Mode: {}\n'
        'Caching: {}').format(
        args.sysroot_base_image, args.docker_network_mode,
        args.sysroot_nocache)
    config_string = str(docker_config)
    assert isinstance(config_string, str)
    assert config_string == test_config_string


def test_sysroot_compiler_constructor(
        platform_config, docker_config, tmp_path):
    """Test the SysrootCompiler constructor assuming valid path setup."""
    # Create mock directories and files
    sysroot_dir, ros_workspace_dir = setup_mock_sysroot(tmp_path)
    sysroot_compiler = SysrootCompiler(
        str(sysroot_dir), str(ros_workspace_dir), platform_config,
        docker_config)

    assert isinstance(sysroot_compiler.get_build_setup_script_path(), Path)
    assert isinstance(sysroot_compiler .get_system_setup_script_path(), Path)


def test_write_cc_build_setup_file(platform_config, docker_config, tmp_path):
    """Check if the build setup file was written to directory."""
    sysroot_dir, ros_workspace_dir = setup_mock_sysroot(tmp_path)
    sysroot_compiler = SysrootCompiler(
        str(tmp_path), str(ros_workspace_dir), platform_config, docker_config)
    setup_script_path = sysroot_compiler._write_cc_build_setup_script()

    assert isinstance(setup_script_path, Path)
    assert setup_script_path.exists() is True


def test_write_cc_system_setup_file(platform_config, docker_config, tmp_path):
    """Check if the system setup file was written to directory."""
    sysroot_dir, ros_workspace_dir = setup_mock_sysroot(tmp_path)
    sysroot_compiler = SysrootCompiler(
        str(tmp_path), str(ros_workspace_dir), platform_config, docker_config)
    setup_script_path = sysroot_compiler._write_cc_system_setup_script()

    assert isinstance(setup_script_path, Path)
    assert setup_script_path.exists() is True
