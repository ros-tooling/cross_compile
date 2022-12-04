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
from platform import system
from unittest.mock import Mock
from unittest.mock import patch

import pytest

from ros_cross_compile.platform import Platform
from ros_cross_compile.sysroot_creator import CreateSysrootStage
from ros_cross_compile.sysroot_creator import prepare_docker_build_environment
from ros_cross_compile.sysroot_creator import setup_emulator

from .utilities import default_pipeline_options

THIS_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


@patch('ros_cross_compile.sysroot_creator.py_platform.system', side_effect=lambda: 'Linux')
def test_emulator_not_installed(system_mock, tmpdir):
    with pytest.raises(RuntimeError):
        setup_emulator('not-an-arch', Path(str(tmpdir)))


@patch('ros_cross_compile.sysroot_creator.py_platform.system', side_effect=lambda: 'Darwin')
def test_emulator_touch(system_mock, tmpdir):
    setup_emulator('aarch64', Path(str(tmpdir)))


def test_prepare_docker_build_basic(tmpdir):
    platform = Platform('armhf', 'debian', 'melodic')
    tmp = Path(str(tmpdir))
    out_dir = prepare_docker_build_environment(platform, tmp, None, None)

    if system() != 'Darwin':
        assert (out_dir / 'bin' / 'qemu-arm-static').exists()
    assert (out_dir / 'rosdep.Dockerfile').exists()
    assert (out_dir / 'sysroot.Dockerfile').exists()


def test_run_twice(tmpdir):
    # The test is that this doesn't throw an exception for already existing paths
    platform = Platform('armhf', 'debian', 'noetic')
    tmp = Path(str(tmpdir))
    prepare_docker_build_environment(platform, tmp, None, None)
    prepare_docker_build_environment(platform, tmp, None, None)


def test_prepare_docker_build_with_user_custom(tmpdir):
    platform = Platform('aarch64', 'ubuntu', 'foxy')
    tmp = Path(str(tmpdir))
    this_dir = Path(__file__).parent
    out_dir = prepare_docker_build_environment(
        platform, tmp,
        custom_data_dir=this_dir / 'data',
        custom_setup_script=this_dir / 'user-custom-setup',
    )

    assert (out_dir / 'bin' / 'qemu-aarch64-static').exists()
    assert (out_dir / 'rosdep_focal.Dockerfile').exists()
    assert (out_dir / 'sysroot.Dockerfile').exists()
    assert (out_dir / 'custom-data' / 'arbitrary.txt')
    assert (out_dir / 'user-custom-setup')


def test_basic_sysroot_creation(tmpdir):
    """Very simple smoke test to validate that syntax is correct."""
    # Very simple smoke test to validate that all internal syntax is correct

    mock_docker_client = Mock()
    mock_data_collector = Mock()
    platform = Platform('aarch64', 'ubuntu', 'foxy')

    stage = CreateSysrootStage()
    stage(
        platform,
        mock_docker_client,
        Path('dummy_path'),
        default_pipeline_options(),
        mock_data_collector)
    assert mock_docker_client.build_image.call_count == 1


def test_create_sysroot_stage_creation():
    temp_stage = CreateSysrootStage()
    assert temp_stage
