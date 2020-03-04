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
from unittest.mock import Mock

from ros_cross_compile.platform import Platform
from ros_cross_compile.sysroot_creator import create_workspace_sysroot_image
from ros_cross_compile.sysroot_creator import prepare_docker_build_environment

THIS_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def test_prepare_docker_build_basic(tmpdir):
    platform = Platform('armhf', 'debian', 'kinetic')
    tmp = Path(str(tmpdir))
    out_dir = prepare_docker_build_environment(platform, tmp, None, None)

    if platform.system() != 'Darwin':
        assert (out_dir / 'bin' / 'qemu-arm-static').exists()
    assert (out_dir / 'rosdep.Dockerfile').exists()
    assert (out_dir / 'sysroot.Dockerfile').exists()


def test_prepare_docker_build_with_user_custom(tmpdir):
    platform = Platform('aarch64', 'ubuntu', 'eloquent')
    tmp = Path(str(tmpdir))
    this_dir = Path(__file__).parent
    out_dir = prepare_docker_build_environment(
        platform, tmp,
        custom_data_dir=this_dir / 'data',
        custom_setup_script=this_dir / 'user-custom-setup',
    )

    assert (out_dir / 'bin' / 'qemu-aarch64-static').exists()
    assert (out_dir / 'rosdep.Dockerfile').exists()
    assert (out_dir / 'sysroot.Dockerfile').exists()
    assert (out_dir / 'custom-data' / 'arbitrary.txt')
    assert (out_dir / 'user-custom-setup')


def test_basic_sysroot_creation(tmpdir):
    """Very simple smoke test to validate that syntax is correct."""
    # Very simple smoke test to validate that all internal syntax is correct

    mock_docker_client = Mock()
    platform = Platform('aarch64', 'ubuntu', 'eloquent')
    create_workspace_sysroot_image(mock_docker_client, platform)
    assert mock_docker_client.build_image.call_count == 1
