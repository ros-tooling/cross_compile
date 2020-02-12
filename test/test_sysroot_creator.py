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
from typing import Tuple
from unittest.mock import Mock

import pytest
from ros_cross_compile.sysroot_creator import Platform
from ros_cross_compile.sysroot_creator import QEMU_DIR_NAME
from ros_cross_compile.sysroot_creator import ROS_DOCKERFILE_NAME
from ros_cross_compile.sysroot_creator import SYSROOT_DIR_NAME
from ros_cross_compile.sysroot_creator import SysrootCreator

THIS_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


@pytest.fixture
def platform_config() -> Platform:
    return Platform(
        arch='aarch64',
        os_name='ubuntu',
        ros_distro='dashing')


def _default_docker_kwargs() -> dict:
    return {
        'platform': Platform('aarch64', 'ubuntu', 'dashing'),
        'override_base_image': 'arm64v8/gcc:9.2.0',
        'sysroot_nocache': False,
    }


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


def test_sysroot_creator_constructor(
        platform_config, tmpdir):
    """Test the SysrootCreator constructor assuming valid path setup."""
    # Create mock directories and files
    sysroot_dir, ros_workspace_dir = setup_mock_sysroot(tmpdir)
    sysroot_creator = SysrootCreator(str(tmpdir), 'ros_ws', platform_config)

    assert isinstance(sysroot_creator.get_build_setup_script_path(), Path)
    assert isinstance(sysroot_creator.get_system_setup_script_path(), Path)


def test_sysroot_creator_constructor_argument_validation(platform_config, tmpdir):
    """Make sure SysrootCreator constructor validates input types properly."""
    sysroot_dir, ros_worspace_dir = setup_mock_sysroot(tmpdir)
    root = str(tmpdir)
    ws = 'ros_ws'
    with pytest.raises(TypeError):
        creator = SysrootCreator(tmpdir, ws, platform_config)
    with pytest.raises(TypeError):
        creator = SysrootCreator(root, Path(ws), platform_config)
    with pytest.raises(TypeError):
        creator = SysrootCreator(root, ws, 'not a Platform')

    creator = SysrootCreator(root, ws, platform_config)
    assert creator


def test_custom_setup_script(platform_config, tmpdir):
    sysroot_dir, ros_workspace_dir = setup_mock_sysroot(tmpdir)
    compiler = SysrootCreator(
        str(tmpdir), 'ros_ws', platform_config,
        custom_setup_script_path=os.path.join(THIS_SCRIPT_DIR, 'custom-setup.sh'))
    assert compiler
    assert (sysroot_dir / 'user-custom-setup').exists()


def test_custom_data_dir(platform_config, tmpdir):
    sysroot_dir, ros_workspace_dir = setup_mock_sysroot(tmpdir)
    compiler = SysrootCreator(
        str(tmpdir), 'ros_ws', platform_config,
        custom_data_dir=os.path.join(THIS_SCRIPT_DIR, 'data'))
    assert compiler
    assert (sysroot_dir / 'user-custom-data' / 'arbitrary.txt').exists()


def test_sysroot_creator_tree_validation(platform_config, tmpdir):
    """
    Ensure that the SysrootCreator constructor validates the workspace.

    Start with empty directory and add one piece at a time, expecting failures until
    all parts are present.
    """
    kwargs = {
        'cc_root_dir': str(tmpdir),
        'ros_workspace_dir': 'ros_ws',
        'platform': platform_config,
        'custom_setup_script_path': None,
    }

    # There's no 'sysroot' at all yet
    with pytest.raises(FileNotFoundError):
        compiler = SysrootCreator(**kwargs)

    sysroot_dir = tmpdir / SYSROOT_DIR_NAME
    sysroot_dir.mkdir()
    # ROS2 ws and qemu dirs are missing
    with pytest.raises(FileNotFoundError):
        compiler = SysrootCreator(**kwargs)

    ros_workspace_dir = sysroot_dir / 'ros_ws'
    ros_workspace_dir.mkdir()
    # qemu dirs are missing
    with pytest.raises(FileNotFoundError):
        compiler = SysrootCreator(**kwargs)

    qemu_dir = sysroot_dir / QEMU_DIR_NAME
    qemu_dir.mkdir()
    # the qemu binary is still missing
    with pytest.raises(FileNotFoundError):
        compiler = SysrootCreator(**kwargs)

    qemu_binary_mock = qemu_dir / 'qemu'
    qemu_binary_mock.ensure()
    # everything is present now
    compiler = SysrootCreator(**kwargs)
    assert compiler


def test_sysroot_creator_tree_additions(platform_config, tmpdir):
    sysroot_dir, ros_workspace_dir = setup_mock_sysroot(tmpdir)
    compiler = SysrootCreator(str(tmpdir), 'ros_ws', platform_config)
    assert compiler
    assert (sysroot_dir / ROS_DOCKERFILE_NAME).exists()
    assert (sysroot_dir / 'mixins' / 'cross-compile.mixin').exists()
    assert (sysroot_dir / 'mixins' / 'index.yaml').exists()


def test_basic_sysroot_creation(platform_config, tmpdir):
    """Very simple smoke test to validate that syntax is correct."""
    # Very simple smoke test to validate that all internal syntax is correct
    mock_docker_client = Mock()
    sysroot_dir, ros_workspace_dir = setup_mock_sysroot(tmpdir)
    creator = SysrootCreator(str(tmpdir), 'ros_ws', platform_config)
    creator.create_workspace_sysroot_image(mock_docker_client)
    assert mock_docker_client.build_image.call_count == 1
