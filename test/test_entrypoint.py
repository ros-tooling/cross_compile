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

import contextlib
import os
from pathlib import Path
from unittest.mock import Mock
from unittest.mock import patch

import pytest

# from ros_cross_compile.docker_client import DockerClient
from ros_cross_compile.data_collector import DataCollector
from ros_cross_compile.dependencies import assert_install_rosdep_script_exists
from ros_cross_compile.dependencies import rosdep_install_script
from ros_cross_compile.platform import Platform
from ros_cross_compile.ros_cross_compile import cross_compile_pipeline
from ros_cross_compile.ros_cross_compile import parse_args


@contextlib.contextmanager
def chdir(dirname: str):
    """Provide a "with" statement for changing the working directory."""
    curdir = os.getcwd()
    try:
        os.chdir(dirname)
        yield
    finally:
        os.chdir(curdir)


def test_trivial_argparse():
    args = parse_args(['somepath', '-a', 'aarch64', '-o', 'ubuntu'])
    assert args


def test_bad_workspace(tmpdir):
    args = parse_args([str(tmpdir), '-a', 'aarch64', '-o', 'ubuntu', '-d', 'foxy'])
    test_collector = DataCollector()
    platform = Platform(args.arch, args.os, args.rosdistro)
    with pytest.raises(ValueError):
        cross_compile_pipeline(args, test_collector, platform)


def test_relative_workspace(tmpdir):
    # Change directory to the tmp dir and invoke using '.' as the
    # workspace to check if relative paths work
    tmp = Path(str(tmpdir))
    test_collector = DataCollector()
    (tmp / 'src').mkdir()
    relative_dir = '.'
    args = parse_args([relative_dir, '-a', 'aarch64', '-o', 'ubuntu', '-d', 'foxy'])
    platform = Platform(args.arch, args.os, args.rosdistro)
    with chdir(str(tmp)), patch(
        'ros_cross_compile.ros_cross_compile.DockerClient', Mock()
    ), patch(
        'ros_cross_compile.dependencies.assert_install_rosdep_script_exists'
    ):
        # should not raise an exception
        cross_compile_pipeline(args, test_collector, platform)


def test_mocked_cc_pipeline(tmpdir):
    tmp = Path(str(tmpdir))
    test_collector = DataCollector()
    (tmp / 'src').mkdir()
    args = parse_args([str(tmpdir), '-a', 'aarch64', '-o', 'ubuntu'])
    platform = Platform(args.arch, args.os, args.rosdistro)
    with patch(
        'ros_cross_compile.ros_cross_compile.DockerClient', Mock()
    ) as docker_mock, patch(
        'ros_cross_compile.dependencies.assert_install_rosdep_script_exists'
    ) as script_mock:
        cross_compile_pipeline(args, test_collector, platform)
        assert script_mock.called
        assert docker_mock.called
        assert docker_mock().build_image.call_count == 2
        assert docker_mock().run_container.call_count == 2


def test_install_rosdep_script_exist(tmpdir):
    ws = Path(str(tmpdir))
    platform = Platform('aarch64', 'ubuntu', 'foxy')
    data_file = ws / rosdep_install_script(platform)
    data_file.parent.mkdir(parents=True)
    data_file.touch()
    check_script = assert_install_rosdep_script_exists(ws, platform)
    assert check_script


def test_install_rosdep_script_doesnot_exist(tmpdir):
    ws = Path(str(tmpdir))
    platform = Platform('aarch64', 'ubuntu', 'foxy')
    data_file = ws / rosdep_install_script(platform)
    data_file.parent.mkdir(parents=True)
    with pytest.raises(RuntimeError):
        assert_install_rosdep_script_exists(ws, platform)
