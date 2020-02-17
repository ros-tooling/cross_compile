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
from pathlib import Path
from unittest.mock import Mock

from ros_cross_compile.platform import Platform
from ros_cross_compile.ros_cross_compile import cross_compile_pipeline
from ros_cross_compile.ros_cross_compile import parse_args
from ros_cross_compile.sysroot_creator import SysrootCreator

from .test_sysroot_creator import setup_mock_sysroot


def test_trivial():
    args = parse_args(['-a', 'aarch64', '-o', 'ubuntu'])
    assert args


def test_pipeline_smoke(tmpdir):
    """Simple test to make sure the cross_compile_pipeline has proper syntax."""
    platform = Platform(arch='aarch64', os_name='ubuntu', ros_distro='dashing')
    mock_docker_client = Mock()
    setup_mock_sysroot(tmpdir)
    sysroot_creator = SysrootCreator(str(tmpdir), 'ros_ws', platform)
    cross_compile_pipeline(mock_docker_client, platform, sysroot_creator, Path('dummy_path'))

    # One build and run each for rosdep and sysroot
    assert mock_docker_client.build_image.call_count == 1
    assert mock_docker_client.run_container.call_count == 1
