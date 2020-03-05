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

from unittest.mock import Mock
from unittest.mock import patch

# from ros_cross_compile.docker_client import DockerClient
from ros_cross_compile.ros_cross_compile import cross_compile_pipeline
from ros_cross_compile.ros_cross_compile import parse_args


def test_trivial_argparse():
    args = parse_args(['somepath', '-a', 'aarch64', '-o', 'ubuntu'])
    assert args


def test_mocked_cc_pipeline(tmpdir):
    args = parse_args([str(tmpdir), '-a', 'aarch64', '-o', 'ubuntu'])
    with patch('ros_cross_compile.ros_cross_compile.DockerClient', Mock()) as docker_mock:
        cross_compile_pipeline(args)
        assert docker_mock.called
        assert docker_mock().build_image.call_count == 2
        assert docker_mock().run_container.call_count == 2
