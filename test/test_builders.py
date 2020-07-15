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

from ros_cross_compile.builders import DockerBuildStage
from ros_cross_compile.builders import run_emulated_docker_build
from ros_cross_compile.pipeline_stages import PipelineStageConfigOptions
from ros_cross_compile.platform import Platform


def test_emulated_docker_build():
    # Very simple smoke test to validate that all internal syntax is correct
    mock_docker_client = Mock()
    mock_data_collector = Mock()
    platform = Platform('aarch64', 'ubuntu', 'eloquent')

    # a default set of customizations for the docker build stage
    customizations = PipelineStageConfigOptions(False, [], None, None, None)
    temp_stage = DockerBuildStage()

    temp_stage(platform, mock_docker_client,
               Path('dummy_path'), customizations, mock_data_collector)

    assert mock_docker_client.run_container.call_count == 1


def test_docker_build_stage_creation():
    temp_stage = DockerBuildStage()
    assert temp_stage


def test_docker_build_stage_name():
    temp_stage = DockerBuildStage()
    assert temp_stage._name == run_emulated_docker_build.__name__
