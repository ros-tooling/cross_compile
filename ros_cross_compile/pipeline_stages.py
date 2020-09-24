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

from abc import ABC, abstractmethod
from pathlib import Path
from typing import List, NamedTuple, Optional

from ros_cross_compile.data_collector import DataCollector
from ros_cross_compile.docker_client import DockerClient
from ros_cross_compile.platform import Platform


"""
A NamedTuple that collects the customizations for each stage passed in by the user.

As such, the documentation for each customization can be found by looking at the
argparse options in ros_cross_compile.py.
"""
PipelineStageOptions = NamedTuple(
    'PipelineStageOptions',
    [
        ('skip_rosdep_keys', List[str]),
        ('custom_script', Optional[Path]),
        ('custom_data_dir', Optional[Path]),
        ('custom_setup_script', Optional[Path]),
        ('runtime_tag', Optional[str]),
    ])


class PipelineStage(ABC):
    """Interface to represent a stage of the cross compile pipeline."""

    @abstractmethod
    def __init__(self, name):
        self._name = name

    @property
    def name(self):
        return self._name

    @abstractmethod
    def __call__(
        self,
        platform: Platform,
        docker_client: DockerClient,
        ros_workspace_dir: Path,
        options: PipelineStageOptions,
        data_collector: DataCollector
    ):
        raise NotImplementedError
