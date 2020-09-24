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

import logging
import os
from pathlib import Path

from ros_cross_compile.data_collector import DataCollector
from ros_cross_compile.docker_client import DockerClient
from ros_cross_compile.pipeline_stages import PipelineStage
from ros_cross_compile.pipeline_stages import PipelineStageOptions
from ros_cross_compile.platform import Platform

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def create_runtime_image(
    docker_client: DockerClient, platform: Platform, workspace_dir: Path, image_tag: str
):
    logger.info('Building runtime image: {}'.format(image_tag))
    docker_client.build_image(
        dockerfile_name=os.path.join(docker_client._default_docker_dir, 'runtime.Dockerfile'),
        dockerfile_dir=workspace_dir,
        tag=image_tag,
        buildargs={
            'BASE_IMAGE': platform.sysroot_image_tag,
            'INSTALL_PATH': 'install_{}'.format(platform.arch),
        },
    )


class PackageRuntimeImageStage(PipelineStage):
    """
    This stage determines what external dependencies are needed for building.

    It outputs a script into the internals directory that will install those
    dependencies for the target platform.
    """

    NAME = 'runtime'

    def __init__(self):
        super().__init__(self.NAME)

    def __call__(
        self, platform: Platform, docker_client: DockerClient, ros_workspace_dir: Path,
        options: PipelineStageOptions,
        data_collector: DataCollector
    ):
        tag = options.runtime_tag
        assert tag is not None
        create_runtime_image(docker_client, platform, ros_workspace_dir, tag)
        img_size = docker_client.get_image_size(tag)
        data_collector.add_size(self.name, img_size)
