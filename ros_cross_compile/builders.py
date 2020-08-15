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


def run_emulated_docker_build(
    docker_client: DockerClient,
    platform: Platform,
    workspace_path: Path
) -> None:
    """
    Spin up a sysroot docker container and run an emulated build inside.

    :param docker_client: Preconfigured to run Docker images.
    :param platform: Information about the target platform.
    :param workspace: Absolute path to the user's source workspace.
    """
    docker_client.run_container(
        image_name=platform.sysroot_image_tag,
        environment={
            'OWNER_USER': str(os.getuid()),
            'ROS_DISTRO': platform.ros_distro,
            'TARGET_ARCH': platform.arch,
        },
        volumes={
            workspace_path: '/ros_ws',
        },
    )


class EmulatedDockerBuildStage(PipelineStage):
    """
    This stage spins up a docker container and runs the emulated build with it.

    Uses the sysroot image from the previous stage.
    """

    def __init__(self):
        super().__init__('emulated_build')

    def __call__(
        self,
        platform: Platform,
        docker_client: DockerClient,
        ros_workspace_dir: Path,
        options: PipelineStageOptions,
        data_collector: DataCollector
    ):
        run_emulated_docker_build(docker_client, platform, ros_workspace_dir)


def run_cross_compile_docker_build(
    docker_client: DockerClient,
    platform: Platform,
    workspace_path: Path,
) -> None:
    docker_client.build_image(
        dockerfile_name='build.Dockerfile',
        tag=platform.build_image_tag,
    )

    docker_client.run_container(
        image_name=platform.build_image_tag,
        environment={
            'OWNER_USER': str(os.getuid()),
            'ROS_DISTRO': platform.ros_distro,
            'TARGET_ARCH': platform.arch,
        },
        volumes={
            workspace_path: '/ros_ws',
        }
    )


class CrossCompileBuild(PipelineStage):
    def __init__(self):
        super().__init__('cross_compile_build')

    def __call__(self, platform: Platform, docker_client: DockerClient, ros_workspace_dir: Path,
                 options: PipelineStageConfigOptions,
                 data_collector: DataCollector):
        run_cross_compile_docker_build(docker_client, platform, ros_workspace_dir)
