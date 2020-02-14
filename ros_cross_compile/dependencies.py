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
from pathlib import Path

from ros_cross_compile.docker_client import DockerClient
from ros_cross_compile.platform import Platform

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('Rosdep Gatherer')


def gather_rosdeps(
    docker_client: DockerClient, platform: Platform, workspace: Path, docker_dir: Path,
) -> None:
    """
    Run the rosdep Docker image, which outputs a script for dependency installation.

    :param docker_client: Will be used to run the container
    :param platform: The name of the image produced by `build_rosdep_image`
    :param workspace: Absolute path to the colcon source workspace.
    :param docker_dir: Absolute path to directory where Dockerfiles can be found.
    :return None
    """
    image_name = 'ros_cross_compile:rosdep'
    logger.info('Building rosdep collector image: %s', image_name)
    docker_client.build_image(
        dockerfile_dir=docker_dir,
        dockerfile_name='rosdep.Dockerfile',
        tag=image_name,
    )

    logger.info('Running rosdep collector image on workspace...')
    docker_client.run_container(
        image_name=image_name,
        environment={
            'ROSDISTRO': platform.ros_distro,
            'TARGET_OS': '{}:{}'.format(platform.os_name, platform.os_distro),
        },
        volumes={
            workspace: '/root/ws'
        },
    )
