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
from typing import List
from typing import Optional

from ros_cross_compile.data_collector import DataCollector
from ros_cross_compile.docker_client import DockerClient
from ros_cross_compile.pipeline_stages import PipelineStage
from ros_cross_compile.pipeline_stages import PipelineStageOptions
from ros_cross_compile.platform import Platform
from ros_cross_compile.sysroot_creator import build_internals_dir, rosdep_install_script

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('Rosdep Gatherer')


CUSTOM_SETUP = '/usercustom/rosdep_setup'
CUSTOM_DATA = '/usercustom/custom-data'
_IMG_NAME = 'ros_cross_compile:rosdep'




def gather_rosdeps(
    docker_client: DockerClient,
    platform: Platform,
    workspace: Path,
    skip_rosdep_keys: List[str] = [],
    custom_script: Optional[Path] = None,
    custom_data_dir: Optional[Path] = None,
) -> None:
    """
    Run the rosdep Docker image, which outputs a script for dependency installation.

    :param docker_client: Will be used to run the container
    :param platform: The name of the image produced by `build_rosdep_image`
    :param workspace: Absolute path to the colcon source workspace.
    :param custom_script: Optional absolute path of a script that does custom setup for rosdep
    :param custom_data_dir: Optional absolute path of a directory containing custom data for setup
    :return None
    """
    out_path = rosdep_install_script(platform)

    logger.info('Building rosdep collector image: %s', _IMG_NAME)
    docker_client.build_image(
        dockerfile_name='rosdep.Dockerfile',
        tag=_IMG_NAME,
    )

    logger.info('Running rosdep collector image on workspace {}'.format(workspace))
    volumes = {
        workspace: '/ws',
    }
    if custom_script:
        volumes[custom_script] = CUSTOM_SETUP
    if custom_data_dir:
        volumes[custom_data_dir] = CUSTOM_DATA

    docker_client.run_container(
        image_name=_IMG_NAME,
        environment={
            'CUSTOM_SETUP': CUSTOM_SETUP,
            'OUT_PATH': str(out_path),
            'OWNER_USER': str(os.getuid()),
            'ROSDISTRO': platform.ros_distro,
            'SKIP_ROSDEP_KEYS': ' '.join(skip_rosdep_keys),
            'TARGET_OS': '{}:{}'.format(platform.os_name, platform.os_distro),
        },
        volumes=volumes,
    )


def assert_install_rosdep_script_exists(
    ros_workspace_dir: Path,
    platform: Platform
) -> bool:
    install_rosdep_script_path = ros_workspace_dir / rosdep_install_script(platform)
    if not install_rosdep_script_path.is_file():
        raise RuntimeError(
            'Rosdep installation script has never been created, you need to run this without '
            'skipping rosdep collection at least once.')
    return True


class CollectDependencyListStage(PipelineStage):
    """
    This stage determines what external dependencies are needed for building.

    It outputs a script into the internals directory that will install those
    dependencies for the target platform.
    """

    def __init__(self):
        super().__init__('gather_rosdeps')

    def __call__(
        self,
        platform: Platform,
        docker_client: DockerClient,
        ros_workspace_dir: Path,
        options: PipelineStageOptions,
        data_collector: DataCollector
    ):
        """
        Run the inspection and output the dependency installation script.

        Also recovers the size of the docker image generated.

        :raises RuntimeError if the step was skipped when no dependency script has been
        previously generated
        """
        gather_rosdeps(
            docker_client=docker_client,
            platform=platform,
            workspace=ros_workspace_dir,
            skip_rosdep_keys=options.skip_rosdep_keys,
            custom_script=options.custom_script,
            custom_data_dir=options.custom_data_dir)
        assert_install_rosdep_script_exists(ros_workspace_dir, platform)

        img_size = docker_client.get_image_size(_IMG_NAME)
        data_collector.add_size(self.name, img_size)
