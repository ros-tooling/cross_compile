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
from typing import Dict
from typing import Optional

import docker

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('Docker Client')


class DockerClient:
    """Simplified Docker API for this package's usage patterns."""

    def __init__(self, disable_cache: bool = False):
        """
        Construct the DockerClient.

        :param disable_cache: If True, disable the Docker cache when building images.
        """
        self._client = docker.from_env()
        self._disable_cache = disable_cache

    def build_image(
        self,
        dockerfile_dir: Path,
        dockerfile_name: str,
        tag: str,
        buildargs: Optional[Dict[str, str]] = None,
    ) -> None:
        """
        Build a Docker image from a Dockerfile.

        :param dockerfile_dir: Absolute path to directory where Dockerfile can be found.
        :param dockerfile_name: The name of the Dockerfile to build.
        :param tag: What tag to give the created image.
        :param buildargs: Optional dictionary of str->str to set arguments for the build.
        :return None
        :raises docker.errors.BuildError: on build error
        """
        # Use low-level API to expose logs for image building
        docker_api = docker.APIClient(base_url='unix://var/run/docker.sock')
        log_generator = docker_api.build(
            path=str(dockerfile_dir),
            dockerfile=dockerfile_name,
            tag=tag,
            buildargs=buildargs,
            quiet=False,
            nocache=self._disable_cache,
            decode=True,
        )
        self._process_build_log(log_generator)

    def _process_build_log(self, log_generator) -> None:
        for chunk in log_generator:
            # There are two outputs we want to capture, stream and error.
            # We also process line breaks.
            error_line = chunk.get('error', None)
            if error_line:
                logger.exception(
                    'Error building Docker image. The follow error was caught:\n' + error_line)
                raise docker.errors.BuildError(error_line)
            line = chunk.get('stream', '')
            line = line.rstrip()
            if line:
                logger.info(line)

    def run_container(
        self,
        image_name: str,
        command: Optional[str] = None,
        environment: Dict[str, str] = {},
        volumes: Dict[Path, str] = {},
    ) -> None:
        """
        Run a container of an existing image.

        :param image_name: Name of the image to run.
        :param command: Optional command to run on the container
        :param environment: Map of environment variable names to values.
        :param volumes: Map of absolute path to a host directory, to str of mount destination.
        :raises docker.errors.ContainerError if container run has nonzero exit code
        :return None
        """
        docker_volumes = {}
        for src, dest in volumes.items():
            docker_volumes[str(src)] = {
                'bind': dest,
                'mode': 'rw',
            }
        logs = self._client.containers.run(
            image=image_name,
            command=command,
            environment=environment,
            volumes=docker_volumes,
            remove=True,
            stream=True,
            network_mode='host',
        )
        for line in logs:
            logger.info(line.decode('utf-8').rstrip())
