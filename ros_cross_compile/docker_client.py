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
import io
import logging
from pathlib import Path
import tarfile
from typing import Dict
from typing import Optional

import docker
from docker.utils import kwargs_from_env as docker_kwargs_from_env

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('Docker Client')

DEFAULT_COLCON_DEFAULTS_FILE = 'defaults.yaml'


class GeneratorStream(io.RawIOBase):
    def __init__(self, generator):
        self.leftover = None
        self.generator = generator

    def readable(self):
        return True

    def readinto(self, b):
        try:
            length = len(b)  # : We're supposed to return at most this much
            chunk = self.leftover or next(self.generator)
            output, self.leftover = chunk[:length], chunk[length:]
            b[:len(output)] = output
            return len(output)
        except StopIteration:
            return 0  # : Indicate EOF


class DockerClient:
    """Simplified Docker API for this package's usage patterns."""

    def __init__(
        self,
        disable_cache: bool = False,
        default_docker_dir: Optional[Path] = None,
        colcon_defaults_file: Optional[Path] = None,
    ):
        """
        Construct the DockerClient.

        :param disable_cache: If True, disable the Docker cache when building images.
        """
        self._client = docker.from_env()
        self._disable_cache = disable_cache
        self._default_docker_dir = str(default_docker_dir or Path(__file__).parent / 'docker')
        self._colcon_defaults_file = str(colcon_defaults_file or DEFAULT_COLCON_DEFAULTS_FILE)

    def build_image(
        self,
        dockerfile_name: str,
        tag: str,
        dockerfile_dir: Optional[Path] = None,
        buildargs: Optional[Dict[str, str]] = None,
    ) -> None:
        """
        Build a Docker image from a Dockerfile.

        :param dockerfile_dir: Absolute path to directory where Dockerfile can be found,
            defaults to the 'docker' directory in this package.
        :param dockerfile_name: The name of the Dockerfile to build.
        :param tag: What tag to give the created image.
        :param buildargs: Optional dictionary of str->str to set arguments for the build.
        :return None
        :raises docker.errors.BuildError: on build error
        """
        # Use low-level API to expose logs for image building
        docker_api = docker.APIClient(**docker_kwargs_from_env())
        logger.info('Sending context to Docker client')
        log_generator = docker_api.build(
            path=str(dockerfile_dir) if dockerfile_dir else self._default_docker_dir,
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
        container_name: Optional[str] = None,
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
        docker_volumes = {
            str(src): {
                'bind': dest,
                'mode': 'rw',
            }
            for src, dest in volumes.items()
        }
        environment['COLCON_DEFAULTS_FILE'] = self._colcon_defaults_file
        # Note that the `run` kwarg `stream` is not available
        # in the version of dockerpy that we are using, so we must detach to live-stream logs
        # Do not `remove` so that the container can be queried for its exit code after finishing
        logger.info("Running docker container of image {}".format(image_name))
        container = self._client.containers.run(
            image=image_name,
            name=container_name,
            command=command,
            environment=environment,
            volumes=docker_volumes,
            detach=True,
            network_mode='host',
        )
        try:
            logs = container.logs(stream=True)
            for line in logs:
                logger.info(line.decode('utf-8').rstrip())
            exit_code = container.wait()
        finally:
            container.stop()
            container.remove()

        if docker.version_info[0] >= 3:
            exit_code = exit_code['StatusCode']

        if exit_code:
            raise docker.errors.ContainerError(
                image_name, exit_code, '', image_name, 'See above ^')

    def get_image_size(self, img_name: str) -> int:
        return self._client.images.get(img_name).attrs['Size']

    def export_image_filesystem(self, image_tag: str):
        container = self._client.containers.run(image=image_tag, detach=True)
        export_generator = container.export()
        stream = io.BufferedReader(GeneratorStream(export_generator))
        tar = tarfile.open(fileobj=stream, mode='r|*')
        return tar
