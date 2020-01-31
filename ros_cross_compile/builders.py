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

import docker

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def run_emulated_docker_build(image_tag: str, workspace_path: Path) -> None:
    """
    Spin up a sysroot docker container and run an emulated build inside.

    :param image_tag: Docker image that contains all preinstalled dependencies for workspace.
    :param workspace: Absolute path to the user's source workspace.
    """
    docker_client = docker.from_env()
    docker_client.containers.run(
        image=image_tag,
        environment={
            'OWNER_USER': os.getuid(),
        },
        volumes={
            str(workspace_path): {'bind': '/ros_ws', 'mode': 'rw'},
        },
        remove=True,
    )
