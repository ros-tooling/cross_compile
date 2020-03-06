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

import platform
import unittest
from unittest.mock import patch

import docker
import pytest

from ros_cross_compile.docker_client import DockerClient

NO_MAC_REASON = 'CI environment cannot install Docker on Mac OS hosts.'
IS_MAC = platform.system() == 'Darwin'


class DockerClientTest(unittest.TestCase):

    def remove_container(self, client: docker.client.DockerClient, name: str) -> None:
        try:
            preexisting_container = client.containers.get(name)
            preexisting_container.remove()
        except docker.errors.NotFound:
            pass

    def test_parse_docker_build_output(self):
        """Test the SysrootCreator constructor assuming valid path setup."""
        # Create mock directories and files
        client = DockerClient()
        log_generator_without_errors = [
            {'stream': ' ---\\u003e a9eb17255234\\n'},
            {'stream': 'Step 1 : VOLUME /data\\n'},
            {'stream': ' ---\\u003e Running in abdc1e6896c6\\n'},
            {'stream': ' ---\\u003e 713bca62012e\\n'},
            {'stream': 'Removing intermediate container abdc1e6896c6\\n'},
            {'stream': 'Step 2 : CMD [\\"/bin/sh\\"]\\n'},
            {'stream': ' ---\\u003e Running in dba30f2a1a7e\\n'},
            {'stream': ' ---\\u003e 032b8b2855fc\\n'},
            {'stream': 'Removing intermediate container dba30f2a1a7e\\n'},
            {'stream': 'Successfully built 032b8b2855fc\\n'},
        ]
        # Just expect it not to raise
        client._process_build_log(log_generator_without_errors)

        log_generator_with_errors = [
            {'stream': ' ---\\u003e a9eb17255234\\n'},
            {'stream': 'Step 1 : VOLUME /data\\n'},
            {'stream': ' ---\\u003e Running in abdc1e6896c6\\n'},
            {'stream': ' ---\\u003e 713bca62012e\\n'},
            {'stream': 'Removing intermediate container abdc1e6896c6\\n'},
            {'stream': 'Step 2 : CMD [\\"/bin/sh\\"]\\n'},
            {'error': ' ---\\COMMAND NOT FOUND\\n'},
        ]
        with pytest.raises(docker.errors.BuildError):
            client._process_build_log(log_generator_with_errors)

    @pytest.mark.skipif(IS_MAC, reason=NO_MAC_REASON)
    def test_fail_docker_run(self):
        client = DockerClient()
        with pytest.raises(docker.errors.ContainerError):
            client.run_container('ubuntu:18.04', command='/bin/sh -c "exit 1"')

    @pytest.mark.skipif(IS_MAC, reason=NO_MAC_REASON)
    def test_stream(self):
        client = DockerClient()
        test_command = 'echo message1 && sleep 2 && echo message2'
        with self.assertLogs('Docker Client', level='INFO') as cm:
            client.run_container('ubuntu:18.04', command='/bin/sh -c "{}"'.format(test_command))

        timestamps = [r.created for r in cm.records]
        assert len(timestamps) == 2, 'Did not receive all expected messages'

        # we know for sure that the logs were streaming if we printed them with a gap between
        # we do not check for the full 2 seconds because sleep is not that precise,
        # but even on the slowest test environment consecutive prints will not take a full second
        timediff = timestamps[1] - timestamps[0]
        assert timediff >= 1

    @pytest.mark.skipif(IS_MAC, reason=NO_MAC_REASON)
    def test_removal(self):
        client = DockerClient()
        api_client = client._client
        name = 'test_removing_run_container'
        self.remove_container(api_client, name)

        client.run_container(
            'ubuntu:18.04', command='/bin/sh -c "echo hello"', container_name=name)

        containers = api_client.containers.list(all=True, filters={'name': name})
        assert len(containers) == 0

    @pytest.mark.skipif(IS_MAC, reason=NO_MAC_REASON)
    def test_removal_on_failure(self):
        client = DockerClient()
        api_client = client._client
        name = 'test_removing_failed_container'
        self.remove_container(api_client, name)

        def mock_logs(*args, **kwargs):
            raise docker.errors.APIError('Logs raises an exception in this test')

        with patch('docker.models.containers.Container.logs', side_effect=mock_logs):
            with pytest.raises(docker.errors.APIError):
                client.run_container(
                    'ubuntu:18.04', command='/bin/sh -c "echo hello"', container_name=name)

        containers = client._client.containers.list(all=True, filters={'name': name})
        assert len(containers) == 0
