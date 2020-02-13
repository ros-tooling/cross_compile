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
import docker
import pytest

from ros_cross_compile.docker_client import DockerClient


def test_parse_docker_build_output():
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
