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
from pathlib import Path
from unittest.mock import Mock

from ros_cross_compile.builders import run_emulated_docker_build
from ros_cross_compile.platform import Platform


def test_emulated_docker_build():
    # Very simple smoke test to validate that all internal syntax is correct
    mock_docker_client = Mock()
    platform = Platform('aarch64', 'ubuntu', 'eloquent')
    run_emulated_docker_build(mock_docker_client, platform, Path('dummy_path'))
    assert mock_docker_client.run_container.call_count == 1
