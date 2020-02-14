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

from ros_cross_compile.dependencies import build_rosdep_image
from ros_cross_compile.dependencies import gather_rosdeps
from ros_cross_compile.platform import Platform


def test_smoke():
    # Very simple smoke test to validate that all internal syntax is correct
    platform = Platform(arch='aarch64', os_name='ubuntu', ros_distro='dashing')

    mock_docker_client = Mock()
    image_tag = build_rosdep_image(mock_docker_client, platform, Path('dummy_path'))
    assert mock_docker_client.build_image.call_count == 1

    gather_rosdeps(mock_docker_client, image_tag, Path('dummy_path'), 'eloquent')
    assert mock_docker_client.run_container.call_count == 1
