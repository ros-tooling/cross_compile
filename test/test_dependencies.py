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

from ros_cross_compile.dependencies import gather_rosdeps
from ros_cross_compile.platform import Platform


def test_dummy_ros2_pkg():
    platform = Platform(arch='aarch64', os_name='ubuntu', ros_distro='dashing')
    outy = gather_rosdeps(platform, Path('test') / 'dummy_pkg_ros2')
    expected = """#[apt] Installation commands:
  apt-get install -y ros-dashing-rclcpp
  apt-get install -y ros-dashing-ament-cmake
"""
    assert outy == expected, 'Rosdep output did not meet expectations.'
