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

import docker
import pytest

from ros_cross_compile.dependencies import gather_rosdeps
from ros_cross_compile.dependencies import rosdep_install_script
from ros_cross_compile.docker_client import DockerClient
from ros_cross_compile.platform import Platform

from .utilities import uses_docker


def make_pkg_xml(contents):
    return """<package format="3">
  <name>dummy</name>
  <version>0.0.0</version>
  <description>Test package</description>
  <maintainer email="example@example.com">Example</maintainer>
  <license>None</license>
  {}
</package>
""".format(contents)


RCLCPP_PKG_XML = make_pkg_xml("""
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
""")

CUSTOM_KEY_PKG_XML = make_pkg_xml("""
  <depend>definitely_does_not_exist</depend>
""")


@uses_docker
def test_dummy_ros2_pkg(tmpdir):
    ws = Path(str(tmpdir))
    pkg_xml = ws / 'src' / 'dummy' / 'package.xml'
    pkg_xml.parent.mkdir(parents=True)
    pkg_xml.write_text(RCLCPP_PKG_XML)

    client = DockerClient()
    platform = Platform(arch='aarch64', os_name='ubuntu', ros_distro='dashing')
    out_script = ws / rosdep_install_script(platform)

    gather_rosdeps(client, platform, workspace=ws)
    result = out_script.read_text().splitlines()
    expected = [
        '#!/bin/bash',
        'set -euxo pipefail',
        '#[apt] Installation commands:',
        '  apt-get install -y ros-dashing-ament-cmake',
        '  apt-get install -y ros-dashing-rclcpp',
    ]
    assert result == expected, 'Rosdep output did not meet expectations.'


@uses_docker
def test_rosdep_bad_key(tmpdir):
    ws = Path(str(tmpdir))
    pkg_xml = Path(ws) / 'src' / 'dummy' / 'package.xml'
    pkg_xml.parent.mkdir(parents=True)
    pkg_xml.write_text(CUSTOM_KEY_PKG_XML)
    client = DockerClient()
    platform = Platform(arch='aarch64', os_name='ubuntu', ros_distro='dashing')
    with pytest.raises(docker.errors.ContainerError):
        gather_rosdeps(client, platform, workspace=ws)


@uses_docker
def test_custom_rosdep_no_data_dir(tmpdir):
    script_contents = """
cat > /test_rules.yaml <<EOF
definitely_does_not_exist:
  ubuntu:
    bionic: [successful_test]
EOF
echo "yaml file:/test_rules.yaml" > /etc/ros/rosdep/sources.list.d/22-test-rules.list
"""
    ws = Path(str(tmpdir))
    pkg_xml = Path(ws) / 'src' / 'dummy' / 'package.xml'
    pkg_xml.parent.mkdir(parents=True)
    pkg_xml.write_text(CUSTOM_KEY_PKG_XML)
    client = DockerClient()
    platform = Platform(arch='aarch64', os_name='ubuntu', ros_distro='dashing')

    rosdep_setup = ws / 'rosdep_setup.sh'
    rosdep_setup.write_text(script_contents)

    gather_rosdeps(client, platform, workspace=ws, custom_script=rosdep_setup)
    out_script = ws / rosdep_install_script(platform)
    result = out_script.read_text().splitlines()
    expected = [
        '#!/bin/bash',
        'set -euxo pipefail',
        '#[apt] Installation commands:',
        '  apt-get install -y successful_test',
    ]
    assert result == expected, 'Rosdep output did not meet expectations.'


@uses_docker
def test_custom_rosdep_with_data_dir(tmpdir):

    rule_contents = """
definitely_does_not_exist:
  ubuntu:
    bionic: [successful_test]
"""

    script_contents = """
#!/bin/bash
set -euxo
cp ./custom-data/test_rules.yaml /test_rules.yaml
echo "yaml file:/test_rules.yaml" > /etc/ros/rosdep/sources.list.d/22-test-rules.list
"""
    ws = Path(str(tmpdir))
    pkg_xml = Path(ws) / 'src' / 'dummy' / 'package.xml'
    pkg_xml.parent.mkdir(parents=True)
    pkg_xml.write_text(CUSTOM_KEY_PKG_XML)
    client = DockerClient()
    platform = Platform(arch='aarch64', os_name='ubuntu', ros_distro='dashing')

    rosdep_setup = ws / 'rosdep_setup.sh'
    rosdep_setup.write_text(script_contents)

    data_dir = ws / 'custom_data'
    data_file = data_dir / 'test_rules.yaml'
    data_file.parent.mkdir(parents=True)
    data_file.write_text(rule_contents)

    gather_rosdeps(
        client, platform, workspace=ws, custom_script=rosdep_setup, custom_data_dir=data_dir)

    out_script = ws / rosdep_install_script(platform)
    result = out_script.read_text().splitlines()
    expected = [
        '#!/bin/bash',
        'set -euxo pipefail',
        '#[apt] Installation commands:',
        '  apt-get install -y successful_test',
    ]
    assert result == expected, 'Rosdep output did not meet expectations.'
