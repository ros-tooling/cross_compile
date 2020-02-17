# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from contextlib import redirect_stdout
import io
import logging
from pathlib import Path

from rosdep2.main import rosdep_main

from ros_cross_compile.platform import Platform


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('Rosdep Gatherer')


def gather_rosdeps_python(platform: Platform, workspace: Path):

    target_os = '{}:{}'.format(platform.os_name, platform.os_distro)
    sources_path = workspace / 'src'
    args = [
        'rosdep', 'install',
        '--os', target_os,
        '--rosdistro', platform.ros_distro,
        '--from-paths', str(sources_path),
        '--ignore-src',
        '--reinstall',
        '--default-yes',
        '--simulate',
    ]

    iocapture = io.StringIO()
    with redirect_stdout(iocapture):
        rosdep_main(args)
    rosdep_output = iocapture.getvalue()
    return rosdep_output
