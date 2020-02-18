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

from contextlib import redirect_stdout
import io
import logging
from pathlib import Path

from ros_cross_compile.platform import Platform
from rosdep2.main import rosdep_main

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('Rosdep Gatherer')


def gather_rosdeps(platform: Platform, sources_path: Path) -> str:
    """
    Get all dependency installation commands for the target workspace.

    :param platform: Information about the target platform.
    :param sources_path: Directory of sources to gather dependencies from.
    :return The exact commands to run for the target platform, as a string.
    :raises RuntimeError if rosdep commands fail to run
    """
    try:
        rosdep_main(['update'])
    except SystemExit as e:
        raise RuntimeError(e)

    target_os = '{}:{}'.format(platform.os_name, platform.os_distro)
    args = [
        'install',
        '--os', target_os,
        '--rosdistro', platform.ros_distro,
        '--from-paths', str(sources_path),
        '--ignore-src',
        '--reinstall',
        '--default-yes',
        '--simulate',
        '--as-root=apt:no',
        '--as-root=pip:no',
    ]

    iocapture = io.StringIO()
    with redirect_stdout(iocapture):
        try:
            rosdep_main(args)
        except SystemExit:
            raise RuntimeError(iocapture.getvalue())
    return iocapture.getvalue()


def write_installer_script(ros_workspace: Path, rosdep_script_contents: str) -> Path:
    relative_path = Path('cc_internals') / 'install_rosdeps.sh'
    full_path = ros_workspace / relative_path
    full_path.parent.mkdir(exist_ok=True)
    full_path.write_text(rosdep_script_contents)
    return relative_path
