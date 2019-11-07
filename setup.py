# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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


"""Package setup for cross_compile."""

import errno
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'cross_compile'


def compute_get_version_root_dir():
    """
    Return the directory containing the .git directory for this package.

    When compiling this package with colcon, users can optionally pass
    a --symlink-install flag which will create a symlink of setup.py into
    <prefix>/build/<pkg>. In this case, setuptools-scm will fail to find
    the .git folder and will abort, unable to determine the package version.

    This code ensures that, if setup.py (__file__), is a symbolic link, we
    teach setuptools-scm to look for a .git folder into the directory the
    symbolic link points to.

    This enables setuptools-scm to work whether or not --merge-install is
    passed by the user.
    """
    try:
        return os.path.dirname(os.readlink(__file__))
    except OSError as os_error:
        if os_error.errno == errno.EINVAL:
            return '.'


setup(
    name=package_name,
    use_scm_version={
        'root': compute_get_version_root_dir(),
        'relative_to': __file__
    },
    setup_requires=['setuptools_scm'],
    packages=find_packages(exclude=['test']),
    maintainer='AWS RoboMaker',
    maintainer_email='ros-contributions@amazon.com',
    url='https://github.com/ros2/cross_compile',
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
    ],
    package_data={
        package_name: ['Dockerfile_workspace']
    },
    install_requires=[
        'setuptools',
        'docker',
    ],
    zip_safe=True,
    description='A cross compilation tool for ROS 2 packages.',
    license='Apache License, Version 2.0',
    tests_require=[
        'pytest',
        'flake8'
    ],
    entry_points={
        'console_scripts': [
            'cross_compile = cross_compile.ros2_cross_compile:main'
        ]
    }
)
