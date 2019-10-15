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

import os

from setuptools import find_packages
from setuptools import setup

package_name = 'cross_compile'

setup(
    name=package_name,
    version='0.0.1',
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
