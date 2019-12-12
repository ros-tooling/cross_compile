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
from os import path

from xml.etree import ElementTree

from setuptools import find_packages
from setuptools import setup

package_name = 'cross_compile'


def build_package_directory_path():
    """
    Return the directory containing the package code, following symbolic links.

    When compiling this package with colcon, users can optionally pass
    a --symlink-install flag which will create a symlink of setup.py into
    <prefix>/build/<pkg>.
    Using this function, a path will be returned containing not only setup.py,
    and some subset of files for which a symlink exists, but the original
    package directory.
    """
    try:
        return os.path.dirname(os.readlink(__file__))
    except OSError as os_error:
        if os_error.errno == errno.EINVAL:
            return os.path.dirname(__file__)


def read_version_from_package_xml():
    tree = ElementTree.parse('package.xml')
    version = tree.find('version')
    return version.text


this_directory = build_package_directory_path()
with open(path.join(this_directory, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name=package_name,
    version=read_version_from_package_xml(),
    packages=find_packages(exclude=['test']),
    author='ROS Tooling Working Group',
    author_email='ros-tooling@googlegroups.com',
    maintainer='ROS Tooling Working Group',
    maintainer_email='ros-tooling@googlegroups.com',
    url='https://github.com/ros-tooling/cross_compile',
    download_url='https://github.com/ros-tooling/cross_compile/releases',
    keywords=['ROS'],
    classifiers=[
        'Development Status :: 4 - Beta',
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Topic :: Software Development',
    ],
    description='A tool to cross-compile ROS 2 packages.',
    long_description=long_description,
    long_description_content_type='text/markdown',
    license='Apache License, Version 2.0',
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        ('share/ament_index/resource_index/packages',
         [os.path.join('resource', package_name)]),
    ],
    package_data={
        package_name: ['Dockerfile_ros']
    },
    install_requires=[
        'setuptools',
        'docker',
    ],
    zip_safe=True,
    tests_require=[
        'pytest',
        'flake8'
    ],
    entry_points={
        'console_scripts': [
            'cross_compile = cross_compile.ros2_cross_compile:main'
        ]
    },
    python_requires='>=3.5',
)
