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
from os import path

from setuptools import find_packages
from setuptools import setup

package_name = 'ros_cross_compile'

this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name=package_name,
    version='0.10.0',
    packages=find_packages(exclude=['test']),
    author='ROS Tooling Working Group',
    author_email='ros-tooling@googlegroups.com',
    maintainer='ROS Tooling Working Group',
    maintainer_email='ros-tooling@googlegroups.com',
    url='https://github.com/ros-tooling/cross_compile',
    download_url='https://github.com/ros-tooling/cross_compile/releases',
    keywords=['ROS', 'ROS2'],
    classifiers=[
        'Development Status :: 4 - Beta',
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Topic :: Software Development',
    ],
    description='A tool to build ROS workspaces for various target architectures and platforms.',
    long_description=long_description,
    long_description_content_type='text/markdown',
    license='Apache License, Version 2.0',
    package_data={
        package_name: ['docker/*.*', 'mixins/*.*', 'qemu/*'],
    },
    install_requires=[
        'docker>=2,<3',
        'setuptools',
    ],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'ros_cross_compile = ros_cross_compile.ros_cross_compile:main'
        ]
    },
    python_requires='>=3.7',
)
