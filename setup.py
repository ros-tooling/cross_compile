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
import subprocess
import tempfile
from xml.etree import ElementTree

from setuptools import find_packages
from setuptools import setup

package_name = 'cross_compile'
ament_lint_temporary_directory = tempfile.TemporaryDirectory()
# The latest version at the time of this writing.
# It should be safe to bump this to the latest ament linter version at any time.
AMENT_LINTER_VERSION = '0.8.1'
AMENT_LINTERS_TO_INSTALL = [
    'ament_lint',
    'ament_copyright',
    'ament_flake8',
    'ament_pep257',
]


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


def read_readme():
    this_directory = build_package_directory_path()
    with open(path.join(this_directory, 'README.md'), encoding='utf-8') as f:
        return f.read()


def fetch_ament_lint_dependency_links():
    """
    Construct the dependency links needed to install the eggs for the ament linters.

    TODO(#123): remove the dependency_links logic when we move over to tox

    NOTES about dependency links:
    * We are installing ament_lint via a URL because it is not released to PyPI and we are
      no longer depending on rosdep for this pure-python package
    * This feature is deprecated in pip 19 - when we upgrade to using it, we will have to install
      our test packages in a different way. Moving test infra out of setuptools and into
      tox (https://tox.readthedocs.io/en/latest/) seems to be the most recommended way.

    NOTE explaining the local checkout into a temporary directory
    * the ament_lint repository is a meta-repo, containing multiple modules in subdirectories
    * pip allows for installing from URL, including pointing at a subdirectory e.g.
      `git+https://github.com/ament/ament_lint.git#egg=ament_lint&subdirectory=ament_lint`
      but the one part of that URL that doesn't work in setuptools is the subdirectory keyword,
      so we can't use git links
    * Subversion allows checking out subdirectories,
      Github supports Subversion urls,
      and pip supports using them for installation, e.g.
      `svn+https://github.com/ament/ament_lint/trunk/ament_lint`,
      but setuptools fails to invoke subversion properly for these links and raises an error,
      so we can't use SVN links
    * Local file:// URIs work fine in setuptools as tested in our infrastructure, so instead
      of the nice-looking options, we manually check out the ament_lint repository and point
      setuptools at its locally-checked-out subdirectories
    """
    repo_dir = str(ament_lint_temporary_directory.name)
    subprocess.check_call([
        'git', 'clone',
        '--branch', AMENT_LINTER_VERSION,
        '--depth', '1',
        'https://github.com/ament/ament_lint/', repo_dir],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return [
        'file://{base}/{pkg}#egg={pkg}-{version}'.format(
            base=repo_dir, pkg=linter, version=AMENT_LINTER_VERSION)
        for linter in AMENT_LINTERS_TO_INSTALL
    ]


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
    dependency_links=fetch_ament_lint_dependency_links(),
    description='A tool to cross-compile ROS 2 packages.',
    long_description=read_readme(),
    long_description_content_type='text/markdown',
    license='Apache License, Version 2.0',
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        ('share/ament_index/resource_index/packages',
         [os.path.join('resource', package_name)]),
    ],
    package_data={
        package_name: ['Dockerfile_ros', 'mixins/*.*'],
    },
    install_requires=[
        'docker>=2,<3',
        'setuptools',
    ],
    zip_safe=True,
    tests_require=[
        'flake8',
        'flake8-blind-except',
        'flake8-builtins',
        'flake8-class-newline',
        'flake8-comprehensions',
        'flake8-deprecated',
        'flake8-docstrings',
        'flake8-import-order',
        'flake8-quotes',
        'pydocstyle',
        'pytest',
        'pytest-cov',
        'pytest-repeat',
        'pytest-runner',
        'yamllint',
    ] + AMENT_LINTERS_TO_INSTALL,
    entry_points={
        'console_scripts': [
            'cross_compile = cross_compile.ros2_cross_compile:main'
        ]
    },
    python_requires='>=3.5',
)
