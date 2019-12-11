#!/usr/bin/env python

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

"""Executable for cross-compiling ROS2 packages."""

import argparse
import logging
import os
import sys

from cross_compile.sysroot_compiler import DockerConfig
from cross_compile.sysroot_compiler import Platform
from cross_compile.sysroot_compiler import SysrootCompiler

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def create_arg_parser():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Sysroot creator for cross compilation workflows.')
    parser.add_argument(
        '-a', '--arch',
        required=True,
        type=str,
        choices=['armhf', 'aarch64'],
        help='Target architecture')
    parser.add_argument(
        '-o', '--os',
        required=True,
        type=str,
        choices=['ubuntu', 'debian'],
        help='Target OS')
    parser.add_argument(
        '-d', '--distro',
        required=False,
        type=str,
        default='dashing',
        choices=['ardent', 'bouncy', 'crystal', 'dashing'],
        help='Target ROS distribution')
    parser.add_argument(
        '-r', '--rmw',
        required=False,
        type=str,
        default='fastrtps',
        choices=['fastrtps', 'opensplice', 'connext'],
        help='Target RMW implementation')
    parser.add_argument(
        '--sysroot-base-image',
        required=False,
        type=str,
        help='Base Docker image to use for building the sysroot. Ex. arm64v8/ubuntu:bionic')
    parser.add_argument(
        '--docker-network-mode',
        required=False,
        type=str,
        default='host',
        help="Docker's network_mode parameter to use for all Docker interactions")
    parser.add_argument(
        '--sysroot-nocache',
        action='store_true',
        required=False,
        help="When set to true, this disables Docker's cache when building "
             'the Docker image for the workspace')
    parser.add_argument(
        '--ros2-workspace',
        required=False,
        type=str,
        default='ros2_ws',
        help="The subdirectory of 'sysroot' that contains your 'src' to be built."
             'The output of the cross compilation will be placed in this directory. '
             "Defaults to 'ros2_ws'.")
    parser.add_argument(
        '--sysroot-path',
        required=False,
        default=os.getcwd(),
        type=str,
        help="The absolute path to the directory containing 'sysroot' where the "
             "'ros2_ws/src' and 'qemu-user-static' directories you created can be found. "
             'Defaults to the current working directory.')
    parser.add_argument(
        '--extra-dependencies',
        required=False,
        default=None,
        type=str,
        help='Provide a YAML file that describes extra apt sources and dependencies to install. '
             'This allows for custom dependencies that would not otherwise be handled by rosdep. '
             'See the section on "Including Non-rosdep Dependencies" in README.md for format.')
    return parser


def main():
    """Start the cross-compilation workflow."""
    # Configuration
    parser = create_arg_parser()
    args = parser.parse_args()
    platform = Platform(args)
    docker_args = DockerConfig(args)

    # Main pipeline
    sysroot_create = SysrootCompiler(cc_root_dir=args.sysroot_path,
                                     ros_workspace_dir=args.ros2_workspace,
                                     platform=platform,
                                     docker_config=docker_args,
                                     extra_dependencies_path=args.extra_dependencies)
    sysroot_create.execute_cc_pipeline()


if __name__ == '__main__':
    if sys.version_info < (3, 5):
        logger.warning('You are using an unsupported version of Python.'
                       'Cross-compile only supports Python >= 3.5 per the ROS2 REP 2000.')
    try:
        main()
    except Exception as e:
        logger.exception(e)
        exit(1)
