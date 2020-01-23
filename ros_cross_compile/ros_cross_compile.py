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

"""Executable for cross-compiling ROS and ROS 2 packages."""

import argparse
import logging
import os
import sys
from typing import List

from ros_cross_compile.sysroot_compiler import DockerConfig
from ros_cross_compile.sysroot_compiler import Platform
from ros_cross_compile.sysroot_compiler import SysrootCompiler

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def parse_args(args: List[str]) -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        prog='ros_cross_compile',  # this can be invoked from __main__.py, so be explicit
        description='Sysroot creator for cross compilation workflows.')
    parser.add_argument(
        '-a', '--arch',
        required=True,
        type=str,
        choices=Platform.SUPPORTED_ARCHITECTURES.keys(),
        help='Target architecture')
    parser.add_argument(
        '-d', '--rosdistro',
        required=False,
        type=str,
        default='dashing',
        choices=Platform.SUPPORTED_ROS_DISTROS + Platform.SUPPORTED_ROS2_DISTROS,
        help='Target ROS distribution')
    parser.add_argument(
        '-o', '--os',
        required=True,
        type=str,
        # NOTE: not specifying choices here, as different distros may support different lists
        help='Target OS')
    parser.add_argument(
        '-r', '--rmw',
        required=False,
        type=str,
        default='fastrtps',
        choices=['fastrtps', 'cyclonedds'],
        help='Target RMW implementation')
    parser.add_argument(
        '--sysroot-base-image',
        required=False,
        type=str,
        help='Override the default base Docker image to use for building the sysroot. '
             'Ex. "arm64v8/ubuntu:bionic"')
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
        '--ros-workspace',
        required=False,
        type=str,
        default='ros_ws',
        help="The subdirectory of 'sysroot' that contains your 'src' to be built."
             'The output of the cross compilation will be placed in this directory. '
             "Defaults to 'ros_ws'.")
    parser.add_argument(
        '--sysroot-path',
        required=False,
        default=os.getcwd(),
        type=str,
        help="The absolute path to the directory containing 'sysroot' where the "
             "'ros2_ws/src' and 'qemu-user-static' directories you created can be found. "
             'Defaults to the current working directory.')
    parser.add_argument(
        '--custom-setup-script',
        required=False,
        default=None,
        type=str,
        help='Provide a path to a shell script that will be executed in the sysroot container '
             'right before running "rosdep install" for your ROS workspace. This allows for '
             'adding extra apt sources that rosdep may not handle, or other arbitrary setup that '
             'is specific to your application build. See the section on "Custom Setup Script" '
             'in the README.md for more details.')
    parser.add_argument(
        '--custom-data-dir',
        required=False,
        default=None,
        type=str,
        help='Provide a path to a custom arbitrary directory to copy into the sysroot container. '
             'You may use this data in your --custom-setup-script, it will be available as '
             '"./custom_data/" in the current working directory when the script is run.')

    return parser.parse_args(args)


def main():
    """Start the cross-compilation workflow."""
    # Configuration
    args = parse_args(sys.argv[1:])
    platform = Platform(args.arch, args.os, args.rosdistro, args.rmw)
    docker_args = DockerConfig(
        platform,
        args.sysroot_base_image,
        args.docker_network_mode,
        args.sysroot_nocache)

    # Main pipeline
    sysroot_create = SysrootCompiler(cc_root_dir=args.sysroot_path,
                                     ros_workspace_dir=args.ros_workspace,
                                     platform=platform,
                                     docker_config=docker_args,
                                     custom_setup_script_path=args.custom_setup_script,
                                     custom_data_dir=args.custom_data_dir)
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
