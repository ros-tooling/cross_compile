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
from pathlib import Path
import sys
from typing import List
from typing import Optional

from ros_cross_compile.builders import run_emulated_docker_build
from ros_cross_compile.dependencies import assert_install_rosdep_script_exists
from ros_cross_compile.dependencies import gather_rosdeps
from ros_cross_compile.docker_client import DEFAULT_COLCON_DEFAULTS_FILE
from ros_cross_compile.docker_client import DockerClient
from ros_cross_compile.platform import Platform
from ros_cross_compile.platform import SUPPORTED_ARCHITECTURES
from ros_cross_compile.platform import SUPPORTED_ROS2_DISTROS
from ros_cross_compile.platform import SUPPORTED_ROS_DISTROS
from ros_cross_compile.runtime import create_runtime_image
from ros_cross_compile.sysroot_creator import create_workspace_sysroot_image
from ros_cross_compile.sysroot_creator import prepare_docker_build_environment

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def _path_if(path: Optional[str] = None) -> Optional[Path]:
    return Path(path) if path else None


def parse_args(args: List[str]) -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        prog='ros_cross_compile',  # this can be invoked from __main__.py, so be explicit
        description='Sysroot creator for cross compilation workflows.')
    parser.add_argument(
        'ros_workspace', type=str,
        help='Path of the colcon workspace to be cross-compiled. Contains "src" directory.')
    parser.add_argument(
        '-a', '--arch',
        required=True,
        type=str,
        choices=SUPPORTED_ARCHITECTURES,
        help='Target architecture')
    parser.add_argument(
        '-d', '--rosdistro',
        required=False,
        type=str,
        default='dashing',
        choices=SUPPORTED_ROS_DISTROS + SUPPORTED_ROS2_DISTROS,
        help='Target ROS distribution')
    parser.add_argument(
        '-o', '--os',
        required=True,
        type=str,
        # NOTE: not specifying choices here, as different distros may support different lists
        help='Target OS')
    parser.add_argument(
        '--sysroot-base-image',
        required=False,
        type=str,
        help='Override the default base Docker image to use for building the sysroot. '
             'Ex. "arm64v8/ubuntu:bionic"')
    parser.add_argument(
        '--sysroot-nocache',
        action='store_true',
        required=False,
        help="When set to true, this disables Docker's cache when building "
             'the Docker image for the workspace')
    parser.add_argument(
        '--custom-rosdep-script',
        required=False,
        default=None,
        type=str,
        help='Provide a path to a shell script that will be executed right before collecting '
             'the list of dependencies for the target workspace. This allows you to install '
             'extra rosdep rules/sources that are not in the standard "rosdistro" set. See the '
             'section "Custom Rosdep Script" in README.md for more details.')
    parser.add_argument(
        '--custom-setup-script',
        required=False,
        default=None,
        type=str,
        help='Provide a path to a shell script that will be executed in the sysroot container '
             'right before running "rosdep install" for your ROS workspace. This allows for '
             'adding extra apt sources that rosdep may not handle, or other arbitrary setup that '
             'is specific to your application build. See the section on "Custom Setup Script" '
             'in README.md for more details.')
    parser.add_argument(
        '--custom-data-dir',
        required=False,
        default=None,
        type=str,
        help='Provide a path to a custom arbitrary directory to copy into the sysroot container. '
             'You may use this data in your --custom-setup-script, it will be available as '
             '"./custom_data/" in the current working directory when the script is run.')
    parser.add_argument(
        '--colcon-defaults',
        required=False,
        default=DEFAULT_COLCON_DEFAULTS_FILE,
        type=str,
        help='Relative path within the workspace to a file that provides colcon arguments. '
             'See "Package Selection and Build Customization" in README.md for more details.')
    parser.add_argument(
        '--skip-rosdep-collection',
        action='store_true',
        required=False,
        help='Skip querying rosdep for dependencies. This is intended to save time when running '
             'repeatedly during development, but has undefined behavior if the dependencies of '
             'the workspace have changed since the last time they were collected.')
    parser.add_argument(
        '--skip-rosdep-keys',
        default=[],
        nargs='+',
        help='Skip specified rosdep keys when collecting dependencies for the workspace.')
    parser.add_argument(
        '--create-runtime-image',
        help='Create a Docker image with the specified name that contains all '
             'runtime dependencies and the created "install" directory for the workspace.')

    return parser.parse_args(args)


def cross_compile_pipeline(
    args: argparse.Namespace,
):
    platform = Platform(args.arch, args.os, args.rosdistro, args.sysroot_base_image)

    ros_workspace_dir = Path(args.ros_workspace)
    skip_rosdep_keys = args.skip_rosdep_keys
    custom_data_dir = _path_if(args.custom_data_dir)
    custom_rosdep_script = _path_if(args.custom_rosdep_script)
    custom_setup_script = _path_if(args.custom_setup_script)

    sysroot_build_context = prepare_docker_build_environment(
        platform=platform,
        ros_workspace=ros_workspace_dir,
        custom_setup_script=custom_setup_script,
        custom_data_dir=custom_data_dir)
    docker_client = DockerClient(
        args.sysroot_nocache,
        default_docker_dir=sysroot_build_context,
        colcon_defaults_file=args.colcon_defaults)

    if not args.skip_rosdep_collection:
        gather_rosdeps(
            docker_client=docker_client,
            platform=platform,
            workspace=ros_workspace_dir,
            skip_rosdep_keys=skip_rosdep_keys,
            custom_script=custom_rosdep_script,
            custom_data_dir=custom_data_dir)
    assert_install_rosdep_script_exists(ros_workspace_dir, platform)
    create_workspace_sysroot_image(docker_client, platform)
    run_emulated_docker_build(docker_client, platform, ros_workspace_dir)
    if args.create_runtime_image is not None:
        create_runtime_image(
            docker_client, platform, ros_workspace_dir, args.create_runtime_image)


def main():
    """Start the cross-compilation workflow."""
    args = parse_args(sys.argv[1:])
    cross_compile_pipeline(args)


if __name__ == '__main__':
    if sys.version_info < (3, 5):
        logger.warning('You are using an unsupported version of Python.'
                       'Cross-compile only supports Python >= 3.5 per the ROS2 REP 2000.')
    try:
        main()
    except Exception as e:
        logger.exception(e)
        exit(1)
