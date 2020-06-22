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
from datetime import datetime
import docker
import logging
from pathlib import Path
import sys
import time
from typing import List
from typing import Optional

from ros_cross_compile.builders import run_emulated_docker_build
from ros_cross_compile.data_collector import data_collector
from ros_cross_compile.dependencies import assert_install_rosdep_script_exists
from ros_cross_compile.dependencies import gather_rosdeps
from ros_cross_compile.docker_client import DEFAULT_COLCON_DEFAULTS_FILE
from ros_cross_compile.docker_client import DockerClient
from ros_cross_compile.pipeline_stages import pipeline_stage
from ros_cross_compile.platform import Platform
from ros_cross_compile.platform import SUPPORTED_ARCHITECTURES
from ros_cross_compile.platform import SUPPORTED_ROS2_DISTROS
from ros_cross_compile.platform import SUPPORTED_ROS_DISTROS
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
        '--print-metrics',
        action='store_true',
        required=False,
        help='Print the collected metrics to its own logging level, METRIC. This option is provided'
              'to give the user flexibility when working with automated solutions and may not want'
              'stdout output.')
    parser.add_argument(
        '--metric-file-path',
        required=False,
        type=str,
        help='Path to file where metric data is written.')

    return parser.parse_args(args)


def cross_compile_pipeline(
    args: argparse.Namespace,
):
    platform = Platform(args.arch, args.os, args.rosdistro, args.sysroot_base_image)
    dc = data_collector()

    # define paths for reading/writing
    ros_workspace_dir = Path(args.ros_workspace)
    metrics_dir = Path(str(ros_workspace_dir) + '/cc_internals/metrics_data/')
    skip_rosdep_keys = args.skip_rosdep_keys
    custom_data_dir = _path_if(args.custom_data_dir)
    custom_rosdep_script = _path_if(args.custom_rosdep_script)
    custom_setup_script = _path_if(args.custom_setup_script)

    # set up class wrappers for functions
    gather_rosdeps_wrapper = pipeline_stage("gather_rosdeps", gather_rosdeps)
    rosdep_script_wrapper = pipeline_stage("assert_install_rosdep_script_exists",
                                           assert_install_rosdep_script_exists)
    create_workspace_sysroot_wrapper = pipeline_stage("create_workspace_sysroot_image",
                                                      create_workspace_sysroot_image)
    run_docker_build_wrapper = pipeline_stage("run_emulated_docker_build",
                                              run_emulated_docker_build)


    # start timing of entire pipeline, and finalize metrics collection setup
    metrics_dir.mkdir(parents=True, exist_ok=True)
    start = time.time()
    today = datetime.now()

    sysroot_build_context = prepare_docker_build_environment(
        platform=platform,
        ros_workspace=ros_workspace_dir,
        custom_setup_script=custom_setup_script,
        custom_data_dir=custom_data_dir)
    docker_client = DockerClient(
        args.sysroot_nocache,
        default_docker_dir=sysroot_build_context,
        colcon_defaults_file=args.colcon_defaults)

    # stage 1
    if not args.skip_rosdep_collection:
        gather_rosdeps_wrapper((
            docker_client,
            platform,
            ros_workspace_dir,
            skip_rosdep_keys,
            custom_rosdep_script,
            custom_data_dir), dc)

    # get the size of the docker image generated in this stage
    rosdep_img_size = docker.from_env().images.get('ros_cross_compile:rosdep').attrs['Size']
    dc.add_datum("gather_rosdeps-img_size", rosdep_img_size / 1000000, "MB", str(today))

    # stage 2
    rosdep_script_wrapper((ros_workspace_dir, platform), dc)

    # stage 3
    create_workspace_sysroot_wrapper((docker_client, platform), dc)

    # get the size of the docker image generated in this stage
    sysroot_img_size = docker.from_env().images.get('ubuntu/aarch64-ubuntu-foxy:latest').attrs['Size']
    dc.add_datum("sysroot_img-size", sysroot_img_size / 1000000, "MB", str(today))

    # stage 4
    run_docker_build_wrapper((docker_client, platform, ros_workspace_dir), dc)

    # stop timing
    total = time.time() - start
    dc.add_datum("total_pipeline-time", total, "seconds", str(today))

    # write metrics to file.
    if args.metric_file_path is not None:
        dc.write_JSON(args.metric_file_path)
    else:
        dc.write_JSON(str(metrics_dir) + '/' + today.strftime('%s') + '.json')


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
