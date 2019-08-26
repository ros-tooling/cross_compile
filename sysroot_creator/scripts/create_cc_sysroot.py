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

"""Script to generate sysroot for cross-compiling ROS2."""

import argparse
import logging
import os
import re
import shutil
import subprocess
import tarfile
import tempfile
from pathlib import Path
from typing import Dict

import docker

import jinja2


CC_BUILD_SETUP_FILE_TEMPLATE = jinja2.Template("""
shell=`echo $SHELL | awk -F/ '{print $NF}'`
if [ -d {{ros_root}} ]
then
    source {{ros_root}}/setup.$shell
else
    echo "WARNING: no ROS distro found on the sysroot"
fi

export TARGET_ARCH={{target_arch}}
export TARGET_TRIPLE={{target_triple}}
export CC_ROOT={{cc_root}}
""")  # noqa

CC_BUILD_SYSTEM_SETUP_SCRIPT_TEMPLATE = jinja2.Template("""
sudo rm -rf /lib/{{target_triple}}
sudo ln -s {{cc_root}}/sysroot/lib/{{target_triple}} /lib/{{target_triple}}
sudo rm -rf /usr/lib/{{target_triple}}
sudo ln -s {{cc_root}}/sysroot/usr/lib/{{target_triple}} /usr/lib/{{target_triple}}

CROSS_COMPILER_LIB=/usr/{{target_triple}}/lib
CROSS_COMPILER_LIB_BAK=/usr/{{target_triple}}/lib_$(date +%s).bak
echo "Backing up $CROSS_COMPILER_LIB to $CROSS_COMPILER_LIB_BAK"
sudo mv $CROSS_COMPILER_LIB $CROSS_COMPILER_LIB_BAK
sudo ln -s {{cc_root}}/sysroot/lib/{{target_triple}} $CROSS_COMPILER_LIB
""")  # noqa

SYSROOT_DIR_NAME: str = 'sysroot'
DOCKER_WS_NAME: str = 'Dockerfile_workspace'
DOCKER_CLIENT = docker.from_env()
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class Platform:
    """A class that represents platform specification for cross compiling.

    Includes:
    1. Target architecture
    2. Target operating system
    3. ROS2 distribution used
    4. RMW implementation used
    """

    def __init__(self, args):
        """Initialize platform parameters."""
        self.arch = args.arch
        self.os = args.os
        self.distro = args.distro
        self.rmw = args.rmw

        if self.arch == 'armhf':
            self.cc_toolchain = 'arm-linux-gnueabihf'
        elif self.arch == 'aarch64':
            self.cc_toolchain = 'aarch64-linux-gnu'

    @property
    def __str__(self):
        """Return string representation of platform parameters."""
        return '-'.join((self.arch, self.os, self.rmw, self.distro))


class DockerConfig:
    """
    A class that represents docker build parameters used in creating sysroot.

    Includes:
    1. Base docker image to use for building sysroot
    2. Docker network mode
    3. Setting to enable/disable caching during docker build
    """

    _default_docker_base_image: Dict[tuple, str] = {
        ('armhf', 'ubuntu'): 'arm32v7/ubuntu:bionic',
        ('armhf', 'debian'): 'arm32v7/debian:latest',
        ('aarch64', 'ubuntu'): 'arm64v8/ubuntu:bionic',
        ('aarch64', 'debian'): 'arm64v8/debian:latest',
    }

    def __init__(self, args):
        """Initialize docker configuration."""
        if args.sysroot_base_image is None:
            self.base_image = \
                self._default_docker_base_image[args.arch, args.os]
        else:
            self.base_image = args.sysroot_base_image

        self.network_mode = args.docker_network_mode
        self.nocache = args.sysroot_nocache

    @property
    def __str__(self):
        """Return string representation of docker build parameters."""
        return 'Base Image: {}\nNetwork Mode: {}\nCaching: {}'.format(
            self.base_image, self.network_mode, self.nocache)


def setup_cc_root_dir(platform: Platform) -> Path:
    """Create cross-compile root directory."""
    if not isinstance(platform, Platform):
        raise TypeError('Argument `platform` must be of type Platform.')
    logger.info('Creating workspace directory...')
    cc_root = Path.cwd() / platform.__str__
    cc_root.mkdir(parents=True, exist_ok=True)
    (cc_root / 'COLCON_IGNORE').touch()
    return cc_root


def setup_sysroot_dir(cc_root_dir: Path, force_sysroot_build: bool) -> Path:
    """Create sysroot directory."""
    logger.info('Creating sysroot directory...')
    target_sysroot = cc_root_dir / SYSROOT_DIR_NAME

    if target_sysroot.exists():
        if not force_sysroot_build:
            logger.info(
                'Using existing sysroot found at: {}'.format(target_sysroot))
        else:
            logger.info('Sysroot exists - forcing rebuild')
    else:
        logger.warning('Sysroot not found, building now')
        os.makedirs(str(target_sysroot))

    return target_sysroot


def get_workspace_image_tag(platform: Platform) -> str:
    """Generate docker image name and tag."""
    return os.getenv('USER') + '/' + platform.__str__ + ':latest'


def build_workspace_sysroot_image(
        platform: Platform,
        docker_args: DockerConfig,
        image_tag: str) -> None:
    """Build the target sysroot docker image."""
    if not isinstance(platform, Platform):
        raise TypeError('Argument `platform` must be of type Platform.')
    if not isinstance(docker_args, DockerConfig):
        raise TypeError('Argument `docker_args` must be of type DockerConfig.')
    if not isinstance(image_tag, str):
        raise TypeError('Argument `image_tag` must be of type str.')
    logger.info(
        'Fetching sysroot base image: {}'.format(docker_args.base_image))
    DOCKER_CLIENT.images.pull(docker_args.base_image)
    # FIXME consider moving constants to static fields
    workspace_dockerfile_path = (Path(__file__).parent / SYSROOT_DIR_NAME /
                                 DOCKER_WS_NAME)

    buildargs = {
        'ROS2_BASE_IMG': docker_args.base_image,
        'ROS2_WORKSPACE': './' + SYSROOT_DIR_NAME,
        'ROS_DISTRO': platform.distro,
        'TARGET_TRIPLE': platform.cc_toolchain,
        'TARGET_ARCH': platform.arch,
    }

    logger.info('Building workspace image: {}'.format(image_tag))
    # Switch to low-level API to expose build logs
    docker_client = docker.APIClient(base_url='unix://var/run/docker.sock')
    log_generator = docker_client.build(
        path='.',
        dockerfile=str(workspace_dockerfile_path),
        tag=image_tag,
        buildargs=buildargs,
        quiet=False,
        nocache=docker_args.nocache,
        network_mode=docker_args.network_mode,
        decode=True)
    for chunk in log_generator:
        # There are usually two outputs we want to capture, stream and error.
        # We also want to remove newline (\n) and carriage returns (\r) to
        # avoid mangled output.
        error_line = chunk.get('error', None)
        if error_line:
            logger.exception('Error building sysroot image. The following error'
                             'was caught:\n{}'.format(error_line))
            raise docker.errors.BuildError(reason=error_line, build_log=error_line)
        line = chunk.get('stream', '')
        line = line.rstrip().lstrip()
        if line:
            logger.info(line)

    logger.info(
        'Successfully created sysroot docker image: {}'.format(image_tag))


def export_workspace_sysroot_image(
        image_tag: str, target_sysroot_path: Path) -> None:
    """Export sysroot filesystem into sysroot directory."""
    logger.info('Exporting sysroot to path [{}]'.format(target_sysroot_path))
    shutil.rmtree(str(target_sysroot_path), ignore_errors=True)
    tmp_sysroot_dir = tempfile.mkdtemp(suffix='-cc_build')
    sysroot_tarball_path = Path(tmp_sysroot_dir) / (SYSROOT_DIR_NAME + '.tar')
    logger.info('Exporting filesystem of image {} into tarball {}'.format(
        image_tag, sysroot_tarball_path))
    try:
        sysroot_container = DOCKER_CLIENT.containers.run(
            image=image_tag, detach=True)
        with open(str(sysroot_tarball_path), 'wb') as out_f:
            out_f.writelines(sysroot_container.export())
        sysroot_container.stop()
        with tarfile.open(str(sysroot_tarball_path)) as sysroot_tar:
            relevant_dirs = [
                'lib', 'usr', 'etc', 'opt', 'root_path', 'ros2_ws/install']
            relevant_members = (
                m for m in sysroot_tar.getmembers()
                if re.match('^({}).*'.format('|'.join(relevant_dirs)), m.name)
                is not None)
            sysroot_tar.extractall(str(target_sysroot_path),
                                   members=relevant_members)
    finally:
        shutil.rmtree(tmp_sysroot_dir, ignore_errors=True)
    logger.info(
        'Success exporting sysroot to path [{}]'.format(target_sysroot_path))


def write_cc_build_setup_file(cc_root_dir: Path, platform: Platform) -> Path:
    """Create setup file for cross-compile build."""
    cc_build_setup_file_path = cc_root_dir / 'cc_build_setup.bash'
    cc_build_setup_file_contents = CC_BUILD_SETUP_FILE_TEMPLATE.render(
        target_arch=platform.arch,
        target_triple=platform.cc_toolchain,
        cc_root=cc_root_dir,
        ros_root='{cc_root_dir}/sysroot/opt/ros/{distro}'.format(
            cc_root_dir=cc_root_dir, distro=platform.distro))
    with open(str(cc_build_setup_file_path), 'w') as out_f:
        out_f.write(cc_build_setup_file_contents)
    return cc_build_setup_file_path


def write_cc_system_setup_script(
        cc_root_dir: Path, platform: Platform) -> Path:
    """Create setup file for sysroot setup."""
    cc_system_setup_script_path = cc_root_dir / 'cc_system_setup.bash'
    cc_system_setup_script_contents = \
        CC_BUILD_SYSTEM_SETUP_SCRIPT_TEMPLATE.render(
            cc_root=cc_root_dir,
            target_triple=platform.cc_toolchain
        )
    with open(str(cc_system_setup_script_path), 'w') as out_f:
        out_f.write(cc_system_setup_script_contents)
    return cc_system_setup_script_path


def setup_sysroot_environment(
        sys_setup_path: Path, build_setup_path: Path) -> None:
    """Set up the environment with variables and symbolic links."""
    logger.info('Sourcing sysroot environment...')
    logger.info("Executing 'bash {}'".format(sys_setup_path))
    subprocess.run(['bash', str(sys_setup_path)])
    logger.info("Executing 'source {}'".format(build_setup_path))
    subprocess.run(['source', str(build_setup_path)], shell=True)


def create_arg_parser():
    """Parse command line arguments."""
    example_text = '''Example usage:
    
    python3 create_cc_sysroot.py --arch armhf --os debian
    python3 create_cc_sysroot.py --arch aarch64 --os ubuntu
    python3 create_cc_sysroot.py -a armhf -o ubuntu -d dashing -r fastrtps \
--sysroot-base-image arm64v8/ubuntu:bionic
    '''
    parser = argparse.ArgumentParser(
        description='Sysroot creator for cross compilation workflows.',
        epilog=example_text,
        formatter_class=argparse.RawDescriptionHelpFormatter)
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
        help='Base Docker image to use for building the sysroot. '
             'Ex. arm64v8/ubuntu:bionic')
    parser.add_argument(
        '--docker-network-mode',
        required=False,
        type=str,
        default='host',
        help="Docker's network_mode parameter to use for all Docker "
             'interactions')
    parser.add_argument(
        '--sysroot-nocache',
        required=False,
        type=bool,
        default=False,
        help="When set to true, this disables Docker's cache when building "
             'the image for the workspace sysroot')
    parser.add_argument(
        '--force-sysroot-build',
        required=False,
        type=bool,
        default=False,
        help='When set to true, we rebuild the sysroot and sysroot image, '
             'even if any of those is available')
    parser.add_argument(
        '--ros2-workspace',
        required=False,
        type=str,
        default='./ros2_ws',
        help="The location of the ROS2 workspace you'll be cross compiling "
             'against. Usually ./ros2_ws if you moved it correctly.')
    return parser


def main():
    """Start the cross-compilation workflow."""
    # Configuration
    parser = create_arg_parser()
    args = parser.parse_args()
    platform = Platform(args)
    docker_args = DockerConfig(args)

    # Main pipeline
    cc_root_dir = setup_cc_root_dir(platform)

    sysroot_dir = setup_sysroot_dir(cc_root_dir, args.force_sysroot_build)

    docker_image_tag = get_workspace_image_tag(platform)

    build_workspace_sysroot_image(platform, docker_args, docker_image_tag)

    export_workspace_sysroot_image(docker_image_tag, sysroot_dir)

    cc_build_setup_file_path = write_cc_build_setup_file(cc_root_dir, platform)

    # generalization of the Poco hack
    # from https://github.com/ros2/cross_compile/blob/master/entry_point.sh#L38
    cc_system_setup_script_path = write_cc_system_setup_script(cc_root_dir,
                                                               platform)

    setup_sysroot_environment(cc_system_setup_script_path,
                              cc_build_setup_file_path)

    logger.info("""
    To setup the cross compilation build environment:

    1. Run the command below to setup using sysroot's GLIBC for \
cross-compilation.
       bash {cc_system_setup_script_path}

    2. Run the command below to export the environment variables used by the \
cross-compiled ROS packages.
       source {cc_build_setup_file_path}

    """.format(cc_system_setup_script_path=cc_system_setup_script_path,
               cc_build_setup_file_path=cc_build_setup_file_path))


if __name__ == '__main__':
    main()
