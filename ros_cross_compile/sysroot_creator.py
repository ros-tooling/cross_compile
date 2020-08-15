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

from distutils.dir_util import copy_tree
import logging
from pathlib import Path
import platform as py_platform
import shutil
from typing import Optional

from ros_cross_compile.data_collector import DataCollector
from ros_cross_compile.data_collector import INTERNALS_DIR
from ros_cross_compile.docker_client import DockerClient
from ros_cross_compile.pipeline_stages import PipelineStage
from ros_cross_compile.pipeline_stages import PipelineStageOptions
from ros_cross_compile.platform import Platform

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def build_internals_dir(platform: Platform) -> Path:
    """Construct a relative path for this build, for storing intermediate artifacts."""
    return Path(INTERNALS_DIR) / str(platform)


def rosdep_install_script(platform: Platform) -> Path:
    """Construct relative path of the script that installs rosdeps into the sysroot image."""
    return build_internals_dir(platform) / 'install_rosdeps.sh'


def _copytree(src: Path, dest: Path) -> None:
    """Copy contents of directory 'src' into 'dest'."""
    copy_tree(str(src), str(dest))


def _copyfile(src: Path, dest: Path) -> None:
    """Copy a single file to a destination location."""
    shutil.copy(str(src), str(dest))


def _copy_or_touch(src: Optional[Path], dest: Path) -> None:
    """
    Copy a single file, if specified, to a destination.

    If the file src is not provided, create an empty file at dest.
    """
    if src:
        _copyfile(src, dest)
    else:
        dest.touch()


def setup_emulator(arch: str, output_dir: Path) -> None:
    """Copy the appropriate emulator binary to the output location."""
    emulator_name = 'qemu-{}-static'.format(arch)
    bin_dir = output_dir / 'bin'
    bin_dir.mkdir(parents=True, exist_ok=True)
    needs_emulator = (py_platform.system() != 'Darwin') and (py_platform.machine() != arch)
    if needs_emulator:
        emulator_path = Path('/') / 'usr' / 'bin' / emulator_name
        if not emulator_path.is_file():
            raise RuntimeError('Could not find the expected QEmu emulator binary "{}"'.format(
                emulator_path))
        _copyfile(emulator_path, bin_dir)
    else:
        (bin_dir / emulator_name).touch()


def prepare_docker_build_environment(
    platform: Platform,
    ros_workspace: Path,
    custom_setup_script: Optional[Path] = None,
    custom_post_build_script: Optional[Path] = None,
    custom_data_dir: Optional[Path] = None,
) -> Path:
    """
    Prepare the directory for the sysroot.Docker build context.

    :param platform Information about the target platform
    :param ros_workspace Location of the ROS source workspace
    :param custom_setup_script Optional arbitrary script
    :param custom_data_dir Optional arbitrary directory for use by custom_setup_script
    :return The directory that was created.
    """
    package_dir = Path(__file__).parent
    docker_build_dir = ros_workspace / build_internals_dir(platform)
    docker_build_dir.mkdir(parents=True, exist_ok=True)
    (docker_build_dir.parent / 'COLCON_IGNORE').touch()

    _copytree(package_dir / 'docker', docker_build_dir)
    _copytree(package_dir / 'mixins', docker_build_dir / 'mixins')
    _copytree(package_dir / 'toolchains', docker_build_dir / 'toolchains')

    custom_data_dest = docker_build_dir / 'user-custom-data'
    if custom_data_dir:
        _copytree(custom_data_dir, custom_data_dest)
    else:
        custom_data_dest.mkdir(exist_ok=True)

    _copy_or_touch(custom_setup_script, docker_build_dir / 'user-custom-setup')
    _copy_or_touch(custom_post_build_script, docker_build_dir / 'user-custom-post-build')

    setup_emulator(platform.qemu_arch, docker_build_dir)

    return docker_build_dir


def create_workspace_sysroot(
    docker_client: DockerClient,
    platform: Platform,
    ros_workspace: Path,
) -> None:
    """
    Create the target platform sysroot image.

    :param docker_client Docker client to use for building
    :param platform Information about the target platform
    :param build_context Directory containing all assets needed by sysroot.Dockerfile
    """
    image_tag = platform.sysroot_image_tag
    sysroot_destination = (ros_workspace / build_internals_dir(platform)).parent / 'sysroot'

    assert_install_rosdep_script_exists(ros_workspace, platform)
    logger.info('Building sysroot image: %s', image_tag)
    docker_client.build_image(
        dockerfile_name='sysroot.Dockerfile',
        tag=image_tag,
        buildargs={
            'BASE_IMAGE': platform.target_base_image,
            'ROS_VERSION': platform.ros_version,
            'DEPENDENCY_SCRIPT': 'install_rosdeps.sh',
        }
    )
    logger.info('Successfully created sysroot docker image: %s', image_tag)
    logger.info('Exporting sysroot')
    fs = docker_client.export_image_filesystem(image_tag)
    logger.info('Extracting sysroot to destination')
    try:
        shutil.rmtree(str(sysroot_destination))
    except FileNotFoundError:
        pass
    fs.extractall(sysroot_destination)
    fs.close()


class CreateSysroot(PipelineStage):
    """
    This stage creates the target platform Docker sysroot image.

    It will output the sysroot image.
    """

    def __init__(self):
        super().__init__('sysroot')

    def __call__(
        self,
        platform: Platform,
        docker_client: DockerClient,
        ros_workspace_dir: Path,
        options: PipelineStageOptions,
        data_collector: DataCollector
    ):
        create_workspace_sysroot_image(docker_client, platform)

        img_size = docker_client.get_image_size(platform.sysroot_image_tag)
        data_collector.add_size(self.name, img_size)
