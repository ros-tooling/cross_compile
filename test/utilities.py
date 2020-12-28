# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

import functools
import platform

import pytest

from ros_cross_compile.pipeline_stages import PipelineStageOptions


DEFAULT_TEST_DISTRO = 'foxy'


def uses_docker(func):
    """Decorate test to be skipped on platforms that don't have Docker for testing."""
    NO_MAC_REASON = 'CI environment cannot install Docker on Mac OS hosts.'
    IS_MAC = platform.system() == 'Darwin'

    @functools.wraps(func)
    @pytest.mark.skipif(IS_MAC, reason=NO_MAC_REASON)
    def wrapper(*args, **kwargs):
        return func(*args, **kwargs)
    return wrapper


def default_pipeline_options() -> PipelineStageOptions:
    return PipelineStageOptions(
        skip_rosdep_keys=[],
        custom_script=None,
        custom_data_dir=None,
        custom_setup_script=None,
        runtime_tag=None)
