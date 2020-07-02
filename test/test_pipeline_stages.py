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

from pathlib import Path

from ros_cross_compile.pipeline_stages import ConfigOptions


def test_config_tuple_creation():
    test_options = ConfigOptions(True, False, Path('./'), Path('./'), Path('./'))

    assert test_options

    assert test_options.skip_rosdep_collection is True
    assert test_options.skip_rosdep_keys is False
    assert test_options.custom_script == Path('./')
    assert test_options.custom_data_dir == Path('./')
    assert test_options.custom_setup_script == Path('./')
