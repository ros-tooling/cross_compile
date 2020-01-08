#!/usr/bin/env python3
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

"""
Lint the index.yaml file as well as all files ending with .mixin.

Inspired by lint.py in https://github.com/colcon/colcon-mixin-repository.
"""

import os

import pkg_resources
import pytest
from yamllint.cli import run


@pytest.mark.linter
def test_colcon_mixins():
    any_error = False
    mixins_dir = pkg_resources.resource_filename('cross_compile', 'mixins')
    for name in sorted(os.listdir(mixins_dir)):
        if name != 'index.yaml' and not name.endswith('.mixin'):
            continue

        try:
            run([
                '--config-data',
                '{'
                'extends: default, '
                'rules: {'
                'document-start: {present: false}, '
                'empty-lines: {max: 0}, '
                'key-ordering: {}, '
                'line-length: {max: 999}'
                '}'
                '}',
                '--strict',
                os.path.join(mixins_dir, name),
            ])
        except SystemExit as e:
            any_error |= bool(e.code)
            continue

    assert not any_error, 'Should not have seen any errors'
