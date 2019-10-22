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


"""
Looks for sphinx-build in lib/ and creates a symlink in bin/.

This workaround mitigates an issue in readthedocs with sphinx where the
sphinx-build binary is installed in a directory which is not listed in PATH,
causing the documentation generation to fail (failed to open sphinx-build).
"""

import os
import pathlib

from setuptools import setup


sphinx_build = next(pathlib.Path('/').glob('**/sphinx-build'))

# Looking for the venv bin directory. The home directory is
# chosen as a starting point, because it excludes system bin
# packages, and the venv is a subdirectory of the home dir.
bin_directory = next(pathlib.Path.home().glob('**/bin'))

try:
    os.symlink(sphinx_build, bin_directory / 'sphinx-build')
except FileExistsError:
    # readthedocs re-use venvs acrosss multiple builds, which means that,
    # sometimes, the sphinx-build will have been previously created.
    pass

package_name = 'sphinx_build_symlink'

setup(
    name=package_name,
    version='0.1.0',
)
