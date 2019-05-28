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

import logging
import sys
from pathlib import Path

from flake8 import LOG
from flake8.api.legacy import get_style_guide


# Avoid debug and info messages from flake8 internals
LOG.setLevel(logging.WARN)


def test_flake8():
    style_guide = get_style_guide(
        extend_ignore=['D100', 'D104'],
        show_source=True,
    )
    style_guide_tests = get_style_guide(
        extend_ignore=['D100', 'D101', 'D102', 'D103', 'D104', 'D105', 'D107'],
        show_source=True,
    )

    stdout = sys.stdout
    sys.stdout = sys.stderr

    report = style_guide.check_files([
        str(Path(__file__).parents[1] / 'cross_compile'),
    ])
    report_tests = style_guide_tests.check_files([
        str(Path(__file__).parents[1] / 'test'),
    ])
    sys.stdout = stdout

    total_errors = report.total_errors + report_tests.total_errors
    if total_errors:
        # Output summary with per-category counts
        print()
        if report.total_errors:
            report._application.formatter.show_statistics(report._stats)
        if report_tests.total_errors:
            report_tests._application.formatter.show_statistics(
                report_tests._stats)
        print(
            'flake8 reported {total_errors} errors'
            .format_map(locals()), file=sys.stderr)

    assert not report.total_errors, \
        'flake8 reported {total_errors} errors'.format_map(locals())
