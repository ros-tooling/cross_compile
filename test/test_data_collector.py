#!/usr/bin/env python

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

import errno
import os
from pathlib import Path
import time

from ros_cross_compile.data_collector import DataCollector
from ros_cross_compile.data_collector import DataTimer
from ros_cross_compile.data_collector import Datum


def test_construction():
    test_datum = Datum('test_stat', 3, 'tests', '3 PM PST')
    test_collector = DataCollector()
    assert test_datum
    assert test_collector


def test_data_collection():
    test_collector = DataCollector()
    test_collector.set_write_path(Path.cwd())

    test_datum_a = Datum('test_stat_1', 3, 'tests', '2:30 PM PST')
    test_datum_b = Datum('test_stat_2', 4, 'tests', '2:45 PM PST')

    test_collector.add_datum(test_datum_a)
    test_collector.add_datum(test_datum_b)

    test_collector.write_file()

    if Path(test_collector.write_path / test_collector.default_file).exists():
        os.remove(test_collector.default_file)
    else:
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT),
                                test_collector.default_file)


def test_data_timer():
    def test_timer_helper():
        time.sleep(3)
    with DataTimer() as test_timer:
        test_timer_helper()

    assert test_timer.elapsed > 3 and test_timer.elapsed < 4
