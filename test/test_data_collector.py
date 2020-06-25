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

import time

from ros_cross_compile.data_collector import DataCollector
from ros_cross_compile.data_collector import DataTimer
from ros_cross_compile.data_collector import DataWriter
from ros_cross_compile.data_collector import Datum


def test_datum_construction():
    test_datum = Datum('test_stat', 3, 'tests', '3 PM PST')
    assert test_datum


def test_collector_construction():
    test_collector = DataCollector()
    assert test_collector


def test_data_collection():
    test_collector = DataCollector()

    test_datum_a = Datum('test_stat_1', 3, 'tests', '2:30 PM PST')
    test_datum_b = Datum('test_stat_2', 4, 'tests', '2:45 PM PST')

    test_collector.add_datum(test_datum_a, 'test_a')
    test_collector.add_datum(test_datum_b, 'test_b')

    gold_data = {'test_a': [{'name': 'test_stat_1', 'value': 3,
                             'unit': 'tests', 'timestamp': '2:30 PM PST'}],
                 'test_b': [{'name': 'test_stat_2', 'value': 4,
                             'unit': 'tests', 'timestamp': '2:45 PM PST'}]}

    assert test_collector.data == gold_data


def test_data_timer():
    test_collector = DataCollector()
    with DataTimer('test_time', test_collector):
        time.sleep(2)

    assert test_collector.data['test_time'][0]['value'] > 0


def test_data_writing(tmp_path):
    test_collector = DataCollector()

    test_datum_a = Datum('test_stat_1', 3, 'tests', '2:30 PM PST')
    test_datum_b = Datum('test_stat_2', 4, 'tests', '2:45 PM PST')

    test_collector.add_datum(test_datum_a, 'test_a')
    test_collector.add_datum(test_datum_b, 'test_b')

    test_writer = DataWriter(tmp_path)

    test_writer.wrap_up(test_collector)

    assert test_writer.write_file.exists()
