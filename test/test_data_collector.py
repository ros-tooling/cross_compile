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
from ros_cross_compile.data_collector import DataWriter
from ros_cross_compile.data_collector import Datum


def test_datum_construction():
    test_datum = Datum('test_stat', 3, 'tests', '3 PM PST', True)
    assert test_datum


def test_collector_construction():
    test_collector = DataCollector()
    assert test_collector


def test_data_collection():
    test_collector = DataCollector()

    test_datum_a = Datum('test_stat_1', 3, 'tests', 130.452, True)
    test_datum_b = Datum('test_stat_2', 4, 'tests', 130.455, True)

    test_collector.add_datum(test_datum_a, 'test_a')
    test_collector.add_datum(test_datum_b, 'test_b')

    to_test_data = test_collector.data
    expected_data = {'test_a': [{'name': 'test_stat_1', 'value': 3,
                                 'unit': 'tests', 'timestamp': 130.452, 'complete': True}],
                     'test_b': [{'name': 'test_stat_2', 'value': 4,
                                 'unit': 'tests', 'timestamp': 130.455, 'complete': True}]}

    assert 'test_a' in to_test_data.keys()
    assert 'test_b' in to_test_data.keys()
    assert expected_data['test_a'][0]['name'] == to_test_data['test_a'][0]['name']
    assert abs(expected_data['test_a'][0]['value']
               - to_test_data['test_a'][0]['value']) < 0.5
    assert expected_data['test_a'][0]['unit'] == to_test_data['test_a'][0]['unit']
    assert abs(expected_data['test_a'][0]['timestamp']
               - to_test_data['test_a'][0]['timestamp']) < 0.5
    assert expected_data['test_a'][0]['complete'] == to_test_data['test_a'][0]['complete']


def test_data_timer_can_time():
    test_collector = DataCollector()
    with test_collector.data_timer('test_time'):
        time.sleep(2)

    assert test_collector.data['test_time'][0]['complete'] is True
    assert test_collector.data['test_time'][0]['value'] > 0


def test_data_timer_error_handling():
    test_collector = DataCollector()
    with test_collector.data_timer('test_time_fail'):
        time.sleep(2)
        raise Exception

    assert test_collector.data['test_time_fail'][0]['complete'] is False


def test_data_writing(tmp_path):
    test_collector = DataCollector()

    test_datum_a = Datum('test_stat_1', 3, 'tests', '2:30 PM PST', True)
    test_datum_b = Datum('test_stat_2', 4, 'tests', '2:45 PM PST', True)

    test_collector.add_datum(test_datum_a, 'test_a')
    test_collector.add_datum(test_datum_b, 'test_b')

    test_writer = DataWriter(tmp_path)

    test_writer.write(test_collector)

    assert test_writer.write_file.exists()
