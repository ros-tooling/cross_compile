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

import json
from json import JSONDecodeError
from pathlib import Path

import pytest

from ros_cross_compile.data_collector import DataCollector
from ros_cross_compile.data_collector import DataWriter
from ros_cross_compile.data_collector import Datum


def test_datum_construction():
    test_datum = Datum('test_stat', 3, 'tests', 130.222, True)
    assert test_datum


def test_collector_construction():
    test_collector = DataCollector()
    assert test_collector


def test_data_collection():
    test_collector = DataCollector()

    test_datum_a = Datum('test_stat_1', 3, 'tests', 130.452, True)
    test_datum_b = Datum('test_stat_2', 4, 'tests', 130.455, True)

    test_collector.add_datum(test_datum_a)
    test_collector.add_datum(test_datum_b)

    to_test_data = test_collector._data

    assert to_test_data[0].name == 'test_stat_1'
    assert to_test_data[1].name == 'test_stat_2'
    assert to_test_data[0].value == 3
    assert to_test_data[0].unit == 'tests'
    assert abs(to_test_data[0].timestamp - 130.452) < 0.1
    assert to_test_data[0].complete


def test_timer_can_time():
    test_collector = DataCollector()
    with test_collector.timer('test_time'):
        pass

    assert test_collector._data[0].complete
    assert test_collector._data[0].value > 0


def test_timer_error_handling():
    test_collector = DataCollector()
    # The timer should not hide the exception, we expect it to add the datum value
    with pytest.raises(Exception):
        with test_collector.timer('test_time_fail'):
            raise Exception

    assert len(test_collector._data) > 0
    assert test_collector._data[0].complete is False


def test_data_writing(tmp_path):
    def load_json_validation(filename: Path) -> bool:
        try:
            with filename.open() as f:
                json.load(f)
                return True
        except JSONDecodeError:
            return False

    test_collector = DataCollector()

    test_datum_a = Datum('test_stat_1', 3, 'tests', 130.243, True)
    test_datum_b = Datum('test_stat_2', 4, 'tests', 130.244, True)

    test_collector.add_datum(test_datum_a)
    test_collector.add_datum(test_datum_b)

    test_writer = DataWriter(tmp_path, 'test.json')

    test_writer.write(test_collector)

    assert test_writer.write_file.exists()
    assert load_json_validation(test_writer.write_file)
