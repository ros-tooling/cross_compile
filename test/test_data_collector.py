#!/usr/bin/env python

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

"""Unit tests for data collector."""

from ros_cross_compile.data_collector import DataCollector
from ros_cross_compile.data_collector import DataMeasurement


def test_data_measurement_construction():
    dm = DataMeasurement('plane_mile_time', 5, 'seconds', '3:04')
    assert dm


def test_data_measurement_functions():
    dm = DataMeasurement('plane_mile_time', 5, 'seconds', '3:02')
    assert repr(dm) == '[3:02] plane_mile_time: 5 seconds'


def test_data_collector_construction():
    dc = DataCollector()
    assert dc


def test_data_collector_functions():
    dc = DataCollector()

    dc.add_datum('plane_speed', '20', 'm/s', '14:05')
    dc.add_datum('plane_os_size', '2000', 'kb', '12:30')
    dc.add_datum('plane_takeoff_time', '20', 'seconds', '13:45')

    dc.write_JSON('./temp.json')
