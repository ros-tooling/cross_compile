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

"""Classes to aid in the collection of benchmarking information."""

import json
from typing import Any


class DataMeasurement:
    """
    Represents a measurement that has been collected.

    Includes:
    * name of measurement
    * value of the measurement
    * units of said value
    """

    def __init__(self, name: str, value: Any, unit: str, timestamp: str):
        self.name = name
        self.value = value
        self.unit = unit
        self.timestamp = timestamp

    def __repr__(self) -> str:
        return '[{}] {}: {} {}'.format(self.timestamp, self.name, self.value, self.unit)


class DataCollector:
    """
    Collects time series metrics for output to a file.

    Includes:
    * A cleanup function in event of early exits
    * Functions to add both new data types and collected metrics
    * A function to write collected data to a JSON file
    """

    def __init__(self):
        # setup directory for saving metrics
        self._data = {'data': []}

    def add_datum(self, name: str, value: Any, unit, timestamp):  # add new collected metric
        new_measure = DataMeasurement(name, value, unit, timestamp)

        # for convenient JSON serializing, we use the __dict__ class
        self._data['data'].append(new_measure.__dict__)

    def write_JSON(self, write_path: str):  # at the end, we write our results to a file
        with open(write_path, 'w') as f:
            json.dump(self._data, f, sort_keys=True, indent=4)
