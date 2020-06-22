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


class data_measurement:
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

    def __repr__(self):
        return '[{}] {}: {} {}'.format(self.timestamp, self.name, self.value, self.unit)

    def get_name(self):
        return self.name

    def get_unit(self):
        return self.unit

    def get_value(self):
        return self.value

    def get_timestamp(self):
        return self.timestamp


class data_collector:
    """
    Represents the tool that will store and write data to a given file.

    Includes:
    * A cleanup function in event of early exits
    * Functions to add both new data types and collected metrics
    * A functioon to write collected data to a JSON file
    """

    def __init__(self):
        self.datums = {'data': []}
        self.datum_types = [('filesize', 'KB'), ('time', 'seconds')]

    def cleanup(self, status_code):  # for now just a placeholder
        pass

    # this isn't strictly necessary, could be deleted
    def add_data_type(self, data_type, unit):
        add = (data_type, unit)
        self.datum_types.append(add)

    def add_datum(self, name, value, unit, timestamp):  # add new collected metric
        new_measure = data_measurement(name, value, unit, timestamp)

        # for convenient JSON serializing, we use the __dict__ class
        self.datums['data'].append(new_measure.__dict__)

    def write_JSON(self, fname):  # at the end, we write our results to a file
        with open(fname, 'w') as f:
            json.dump(self.datums, f, sort_keys=True, indent=4)
