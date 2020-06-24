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

"""Classes for time series data collection and writing said data to a file."""

from datetime import datetime
import json
from pathlib import Path
import time
from typing import NamedTuple, Union


class Datum(NamedTuple):
    """Represents a collected piece of data."""

    name: str
    value: Union[int, float]
    unit: str
    timestamp: str


class DataCollector:
    """Provides an interface to collect time series data."""

    default_file = Path(datetime.now().strftime('%s') + '.json')

    def __init__(self):
        self._data = []

    def add_datum(self, new_datum: Datum):
        self._data.append(new_datum._asdict())

    def set_write_path(self, write_path: Path):
        self.write_path = write_path

    def write_file(self):
        with open(self.write_path / DataCollector.default_file, 'w') as f:
            json.dump(self._data, f, sort_keys=True, indent=4)


class DataTimer:
    """
    Represents a stopwatch-style timer.

    This class is designed to wrap a function call using a with
    statement.
    """

    def __enter__(self):
        self._start = time.monotonic()
        return self

    def __exit__(self, *exc):
        self.elapsed = time.monotonic() - self._start
