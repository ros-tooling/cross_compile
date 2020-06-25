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

from ros_cross_compile.sysroot_creator import INTERNALS_DIR


Datum = NamedTuple('Datum', [('name', str),
                             ('value', Union[int, float]),
                             ('unit', str),
                             ('timestamp', str)])


class DataCollector:
    """Provides an interface to collect time series data."""

    def __init__(self):
        self.data = {}

    def add_datum(self, new_datum: Datum, new_datum_name: str):
        if new_datum_name not in self.data:
            self.data[new_datum_name] = [new_datum._asdict()]
        else:
            self.data[new_datum_name].append(new_datum._asdict())


class DataTimer:
    """
    Represents a stopwatch-style timer.

    This class is designed to wrap a function call using a with
    statement.
    """

    def __init__(self, name: str, data_collector: DataCollector):
        self.name = name
        self.data_collector = data_collector

    def __enter__(self):
        self._start = time.monotonic()
        return self

    def __exit__(self, *exc):
        self.elapsed = time.monotonic() - self._start
        time_metric = Datum(self.name + '-time', self.elapsed,
                            'seconds', str(datetime.now()))
        self.data_collector.add_datum(time_metric, self.name)


class DataWriter:
    """Provides an interface to write collected data to a file."""

    def __init__(self, ros_workspace_dir: Path,
                 output_file: Path = Path(datetime.now().strftime('%s') + '.json')):
        """Configure path for writing data."""
        self.write_path = Path(str(ros_workspace_dir)) / Path(INTERNALS_DIR) / Path('metrics')
        self.write_path.mkdir(parents=True, exist_ok=True)

        self.write_file = self.write_path / output_file

    def wrap_up(self, data_collector: DataCollector):
        with self.write_file.open('w') as f:
            json.dump(data_collector.data, f, sort_keys=True, indent=4)
