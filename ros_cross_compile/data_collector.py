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

from contextlib import contextmanager
from datetime import datetime
from enum import Enum
import json
from pathlib import Path
import time
from typing import Dict, List, NamedTuple, Union


INTERNALS_DIR = 'cc_internals'


Datum = NamedTuple('Datum', [('name', str),
                             ('value', Union[int, float]),
                             ('unit', str),
                             ('timestamp', float),
                             ('complete', bool)])


class Units(Enum):
    Seconds = 'seconds'
    Bytes = 'bytes'


class DataCollector:
    """Provides an interface to collect time series data."""

    def __init__(self):
        self._data = []

    def add_datum(self, new_datum: Datum):
        self._data.append(new_datum)

    def serialize_data(self) -> List[Dict]:
        return list(map(lambda d: d._asdict(), self._data))

    @contextmanager
    def timer(self, name: str):
        """Provide an interface to time a statement's duration with a 'with'."""
        start = time.monotonic()
        complete = False
        try:
            yield
            complete = True
        finally:
            elapsed = time.monotonic() - start
            time_metric = Datum('{}-time'.format(name), elapsed,
                                Units.Seconds.value, time.monotonic(), complete)
            self.add_datum(time_metric)


class DataWriter:
    """Provides an interface to write collected data to a file."""

    def __init__(self, ros_workspace_dir: Path,
                 output_file: Path = Path(datetime.now().strftime('%s') + '.json')):
        """Configure path for writing data."""
        self._write_path = Path(str(ros_workspace_dir)) / Path(INTERNALS_DIR) / Path('metrics')
        self._write_path.mkdir(parents=True, exist_ok=True)
        self.write_file = self._write_path / output_file

    def write(self, data_collector: DataCollector):
        """
        Write collected datums to a file.

        Before writing, however, we convert each datum to a dictionary,
        so that they are conveniently 'dumpable' into a JSON file.
        """
        data_to_dump = data_collector.serialize_data()
        with self.write_file.open('w') as f:
            json.dump(list(data_to_dump), f, sort_keys=True, indent=4)
