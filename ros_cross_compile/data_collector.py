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
import json
from pathlib import Path
import time
from typing import NamedTuple, Union

from ros_cross_compile.sysroot_creator import INTERNALS_DIR


Datum = NamedTuple('Datum', [('name', str),
                             ('value', Union[int, float]),
                             ('unit', str),
                             ('timestamp', float),
                             ('complete', bool)])


class DataCollector:
    """Provides an interface to collect time series data."""

    def __init__(self):
        self.data = {}
        self.elapsed = 0

    def add_datum(self, new_datum: Datum, new_datum_name: str):
        if new_datum_name not in self.data:
            self.data[new_datum_name] = [new_datum._asdict()]
        else:
            self.data[new_datum_name].append(new_datum._asdict())

    @contextmanager
    def data_timer(self, name: str) -> int:
        start = time.monotonic()
        try:
            yield
        except:
            elapsed = time.monotonic() - start
            time_metric = Datum(name + '-time', elapsed,
                                'seconds', time.monotonic(), False)
            self.add_datum(time_metric, name)
        else:
            elapsed = time.monotonic() - start
            time_metric = Datum(name + '-time', elapsed,
                                'seconds', time.monotonic(), True)
            self.add_datum(time_metric, name)


class DataWriter:
    """Provides an interface to write collected data to a file."""

    def __init__(self, ros_workspace_dir: Path,
                 output_file: Path = Path(datetime.now().strftime('%s') + '.json')):
        """Configure path for writing data."""
        self.write_path = Path(str(ros_workspace_dir)) / Path(INTERNALS_DIR) / Path('metrics')
        self.write_path.mkdir(parents=True, exist_ok=True)

        self.write_file = self.write_path / output_file

    def write(self, data_collector: DataCollector):
        with self.write_file.open('w') as f:
            json.dump(data_collector.data, f, sort_keys=True, indent=4)
