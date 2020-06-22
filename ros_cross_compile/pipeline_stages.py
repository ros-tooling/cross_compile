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

"""Wrapper classes for each pipeline stage."""

import time
from datetime import datetime

from ros_cross_compile.data_collector import data_collector


class pipeline_stage():
    """
    Represents a stage of the cross compilation pipeline.

    Includes:
    * A getter for the name of the stage
    * A __call__ option to call the underlying function
    """

    def __init__(self, stage_name, func):
        self.stage_name = stage_name
        self.func = func

    def get_name(self):
        return self.stage_name

    def __call__(self, params, data_collector):
        # start timing of current pipeline stage
        start = time.time()

        # The parameters to the function are passed in as a tuple,
        #  then unpacked into positional arguments for the function
        self.func(*params)

        # end timing and save to data collector
        total = time.time() - start
        today = datetime.now()

        data_collector.add_datum(self.get_name() + '-time', total, "seconds", str(today))
