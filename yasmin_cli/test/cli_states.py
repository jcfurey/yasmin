# Copyright (C) 2026
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

"""States loaded by the factory nodes in the CLI end-to-end tests."""

import os
import time

from yasmin import Blackboard, State


class WriteValue(State):
    """Appends the 'val' input key to the file named by CLI_TEST_OUTPUT."""

    def __init__(self) -> None:
        super().__init__(["done"])

    def execute(self, blackboard: Blackboard) -> str:
        with open(os.environ["CLI_TEST_OUTPUT"], "a") as output:
            output.write(f"{blackboard['val']}\n")
        return "done"


class Fail(State):
    def __init__(self) -> None:
        super().__init__(["done"])

    def execute(self, blackboard: Blackboard) -> str:
        raise RuntimeError("expected failure")


class WaitForCancel(State):
    """Signals readiness through CLI_TEST_OUTPUT, then waits to be canceled."""

    def __init__(self) -> None:
        super().__init__(["done"])

    def execute(self, blackboard: Blackboard) -> str:
        with open(os.environ["CLI_TEST_OUTPUT"], "a") as output:
            output.write("waiting\n")
        deadline = time.monotonic() + 30.0
        while not self.is_canceled() and time.monotonic() < deadline:
            time.sleep(0.05)
        return "done"
