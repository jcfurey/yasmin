# Copyright (C) 2026 Miguel Ángel González Santamarta
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

from threading import Event
from time import monotonic
from typing import Callable, Optional, Set

from rclpy.node import Node

import yasmin
from yasmin_ros.basic_outcomes import CANCEL, TIMEOUT
from yasmin_ros.yasmin_node import YasminNode


def resolve_node(node: Optional[Node] = None) -> Node:
    if node is not None:
        return node
    return YasminNode.get_instance()


def setup_outcomes(
    outcomes: Optional[Set[str]],
    base_outcomes: Set[str],
    add_timeout: bool = False,
) -> Set[str]:
    outcomes = set(outcomes or ())
    outcomes.update(base_outcomes)
    if add_timeout:
        outcomes.add(TIMEOUT)
    return outcomes


def cancel_with_event(event: Event) -> None:
    event.set()


def wait_with_retry(
    condition_fn: Callable[[], bool],
    max_retry: int,
    log_msg: str,
    cancel_check: Optional[Callable[[], bool]] = None,
) -> Optional[str]:
    retry_count = 0
    while not condition_fn():
        if cancel_check is not None and cancel_check():
            return CANCEL
        yasmin.YASMIN_LOG_WARN(f"{log_msg}")
        if retry_count < max_retry:
            retry_count += 1
            yasmin.YASMIN_LOG_WARN(f"Retrying ({retry_count}/{max_retry})")
        else:
            return TIMEOUT
    return None


def wait_for_server_with_retry(
    wait_fn: Callable[[float], bool],
    timeout: Optional[float],
    max_retry: int,
    log_msg: str,
    cancel_check: Callable[[], bool],
) -> Optional[str]:
    """Wait in short slices, preserving the timeout budget for each retry."""

    def attempt():
        deadline = None if timeout is None else monotonic() + max(0.0, timeout)
        while not cancel_check():
            remaining = None if deadline is None else max(0.0, deadline - monotonic())
            if wait_fn(0.1 if remaining is None else min(0.1, remaining)):
                return True
            if deadline is not None and monotonic() >= deadline:
                return False
        return False

    return wait_with_retry(attempt, max_retry, log_msg, cancel_check)
