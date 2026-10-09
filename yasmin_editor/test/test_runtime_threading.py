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

"""Runtime controller threading, cancellation and status regressions.

Blocking scenarios run in helper threads with a join timeout, so a regression
fails the test instead of hanging the test process.
"""

import importlib
import sys
import threading
import time
import types

import pytest

pytest.importorskip("yasmin_editor.qt_compat")

from yasmin_editor.qt_compat import QtCore  # noqa: E402

DIRECT = QtCore.Qt.ConnectionType.DirectConnection
JOIN_TIMEOUT = 3.0


class RuntimeStateMachine:
    """Stand-in for ``yasmin.StateMachine`` recording cancel requests."""

    def __init__(self) -> None:
        self.cancel_state_threads = []
        self.cancel_sm_threads = []

    def add_start_cb(self, *_args, **_kwargs):
        return None

    def add_transition_cb(self, *_args, **_kwargs):
        return None

    def add_end_cb(self, *_args, **_kwargs):
        return None

    def cancel_state(self):
        self.cancel_state_threads.append(threading.current_thread())

    def cancel_state_machine(self):
        self.cancel_sm_threads.append(threading.current_thread())

    def __call__(self, _blackboard):
        return "done"


class Blackboard:
    def __init__(self, other=None) -> None:
        self._data = {} if other is None else other._data


@pytest.fixture
def runtime_module(monkeypatch):
    yasmin_module = types.ModuleType("yasmin")
    yasmin_module.Blackboard = Blackboard
    yasmin_module.StateMachine = RuntimeStateMachine
    yasmin_module.LogLevel = types.SimpleNamespace(
        ERROR="ERROR", WARN="WARN", INFO="INFO", DEBUG="DEBUG"
    )
    yasmin_module.set_loggers = lambda *_args, **_kwargs: None
    yasmin_module.set_log_level = lambda *_args, **_kwargs: None
    yasmin_module.log_level_to_name = lambda level: str(level)
    factory_module = types.ModuleType("yasmin_factory")
    factory_module.YasminFactory = lambda: types.SimpleNamespace(
        create_sm_from_file=lambda _path: RuntimeStateMachine()
    )
    monkeypatch.setitem(sys.modules, "yasmin", yasmin_module)
    monkeypatch.setitem(sys.modules, "yasmin_factory", factory_module)
    for name in ("yasmin_editor.runtime.runtime", "yasmin_editor.runtime.logging"):
        monkeypatch.delitem(sys.modules, name, raising=False)
    return importlib.import_module("yasmin_editor.runtime.runtime")


@pytest.fixture
def runtime(runtime_module):
    instance = runtime_module.Runtime()
    instance.sm = RuntimeStateMachine()
    yield instance
    instance._disposed = True


def run_with_timeout(target, *args):
    thread = threading.Thread(target=target, args=args, daemon=True)
    thread.start()
    thread.join(JOIN_TIMEOUT)
    return thread


def wait_until(predicate, timeout=JOIN_TIMEOUT):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def join_cancel_threads(runtime):
    for thread in list(runtime._cancel_threads):
        thread.join(JOIN_TIMEOUT)


def test_resume_does_not_emit_while_holding_the_state_lock(runtime):
    # E06: _resume() emitted blocked_changed while holding _worker_state_lock;
    # the GUI slot reads runtime state (same lock) synchronously -> deadlock.
    observed = []
    runtime.blocked_changed.connect(
        lambda blocked: observed.append(
            (blocked, runtime.is_blocked(), runtime.get_status_label())
        ),
        type=DIRECT,
    )
    runtime._running = True
    runtime._blocked = True

    thread = run_with_timeout(runtime._resume, False)

    assert not thread.is_alive(), "resume deadlocked on the worker-state lock"
    assert observed == [(False, False, "Running")]


def test_cancel_state_while_paused_does_not_call_into_yasmin(runtime):
    # E07: yasmin's cancel_state() waits for a current state that cannot appear
    # while the worker is paused in a callback; it must not be called then.
    statuses = []
    runtime.status_changed.connect(statuses.append, type=DIRECT)
    runtime._running = True
    runtime._blocked = True

    thread = run_with_timeout(runtime.cancel_state)

    assert not thread.is_alive()
    assert runtime.sm.cancel_state_threads == []
    assert "paused" in statuses[-1]


def test_cancel_state_runs_off_the_calling_thread(runtime):
    # E07: blocking yasmin cancel calls must never run on the GUI thread.
    runtime._running = True

    runtime.cancel_state()
    join_cancel_threads(runtime)

    assert len(runtime.sm.cancel_state_threads) == 1
    assert runtime.sm.cancel_state_threads[0] is not threading.current_thread()


def test_cancel_state_machine_releases_a_paused_worker(runtime):
    # E13: cancel_state_machine() set the yasmin flag but left the worker paused,
    # so the run never ended and runtime mode could not be left.
    runtime._running = True
    runtime._request_pause("Paused at breakpoint: B")
    worker = threading.Thread(target=runtime._pause_if_requested, daemon=True)
    worker.start()
    assert wait_until(runtime.is_blocked)

    runtime.cancel_state_machine()
    worker.join(JOIN_TIMEOUT)

    assert not worker.is_alive(), "paused worker was not released by the cancel"
    assert len(runtime.sm.cancel_sm_threads) == 1
    assert runtime.sm.cancel_sm_threads[0] is not threading.current_thread()

    # Breakpoints reached while the cancel propagates must not pause again.
    runtime._request_pause("Paused at breakpoint: C")
    assert not run_with_timeout(runtime._pause_if_requested).is_alive()


def test_failed_run_reports_failed_instead_of_a_blank_finish(runtime):
    # E16: yasmin calls the end callbacks with "" when the run raises; that was
    # reported as "Runtime finished with outcome: " and a "Ready" badge.
    errors = []
    runtime.error_occurred.connect(errors.append, type=DIRECT)
    runtime._running = True

    runtime.end_cb(None, "", tuple())
    assert not runtime.is_finished()

    class FailingStateMachine(RuntimeStateMachine):
        def __call__(self, _blackboard):
            raise RuntimeError("boom")

    runtime.sm = FailingStateMachine()
    runtime._execute_worker()

    assert runtime.is_finished()
    assert runtime.get_status_label() == "Failed"
    assert errors and "boom" in errors[-1]


def test_runtime_log_buffer_is_bounded(runtime_module):
    # E15: every log line was kept forever on looping machines.
    logging_module = sys.modules["yasmin_editor.runtime.logging"]
    limit = getattr(logging_module, "MAX_LOG_ENTRIES", 5000)
    logger = logging_module.RuntimeLogger(
        append_callback=lambda _message: None,
        clear_callback=lambda: None,
        is_disposed=lambda: False,
    )

    for index in range(limit + 50):
        logger.append(f"line {index}")

    logs = logger.get_logs()
    assert len(logs) == limit
    assert logs[-1] == f"line {limit + 49}"


def test_shell_command_repr_has_no_side_effects():
    # E14: repr()/str() executed the command, so printing the namespace ran
    # next/step/cont/... and canceled the machine.
    from yasmin_editor.runtime import interactive_shell

    calls = []
    command = interactive_shell._RuntimeShellCommand(
        "cancel_sm", lambda: calls.append("cancel_sm"), "Cancel the machine."
    )

    repr(command)
    str(command)
    repr({"cancel_sm": command})
    assert calls == []
    assert command() == "cancel_sm executed"
    assert calls == ["cancel_sm"]

    rewrite = interactive_shell.rewrite_bare_shell_commands
    assert rewrite(["next\n"]) == ["next()\n"]
    assert rewrite(["  cancel_sm  \n"]) == ["cancel_sm()\n"]
    assert rewrite(["next(it)\n"]) == ["next(it)\n"]
    assert rewrite(["x = 1\n", "next\n"]) == ["x = 1\n", "next\n"]
