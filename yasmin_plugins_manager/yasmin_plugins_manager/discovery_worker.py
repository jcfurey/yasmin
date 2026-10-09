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

"""
Out-of-process plugin discovery.

Discovering Python and C++ plugins means importing arbitrary Python modules,
loading plugin libraries and constructing states. Any of these may print to
stdout, block forever, initialize ROS or crash the interpreter. They therefore
run in a worker process that handles one package per request:

* the parent sends one JSON task per line on the worker's stdin,
* the worker answers with JSON lines on its stdout: a ``scan`` message (files to
  track for cache invalidation), ``progress`` messages (the module or class
  being loaded) and finally a ``result`` message,
* the parent enforces a per-package timeout. When the worker times out or dies,
  the parent kills it, records the package as failed and starts a new worker for
  the next package.

The worker redirects its own stdin and stdout to ``/dev/null`` before importing
anything, so imported code can neither read the task stream nor corrupt the
result stream.
"""

import importlib
import importlib.util
import inspect
import json
import os
import queue
import signal
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from yasmin_plugins_manager.cache import recursive_dir_signature, stat_signature

PACKAGE_TIMEOUT_ENV_VAR = "YASMIN_DISCOVERY_PACKAGE_TIMEOUT"
DEFAULT_PACKAGE_TIMEOUT_SEC = 10.0
MIN_STARTUP_TIMEOUT_SEC = 30.0
MAX_PYTHON_PACKAGE_DEPTH = 3
SKIPPED_PYTHON_DIRS = {"__pycache__", "test", "tests"}
SKIPPED_PYTHON_FILES = {"__init__.py", "__main__.py"}
SKIPPED_PYTHON_PACKAGES = {
    "rosidl_adapter",
    "rosidl_cli",
    "rosidl_generator_c",
    "rosidl_generator_cpp",
    "rosidl_generator_py",
    "rosidl_parser",
    "rosidl_pycommon",
    "rosidl_runtime_py",
    "rosidl_typesupport_c",
    "rosidl_typesupport_cpp",
    "rosidl_typesupport_introspection_c",
    "rosidl_typesupport_introspection_cpp",
}

# Started with ``python -c`` so that the current directory (sys.path[0] == "") can
# be removed before anything is imported from it.
_WORKER_BOOTSTRAP = (
    "import sys\n"
    "if sys.path and sys.path[0] in ('', '.'):\n"
    "    del sys.path[0]\n"
    "from yasmin_plugins_manager.discovery_worker import main\n"
    "main()\n"
)


def get_package_timeout_sec(value: Optional[float] = None) -> float:
    """Return the per-package timeout from the argument, the environment or the default."""
    if value is None:
        raw_value = os.environ.get(PACKAGE_TIMEOUT_ENV_VAR, "")
        try:
            value = float(raw_value) if raw_value.strip() else DEFAULT_PACKAGE_TIMEOUT_SEC
        except ValueError:
            value = DEFAULT_PACKAGE_TIMEOUT_SEC

    if not value or value <= 0:
        return DEFAULT_PACKAGE_TIMEOUT_SEC
    return float(value)


def make_failure(
    package: str,
    kind: str,
    target: Optional[str],
    reason: str,
    relevant: bool,
) -> Dict[str, Any]:
    """Build one discovery failure record."""
    return {
        "package": package,
        "kind": kind,
        "target": target,
        "reason": reason,
        "relevant": bool(relevant),
    }


def describe_exception(exc: BaseException) -> str:
    """Return a one-line description of an exception raised by plugin code."""
    if isinstance(exc, SystemExit):
        return f"SystemExit({exc.code!r})"
    message = str(exc).strip().splitlines()
    return f"{type(exc).__name__}: {message[0]}" if message else type(exc).__name__


def is_state_constructible_without_arguments(state_class: type) -> bool:
    """Check whether ``state_class()`` is expected to work without arguments."""
    try:
        signature = inspect.signature(state_class)
    except (TypeError, ValueError):
        return False

    for parameter in signature.parameters.values():
        if parameter.kind in (
            inspect.Parameter.VAR_POSITIONAL,
            inspect.Parameter.VAR_KEYWORD,
        ):
            continue

        if parameter.default is inspect.Parameter.empty:
            return False

    return True


def _mentions_yasmin(file_path: Optional[str]) -> bool:
    """Return whether a source file references YASMIN (used to rate failures)."""
    if not file_path:
        return False
    try:
        with open(file_path, "rb") as handle:
            return b"yasmin" in handle.read()
    except OSError:
        return False


def scan_python_package(
    package_name: str,
) -> Optional[Tuple[List[Tuple[str, str]], List[dict], bool]]:
    """
    List the modules of a Python package without importing it.

    Returns
    -------
    Optional[Tuple[List[Tuple[str, str]], List[dict], bool]]
        ``(modules, tracked, is_package)`` where ``modules`` holds
        ``(module_name, file_path)`` pairs and ``tracked`` the signatures used
        for cache invalidation, or ``None`` if there is no such Python module.
    """
    if (
        package_name in SKIPPED_PYTHON_PACKAGES
        or package_name.startswith("_")
        or not package_name.isidentifier()
    ):
        return None

    try:
        spec = importlib.util.find_spec(package_name)
    except Exception:
        return None

    if spec is None:
        return None

    locations = spec.submodule_search_locations
    if locations is None:
        # A single-file module: scan the module itself, not its directory.
        origin = spec.origin
        if not origin or not os.path.isfile(origin):
            return None
        signature = stat_signature(origin)
        return [(package_name, origin)], [signature] if signature else [], False

    modules: List[Tuple[str, str]] = []
    tracked: List[dict] = []

    for location in list(locations):
        signature = recursive_dir_signature(location)
        if signature:
            tracked.append(signature)

        for root, dirs, files in os.walk(location):
            rel_dir = os.path.relpath(root, location)
            parts = [] if rel_dir == os.curdir else rel_dir.split(os.sep)

            if len(parts) >= MAX_PYTHON_PACKAGE_DEPTH:
                dirs[:] = []
            else:
                dirs[:] = sorted(
                    d for d in dirs if d not in SKIPPED_PYTHON_DIRS and d.isidentifier()
                )

            for file_name in sorted(files):
                if not file_name.endswith(".py") or file_name in SKIPPED_PYTHON_FILES:
                    continue
                stem = file_name[:-3]
                if not stem.isidentifier():
                    continue
                module_name = ".".join([package_name, *parts, stem])
                modules.append((module_name, os.path.join(root, file_name)))

    return modules, tracked, True


class _WorkerTaskRunner:
    """Worker-side handling of one package task."""

    def __init__(self, send: Callable[[dict], None]) -> None:
        self._send = send

    def run(self, task: dict) -> dict:
        task_id = task.get("id")
        package = task["package"]
        relevant_package = bool(task.get("relevant", False))
        result: Dict[str, Any] = {
            "type": "result",
            "id": task_id,
            "package": package,
            "cpp_plugins": [],
            "python_plugins": [],
            "failures": [],
        }

        scan = scan_python_package(package) if task.get("python") else None
        self._send(
            {
                "type": "scan",
                "id": task_id,
                "tracked": scan[1] if scan else [],
            }
        )

        for class_name in task.get("cpp_classes", []):
            self._load_cpp_plugin(task_id, package, class_name, result)

        if scan:
            modules, _, is_package = scan
            self._load_python_package(
                task_id, package, modules, is_package, relevant_package, result
            )

        return result

    def _progress(self, task_id: Any, target: str) -> None:
        self._send({"type": "progress", "id": task_id, "target": target})

    def _load_cpp_plugin(
        self, task_id: Any, package: str, class_name: str, result: dict
    ) -> None:
        from yasmin_plugins_manager.plugin_info import PluginInfo

        self._progress(task_id, class_name)
        try:
            plugin = PluginInfo(
                plugin_type="cpp", class_name=class_name, package_name=package
            )
        except KeyboardInterrupt:
            raise
        except BaseException as exc:
            result["failures"].append(
                make_failure(package, "cpp", class_name, describe_exception(exc), True)
            )
            return
        result["cpp_plugins"].append(plugin.to_cache_dict())

    def _import(self, task_id: Any, module_name: str) -> Any:
        self._progress(task_id, module_name)
        return importlib.import_module(module_name)

    def _load_python_package(
        self,
        task_id: Any,
        package: str,
        modules: List[Tuple[str, str]],
        is_package: bool,
        relevant_package: bool,
        result: dict,
    ) -> None:
        if is_package:
            try:
                self._import(task_id, package)
            except KeyboardInterrupt:
                raise
            except BaseException as exc:
                init_file = next((path for _, path in modules), None)
                try:
                    spec = importlib.util.find_spec(package)
                    init_file = spec.origin if spec is not None else init_file
                except Exception:
                    pass
                result["failures"].append(
                    make_failure(
                        package,
                        "python",
                        package,
                        describe_exception(exc),
                        relevant_package or _mentions_yasmin(init_file),
                    )
                )
                return

        for module_name, file_path in modules:
            try:
                module = self._import(task_id, module_name)
                members = inspect.getmembers(module, inspect.isclass)
            except KeyboardInterrupt:
                raise
            except BaseException as exc:
                result["failures"].append(
                    make_failure(
                        package,
                        "python",
                        module_name,
                        describe_exception(exc),
                        relevant_package or _mentions_yasmin(file_path),
                    )
                )
                continue

            for class_name, state_class in members:
                if self._is_discoverable_state(module_name, state_class):
                    self._load_python_plugin(
                        task_id, package, module_name, class_name, result
                    )

    @staticmethod
    def _is_discoverable_state(module_name: str, state_class: type) -> bool:
        from yasmin import State

        try:
            return (
                state_class.__module__ == module_name
                and issubclass(state_class, State)
                and state_class is not State
                and is_state_constructible_without_arguments(state_class)
            )
        except Exception:
            return False

    def _load_python_plugin(
        self,
        task_id: Any,
        package: str,
        module_name: str,
        class_name: str,
        result: dict,
    ) -> None:
        from yasmin_plugins_manager.plugin_info import PluginInfo

        target = f"{module_name}.{class_name}"
        self._progress(task_id, target)
        try:
            plugin = PluginInfo(
                plugin_type="python",
                class_name=class_name,
                module=module_name,
                package_name=package,
            )
        except KeyboardInterrupt:
            raise
        except BaseException as exc:
            result["failures"].append(
                make_failure(package, "python", target, describe_exception(exc), True)
            )
            return
        result["python_plugins"].append(plugin.to_cache_dict())


def _isolate_stdio() -> Tuple[Any, int]:
    """Keep private copies of stdin/stdout for the protocol; hide the originals."""
    protocol_in = os.fdopen(os.dup(0), "r", encoding="utf-8")
    protocol_out = os.dup(1)

    devnull = os.open(os.devnull, os.O_RDWR)
    os.dup2(devnull, 0)
    os.dup2(devnull, 1)
    os.close(devnull)
    sys.stdin = open(os.devnull, "r", encoding="utf-8")
    sys.stdout = open(os.devnull, "w", encoding="utf-8")
    return protocol_in, protocol_out


def _shutdown_and_exit(code: int) -> None:
    """Exit immediately, giving ROS a bounded chance to shut down first."""
    rclpy = sys.modules.get("rclpy")
    if rclpy is not None:
        watchdog = threading.Timer(2.0, os._exit, args=(code,))
        watchdog.daemon = True
        watchdog.start()
        try:
            rclpy.try_shutdown()
        except BaseException:
            pass
    # Plugin code may have started non-daemon threads or atexit handlers that
    # never return, so skip the regular interpreter shutdown.
    os._exit(code)


def main() -> None:
    """Run the discovery worker loop."""
    protocol_in, protocol_out = _isolate_stdio()

    def send(message: dict) -> None:
        data = (json.dumps(message, default=str) + "\n").encode("utf-8")
        while data:
            written = os.write(protocol_out, data)
            data = data[written:]

    try:
        from yasmin import LogLevel, set_log_level

        set_log_level(LogLevel.WARN)
    except Exception:
        pass

    runner = _WorkerTaskRunner(send)
    send({"type": "ready", "pid": os.getpid()})

    for line in protocol_in:
        try:
            message = json.loads(line)
        except ValueError:
            continue

        if not isinstance(message, dict) or message.get("type") == "exit":
            break
        if message.get("type") == "task":
            try:
                result = runner.run(message)
            except Exception as exc:
                package = str(message.get("package"))
                result = {
                    "type": "result",
                    "id": message.get("id"),
                    "package": package,
                    "cpp_plugins": [],
                    "python_plugins": [],
                    "failures": [
                        make_failure(
                            package, "package", None, describe_exception(exc), True
                        )
                    ],
                }
            send(result)

    _shutdown_and_exit(0)


class WorkerStartError(RuntimeError):
    """Raised when the discovery worker cannot be started."""


class _WorkerExited(Exception):
    pass


def _peek_exit_code(proc: subprocess.Popen, timeout_sec: float) -> Optional[int]:
    """
    Wait up to ``timeout_sec`` for a process to exit without reaping it.

    The process stays a zombie until ``Popen.wait()``, so its process group id
    cannot be reused while the group is killed. Negative codes are signals.
    """
    deadline = time.monotonic() + timeout_sec
    while True:
        if hasattr(os, "waitid"):
            try:
                info = os.waitid(os.P_PID, proc.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
            except ChildProcessError:
                return proc.returncode
            if info is not None:
                if info.si_code == os.CLD_EXITED:
                    return info.si_status
                return -info.si_status
        else:
            code = proc.poll()
            if code is not None:
                return code

        if time.monotonic() >= deadline:
            return None
        time.sleep(0.02)


class DiscoveryWorkerClient:
    """Parent-side handle of one discovery worker process."""

    def __init__(self, startup_timeout_sec: float) -> None:
        self._startup_timeout_sec = startup_timeout_sec
        self._proc: Optional[subprocess.Popen] = None
        self._messages: "queue.Queue[Optional[bytes]]" = queue.Queue()
        self._stderr: Any = None
        self._next_task_id = 0

    def start(self) -> None:
        """Start the worker and wait until it is ready."""
        self._stderr = tempfile.TemporaryFile()
        env = dict(os.environ)
        # Nodes accidentally created by plugin code must not join the ROS graph.
        env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "OFF"

        popen_kwargs: Dict[str, Any] = {
            "stdin": subprocess.PIPE,
            "stdout": subprocess.PIPE,
            "stderr": self._stderr,
            "env": env,
            "close_fds": True,
        }
        if os.name == "posix":
            # Own process group: kills also reach processes spawned by plugin code.
            popen_kwargs["start_new_session"] = True

        try:
            self._proc = subprocess.Popen(
                [sys.executable, "-c", _WORKER_BOOTSTRAP], **popen_kwargs
            )
        except OSError as exc:
            self.close()
            raise WorkerStartError(f"could not start discovery worker: {exc}") from exc

        reader = threading.Thread(
            target=self._read_stdout,
            args=(self._proc.stdout, self._messages),
            name="yasmin_discovery_worker_reader",
            daemon=True,
        )
        reader.start()

        deadline = time.monotonic() + self._startup_timeout_sec
        try:
            message = self._next_message(deadline)
        except TimeoutError:
            self.close()
            raise WorkerStartError(
                f"discovery worker did not start within {self._startup_timeout_sec:g} s"
            )
        except _WorkerExited:
            reason = self._describe_exit(0)
            self.close()
            raise WorkerStartError(f"discovery worker failed to start ({reason})")

        if message.get("type") != "ready":
            self.close()
            raise WorkerStartError("discovery worker sent an unexpected first message")

    @staticmethod
    def _read_stdout(stream: Any, messages: "queue.Queue[Optional[bytes]]") -> None:
        try:
            for line in stream:
                messages.put(line)
        except (OSError, ValueError):
            pass
        finally:
            messages.put(None)

    def _next_message(self, deadline: float) -> dict:
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError()
            try:
                line = self._messages.get(timeout=remaining)
            except queue.Empty:
                raise TimeoutError()
            if line is None:
                raise _WorkerExited()
            try:
                message = json.loads(line)
            except ValueError:
                continue
            if isinstance(message, dict):
                return message

    def _stderr_offset(self) -> int:
        try:
            return os.fstat(self._stderr.fileno()).st_size
        except (OSError, AttributeError, ValueError):
            return 0

    def _stderr_tail(self, offset: int, max_lines: int = 5, max_chars: int = 600) -> str:
        try:
            self._stderr.seek(offset)
            data = self._stderr.read().decode("utf-8", errors="replace")
        except (OSError, AttributeError, ValueError):
            return ""
        lines = [line.strip() for line in data.splitlines() if line.strip()]
        tail = " | ".join(lines[-max_lines:])
        return tail[-max_chars:]

    def _describe_exit(self, stderr_offset: int) -> str:
        code = _peek_exit_code(self._proc, 5.0) if self._proc is not None else None

        if code is None:
            description = "worker stopped responding"
        elif code < 0:
            try:
                description = f"worker killed by {signal.Signals(-code).name}"
            except ValueError:
                description = f"worker killed by signal {-code}"
        else:
            description = f"worker exited with code {code}"

        tail = self._stderr_tail(stderr_offset)
        return f"{description}; stderr: {tail}" if tail else description

    def run_task(
        self, task: dict, timeout_sec: float
    ) -> Tuple[Optional[dict], List[dict], str]:
        """
        Send one package task and wait for its result.

        Returns
        -------
        Tuple[Optional[dict], List[dict], str]
            ``(result, tracked, error)``. ``result`` is ``None`` when the worker
            timed out or died; the worker is then closed and ``error`` explains why.
        """
        self._next_task_id += 1
        task = dict(task, type="task", id=self._next_task_id)
        tracked: List[dict] = []
        last_target: Optional[str] = None
        stderr_offset = self._stderr_offset()
        deadline = time.monotonic() + timeout_sec

        try:
            if self._proc is None or self._proc.stdin is None:
                raise _WorkerExited()
            self._proc.stdin.write((json.dumps(task) + "\n").encode("utf-8"))
            self._proc.stdin.flush()
            while True:
                message = self._next_message(deadline)
                if message.get("id") != task["id"]:
                    continue
                message_type = message.get("type")
                if message_type == "scan":
                    tracked.extend(
                        item
                        for item in message.get("tracked", [])
                        if isinstance(item, dict)
                    )
                elif message_type == "progress":
                    last_target = message.get("target")
                elif message_type == "result":
                    return message, tracked, ""
        except TimeoutError:
            error = f"timed out after {timeout_sec:g} s"
            self.close()
        except (_WorkerExited, OSError, ValueError):
            error = f"discovery worker crashed ({self._describe_exit(stderr_offset)})"
            self.close()

        if last_target:
            error += f" while loading {last_target}"
        return None, tracked, error

    def close(self, grace_sec: float = 0.0) -> None:
        """Stop the worker and every process it started."""
        proc, self._proc = self._proc, None
        if proc is not None:
            try:
                if grace_sec > 0 and proc.stdin is not None:
                    proc.stdin.write(b'{"type": "exit"}\n')
                    proc.stdin.flush()
            except (OSError, ValueError):
                pass
            try:
                if proc.stdin is not None:
                    proc.stdin.close()
            except (OSError, ValueError):
                pass

            if grace_sec > 0:
                _peek_exit_code(proc, grace_sec)

            if os.name == "posix":
                # The worker is not reaped yet, so its process group id is still
                # reserved; this also stops processes spawned by plugin code.
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
            elif proc.poll() is None:
                proc.kill()

            try:
                proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                pass
            try:
                if proc.stdout is not None:
                    proc.stdout.close()
            except (OSError, ValueError):
                pass

        if self._stderr is not None:
            try:
                self._stderr.close()
            except OSError:
                pass
            self._stderr = None


class DiscoveryWorkerSession:
    """
    Run package tasks in worker processes, restarting a worker after a failure.

    A package whose task times out or crashes the worker is reported as failed;
    the next package is processed by a fresh worker.
    """

    MAX_START_ATTEMPTS = 2

    def __init__(
        self,
        package_timeout_sec: Optional[float] = None,
        startup_timeout_sec: Optional[float] = None,
    ) -> None:
        self.package_timeout_sec = get_package_timeout_sec(package_timeout_sec)
        self.startup_timeout_sec = startup_timeout_sec or max(
            MIN_STARTUP_TIMEOUT_SEC, self.package_timeout_sec
        )
        self._client: Optional[DiscoveryWorkerClient] = None
        self._unavailable_reason: Optional[str] = None
        self._unavailable_reported = False

    def __enter__(self) -> "DiscoveryWorkerSession":
        return self

    def __exit__(self, *exc_info: Any) -> None:
        self.close()

    def _ensure_client(self) -> Optional[DiscoveryWorkerClient]:
        if self._client is not None:
            return self._client
        if self._unavailable_reason is not None:
            return None

        last_error = ""
        for _ in range(self.MAX_START_ATTEMPTS):
            client = DiscoveryWorkerClient(self.startup_timeout_sec)
            try:
                client.start()
            except WorkerStartError as exc:
                last_error = str(exc)
                continue
            self._client = client
            return client

        self._unavailable_reason = last_error
        return None

    def run(self, task: dict) -> Tuple[Optional[dict], List[dict], List[dict]]:
        """
        Run one package task.

        Returns
        -------
        Tuple[Optional[dict], List[dict], List[dict]]
            ``(result, tracked, worker_failures)``.
        """
        package = task["package"]
        client = self._ensure_client()
        if client is None:
            # Warn once; the remaining packages are recorded without a warning.
            failure = make_failure(
                package,
                "package",
                None,
                f"discovery worker unavailable: {self._unavailable_reason}",
                not self._unavailable_reported,
            )
            self._unavailable_reported = True
            return None, [], [failure]

        result, tracked, error = client.run_task(task, self.package_timeout_sec)
        if result is not None:
            return result, tracked, []

        self._client = None
        return None, tracked, [make_failure(package, "package", None, error, True)]

    def close(self) -> None:
        """Stop the current worker, if any."""
        if self._client is not None:
            self._client.close(grace_sec=2.0)
            self._client = None


if __name__ == "__main__":
    main()
