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
Fixtures that build fake ament prefixes for discovery tests.

Every test runs against its own prefixes in ``tmp_path`` with
``AMENT_PREFIX_PATH`` restricted to them (plus explicitly requested real
packages) and with all cache locations inside ``tmp_path``.
"""

import json
import os
import subprocess
import sys
import textwrap
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import pytest
from ament_index_python import PackageNotFoundError, get_package_prefix

PYTHON_LIB_DIR = os.path.join(
    "lib", f"python{sys.version_info.major}.{sys.version_info.minor}", "site-packages"
)
TEST_DIR = Path(__file__).resolve().parent


def _real_prefix(package: str) -> Optional[str]:
    try:
        return get_package_prefix(package)
    except (PackageNotFoundError, ValueError, KeyError):
        return None


# Resolved before any test changes AMENT_PREFIX_PATH.
REAL_PREFIXES: Dict[str, Optional[str]] = {
    name: _real_prefix(name) for name in ("yasmin", "yasmin_ros", "yasmin_factory")
}
ORIGINAL_PYTHONPATH = os.environ.get("PYTHONPATH", "")

STATE_TEMPLATE = """
from yasmin import State


class {class_name}(State):
    def __init__(self):
        super().__init__({outcomes!r})
        self.set_description("{class_name} description")

    def execute(self, blackboard):
        return {first_outcome!r}
"""


def state_source(class_name: str, outcomes: Iterable[str] = ("done",)) -> str:
    """Return the source of a module defining a constructible Python state."""
    outcomes = list(outcomes)
    return STATE_TEMPLATE.format(
        class_name=class_name, outcomes=outcomes, first_outcome=outcomes[0]
    )


def state_machine_xml(outcomes: str = "done", body: str = "") -> str:
    """Return a minimal XML state machine."""
    return (
        '<?xml version="1.0"?>\n'
        f'<StateMachine outcomes="{outcomes}">\n{body}</StateMachine>\n'
    )


class FakeWorkspace:
    """Builder for fake ament prefixes."""

    def __init__(self, root: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        self.root = root
        self.monkeypatch = monkeypatch
        self.prefixes: List[Path] = []
        self.cache_dir = root / "cache"
        self.real_packages: List[str] = []

    def prefix(self, relative: str = "ws/install") -> Path:
        path = self.root / relative
        path.mkdir(parents=True, exist_ok=True)
        if path not in self.prefixes:
            self.prefixes.append(path)
        return path

    def add_package(
        self,
        name: str,
        prefix: Optional[Path] = None,
        depends: Iterable[str] = (),
        export: str = "",
    ) -> Path:
        prefix = prefix or self.prefix()
        marker_dir = prefix / "share" / "ament_index" / "resource_index" / "packages"
        marker_dir.mkdir(parents=True, exist_ok=True)
        (marker_dir / name).write_text("")
        share = prefix / "share" / name
        share.mkdir(parents=True, exist_ok=True)
        depend_tags = "".join(f"<depend>{dep}</depend>" for dep in depends)
        (share / "package.xml").write_text(
            '<?xml version="1.0"?>\n'
            f'<package format="3"><name>{name}</name><version>0.0.0</version>'
            "<description>fake</description>"
            '<maintainer email="a@b.c">a</maintainer><license>Apache-2.0</license>'
            f"{depend_tags}<export><build_type>ament_python</build_type>{export}"
            "</export></package>\n"
        )
        return prefix

    def site_packages(self, prefix: Optional[Path] = None) -> Path:
        path = (prefix or self.prefix()) / PYTHON_LIB_DIR
        path.mkdir(parents=True, exist_ok=True)
        return path

    def add_python_file(
        self, relative: str, source: str, prefix: Optional[Path] = None
    ) -> Path:
        """Write ``site-packages/<relative>``; package directories get an ``__init__``."""
        path = self.site_packages(prefix) / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        current = path.parent
        while current != self.site_packages(prefix):
            init_file = current / "__init__.py"
            if not init_file.exists():
                init_file.write_text("")
            current = current.parent
        path.write_text(textwrap.dedent(source))
        return path

    def add_share_file(
        self, package: str, relative: str, content, prefix: Optional[Path] = None
    ) -> Path:
        path = (prefix or self.prefix()) / "share" / package / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        if isinstance(content, bytes):
            path.write_bytes(content)
        else:
            path.write_text(content)
        return path

    def add_plugin_description(
        self,
        package: str,
        relative: str,
        content: str,
        prefix: Optional[Path] = None,
        category: str = "yasmin",
    ) -> Path:
        prefix = prefix or self.prefix()
        path = self.add_share_file(package, relative, content, prefix)
        index_dir = (
            prefix
            / "share"
            / "ament_index"
            / "resource_index"
            / f"{category}__pluginlib__plugin"
        )
        index_dir.mkdir(parents=True, exist_ok=True)
        entry = index_dir / package
        existing = entry.read_text() if entry.exists() else ""
        entry.write_text(existing + f"share/{package}/{relative}\n")
        return path

    def use_real_packages(self, *names: str) -> None:
        for name in names:
            if not REAL_PREFIXES.get(name):
                pytest.skip(f"package {name} is not installed")
        self.real_packages = list(names)

    def activate(self) -> None:
        prefixes = [str(prefix) for prefix in self.prefixes]
        for name in self.real_packages:
            real_prefix = REAL_PREFIXES[name]
            if real_prefix not in prefixes:
                prefixes.append(real_prefix)
        self.monkeypatch.setenv("AMENT_PREFIX_PATH", os.pathsep.join(prefixes))

        python_paths = [str(prefix / PYTHON_LIB_DIR) for prefix in self.prefixes]
        # Like a sourced workspace: PYTHONPATH and this process' sys.path agree.
        for path in reversed(python_paths):
            if path not in sys.path:
                self.monkeypatch.syspath_prepend(path)
        if ORIGINAL_PYTHONPATH:
            python_paths.append(ORIGINAL_PYTHONPATH)
        self.monkeypatch.setenv("PYTHONPATH", os.pathsep.join(python_paths))

    def manager(self, **kwargs):
        from yasmin_plugins_manager.plugin_manager import PluginManager

        self.activate()
        kwargs.setdefault("cache_dir", self.cache_dir)
        return PluginManager(**kwargs)

    def discover(self, force_refresh: bool = False, **kwargs):
        manager = self.manager(**kwargs)
        manager.load_all_plugins(hide_progress=True, force_refresh=force_refresh)
        return manager

    def run_discovery_subprocess(
        self, timeout: float = 120.0, package_timeout_sec: Optional[float] = None
    ) -> dict:
        """
        Run discovery in a child process (see ``discovery_runner.py``).

        Used where a regression could hang or crash the test process itself.
        """
        self.activate()
        command = [
            sys.executable,
            str(TEST_DIR / "discovery_runner.py"),
            str(self.cache_dir),
        ]
        if package_timeout_sec is not None:
            command.append(str(package_timeout_sec))
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=timeout,
            env=dict(os.environ),
        )
        assert completed.returncode == 0, completed.stderr[-2000:]
        lines = [line for line in completed.stdout.splitlines() if line.startswith("{")]
        assert lines, completed.stdout[-2000:] + completed.stderr[-2000:]
        return json.loads(lines[-1])


@pytest.fixture(autouse=True)
def isolated_cache(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """Never touch the user's cache: point every cache location into tmp_path."""
    cache_dir = tmp_path / "default_cache"
    monkeypatch.setenv("YASMIN_CACHE", str(cache_dir))
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "xdg_cache"))
    monkeypatch.delenv("YASMIN_DISCOVERY_IGNORE_PACKAGES", raising=False)
    monkeypatch.delenv("YASMIN_DISCOVERY_PACKAGE_TIMEOUT", raising=False)
    return cache_dir


@pytest.fixture
def fake_ws(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> FakeWorkspace:
    return FakeWorkspace(tmp_path, monkeypatch)


def plugin_ids(plugins) -> List[str]:
    return sorted(plugin.unique_id for plugin in plugins)
