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

import json
import os
import subprocess
import sys
import textwrap
import time
from pathlib import Path

import pytest

from conftest import plugin_ids, state_machine_xml, state_source
from yasmin_plugins_manager import cache as cache_module


def _touch_newer(path: Path, content) -> None:
    """Rewrite a file and make sure its mtime changes even on coarse filesystems."""
    if isinstance(content, bytes):
        path.write_bytes(content)
    else:
        path.write_text(content)
    stat_result = path.stat()
    os.utime(path, ns=(stat_result.st_atime_ns, stat_result.st_mtime_ns + 10**9))


def _forbid_worker(monkeypatch) -> None:
    from yasmin_plugins_manager import discovery_worker

    def fail(*args, **kwargs):
        raise AssertionError("the discovery worker must not run for a valid cache")

    monkeypatch.setattr(discovery_worker.DiscoveryWorkerSession, "run", fail)


def test_warm_load_uses_cache_without_worker(fake_ws, monkeypatch):
    fake_ws.add_package("fake_cached")
    fake_ws.add_python_file("fake_cached/states.py", state_source("CachedState"))
    fake_ws.add_share_file("fake_cached", "sm/main.xml", state_machine_xml())

    cold = fake_ws.discover()
    _forbid_worker(monkeypatch)
    warm = fake_ws.discover()

    assert plugin_ids(warm.python_plugins) == plugin_ids(cold.python_plugins)
    assert plugin_ids(warm.xml_files) == ["xml:fake_cached:sm/main.xml"]
    assert warm.python_plugins[0].package_name == "fake_cached"


def test_new_xml_in_share_subdirectory_invalidates_cache(fake_ws):
    # M05: only the top-level share directory used to be tracked.
    fake_ws.add_package("fake_sm")
    fake_ws.add_share_file("fake_sm", "sm/a.xml", state_machine_xml())
    assert plugin_ids(fake_ws.discover().xml_files) == ["xml:fake_sm:sm/a.xml"]

    fake_ws.add_share_file("fake_sm", "sm/b.xml", state_machine_xml())

    assert plugin_ids(fake_ws.discover().xml_files) == [
        "xml:fake_sm:sm/a.xml",
        "xml:fake_sm:sm/b.xml",
    ]


def test_fixing_a_broken_package_invalidates_cache(fake_ws):
    # M05: a package whose import failed was not tracked at all.
    fake_ws.add_package("fake_init")
    fake_ws.add_python_file("fake_init/states.py", state_source("InitState"))
    init_file = fake_ws.site_packages() / "fake_init" / "__init__.py"
    _touch_newer(init_file, "raise RuntimeError('bug in __init__')\n")
    assert fake_ws.discover().python_plugins == []

    _touch_newer(init_file, "# fixed\n")

    assert plugin_ids(fake_ws.discover().python_plugins) == [
        "python:fake_init.states:InitState"
    ]


def test_rebuilt_plugin_library_invalidates_cache(fake_ws):
    # M05: C++ metadata comes from the library, which was not tracked.
    fake_ws.add_package("fake_lib")
    fake_ws.add_plugin_description(
        "fake_lib",
        "plugins.xml",
        '<library path="fake_lib_states"><class name="fake_lib/S" type="S"'
        ' base_class_type="yasmin::State"/></library>',
    )
    library = fake_ws.prefix() / "lib" / "libfake_lib_states.so"
    library.parent.mkdir(parents=True, exist_ok=True)
    library.write_bytes(b"version 1")

    fake_ws.discover()
    assert fake_ws.manager()._load_from_cache() is True

    _touch_newer(library, b"version 2, rebuilt")

    assert fake_ws.manager()._load_from_cache() is False


def test_appearing_plugin_library_invalidates_cache(fake_ws):
    fake_ws.add_package("fake_nolib")
    fake_ws.add_plugin_description(
        "fake_nolib",
        "plugins.xml",
        '<library path="fake_nolib_states"><class name="fake_nolib/S" type="S"'
        ' base_class_type="yasmin::State"/></library>',
    )
    fake_ws.discover()
    assert fake_ws.manager()._load_from_cache() is True

    library = fake_ws.prefix() / "lib" / "libfake_nolib_states.so"
    library.parent.mkdir(parents=True, exist_ok=True)
    library.write_bytes(b"built later")

    assert fake_ws.manager()._load_from_cache() is False


@pytest.mark.parametrize(
    "content",
    [
        "[]",
        '"text"',
        "null",
        '{"tracked_files": [{"mtime_ns": 1}]}',
        '{"cpp_plugins": "not a list"}',
        '{"tracked_dirs": [42]}',
        "{truncated",
    ],
)
def test_malformed_cache_triggers_rediscovery(fake_ws, content):
    # M08: valid JSON with an unexpected shape used to raise.
    fake_ws.add_package("fake_shape")
    fake_ws.add_python_file("fake_shape/states.py", state_source("ShapeState"))
    fake_ws.activate()
    cache_file = cache_module.get_cache_file(fake_ws.cache_dir)
    cache_file.write_text(content)

    manager = fake_ws.discover()

    assert plugin_ids(manager.python_plugins) == ["python:fake_shape.states:ShapeState"]
    assert cache_module.is_cache_shape_valid(json.loads(cache_file.read_text()))


def test_concurrent_writers_never_expose_partial_cache_files(tmp_path):
    # M07: all writers used to share one temporary file name.
    writer = textwrap.dedent("""
        import sys
        from pathlib import Path
        from yasmin_plugins_manager.cache import save_cache

        name = sys.argv[2]
        payload = {"writer": name, "blob": [name * 50] * int(sys.argv[3])}
        for _ in range(int(sys.argv[4])):
            save_cache(payload, Path(sys.argv[1]))
        """)
    cache_dir = tmp_path / "race"
    cache_dir.mkdir()
    writers = [
        subprocess.Popen(
            [sys.executable, "-c", writer, str(cache_dir), name, size, "150"],
            env=dict(os.environ),
        )
        for name, size in (("A", "2000"), ("B", "20000"))
    ]

    cache_file = cache_module.get_cache_file(cache_dir)
    valid_reads = invalid_reads = 0
    deadline = time.monotonic() + 120
    while any(proc.poll() is None for proc in writers) and time.monotonic() < deadline:
        try:
            json.loads(cache_file.read_text())
            valid_reads += 1
        except FileNotFoundError:
            continue
        except ValueError:
            invalid_reads += 1

    for proc in writers:
        proc.wait(timeout=60)
        assert proc.returncode == 0

    assert valid_reads > 0
    assert invalid_reads == 0
    assert [entry.name for entry in cache_dir.iterdir()] == [cache_file.name]


def test_cache_location_honors_xdg_and_environment(monkeypatch, tmp_path):
    # M07: XDG_CACHE_HOME is honored and every environment has its own file.
    monkeypatch.delenv("YASMIN_CACHE")
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "xdg"))
    assert cache_module.get_default_cache_dir() == (
        tmp_path / "xdg" / "yasmin_plugins_manager"
    )

    monkeypatch.setenv("XDG_CACHE_HOME", "relative/path")
    assert cache_module.get_default_cache_dir() == (
        Path(os.path.expanduser("~")) / ".cache" / "yasmin_plugins_manager"
    )

    monkeypatch.setenv("YASMIN_CACHE", str(tmp_path / "explicit"))
    assert cache_module.get_default_cache_dir() == tmp_path / "explicit"

    monkeypatch.setenv("AMENT_PREFIX_PATH", "/ws_one/install")
    first = cache_module.get_cache_file()
    monkeypatch.setenv("AMENT_PREFIX_PATH", "/ws_two/install")
    second = cache_module.get_cache_file()
    assert first != second
    assert first.parent == second.parent == tmp_path / "explicit"


def test_stale_cache_files_of_other_environments_are_removed(tmp_path):
    cache_dir = tmp_path / "cache"
    cache_dir.mkdir()
    old_file = cache_dir / "plugins_cache_0000000000000000.json"
    old_file.write_text("{}")
    old_time = time.time() - cache_module.STALE_CACHE_FILE_AGE_SEC - 60
    os.utime(old_file, (old_time, old_time))
    recent_file = cache_dir / "plugins_cache_1111111111111111.json"
    recent_file.write_text("{}")
    unrelated = cache_dir / "notes.json"
    unrelated.write_text("{}")

    cache_module.save_cache({"cache_version": 0}, cache_dir)

    names = sorted(entry.name for entry in cache_dir.iterdir())
    assert old_file.name not in names
    assert recent_file.name in names and unrelated.name in names
    assert cache_module.get_cache_file(cache_dir).name in names


def test_signature_validation_handles_bad_entries(tmp_path):
    existing = tmp_path / "file.txt"
    existing.write_text("x")

    assert cache_module.is_stat_signature_valid(
        cache_module.stat_signature(str(existing))
    )
    assert not cache_module.is_stat_signature_valid({"mtime_ns": 1})
    assert not cache_module.is_stat_signature_valid({"path": 3})
    assert not cache_module.is_stat_signature_valid(
        cache_module.missing_signature(str(existing))
    )
    assert cache_module.is_stat_signature_valid(
        cache_module.missing_signature(str(tmp_path / "absent"))
    )
