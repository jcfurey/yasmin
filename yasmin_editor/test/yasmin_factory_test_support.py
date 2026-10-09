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

"""Helpers to load editor XML with the real Python ``YasminFactory``.

The factory example XMLs live in ``yasmin_factory/test`` next to this package
and reference the Python test states in ``test.test_simple_state`` and the C++
test plugins of ``yasmin_factory``. Tests skip when those are not available.
"""

from __future__ import annotations

import json
import subprocess
import sys
import textwrap
from pathlib import Path
from typing import Any, Dict, List, Optional
from xml.etree import ElementTree as ET

FACTORY_SOURCE_DIR = Path(__file__).resolve().parents[2] / "yasmin_factory"
FACTORY_TEST_DIR = FACTORY_SOURCE_DIR / "test"


def factory_example_xmls() -> List[Path]:
    """Return the factory example XMLs whose root is a ``StateMachine``."""

    if not FACTORY_TEST_DIR.is_dir():
        return []
    examples = []
    for path in sorted(FACTORY_TEST_DIR.glob("*.xml")):
        try:
            if ET.parse(path).getroot().tag == "StateMachine":
                examples.append(path)
        except ET.ParseError:
            continue
    return examples


def use_factory_test_states(monkeypatch) -> None:
    """Make ``test.test_simple_state`` resolve to the factory's test states."""

    if not (FACTORY_TEST_DIR / "test_simple_state.py").is_file():
        import pytest

        pytest.skip("yasmin_factory test states are not available")
    monkeypatch.syspath_prepend(str(FACTORY_SOURCE_DIR))
    for name in [
        name for name in sys.modules if name == "test" or name.startswith("test.")
    ]:
        monkeypatch.delitem(sys.modules, name)


def _freeze(value: Any) -> Any:
    return json.loads(json.dumps(value, sort_keys=True, default=str))


def _entries(container: Any, getter: str) -> Dict[str, Any]:
    method = getattr(container, getter, None)
    if not callable(method):
        return {}
    items = method()
    return dict(items.items()) if hasattr(items, "items") else {}


def factory_fingerprint(state: Any) -> Dict[str, Any]:
    """Return a comparable description of a factory-built state tree."""

    fingerprint: Dict[str, Any] = {
        "kind": type(state).__name__,
        "outcomes": sorted(state.get_outcomes()),
        "description": state.get_description(),
        "input_keys": sorted(_freeze(state.get_input_keys()), key=str),
        "output_keys": sorted(_freeze(state.get_output_keys()), key=str),
        "parameters": sorted(_freeze(state.get_parameters()), key=str),
    }
    if hasattr(state, "get_outcome_descriptions"):
        fingerprint["outcome_descriptions"] = _freeze(state.get_outcome_descriptions())
    if hasattr(state, "get_start_state"):
        fingerprint["start_state"] = state.get_start_state()
    if hasattr(state, "get_default_outcome"):
        fingerprint["default_outcome"] = state.get_default_outcome()
    if hasattr(state, "get_outcome_map"):
        fingerprint["outcome_map"] = _freeze(state.get_outcome_map())
    if hasattr(state, "get_parameter_mappings"):
        fingerprint["parameter_mappings"] = _freeze(state.get_parameter_mappings())

    children: Dict[str, Any] = {}
    for name, entry in _entries(state, "get_states").items():
        child = entry.get("state", entry) if isinstance(entry, dict) else entry
        child_fingerprint = factory_fingerprint(child)
        if isinstance(entry, dict) and "transitions" in entry:
            child_fingerprint["transitions"] = _freeze(entry["transitions"])
        children[str(name)] = child_fingerprint
    for name, entry in _entries(state, "get_regions").items():
        children[f"region:{name}"] = factory_fingerprint(entry)
    if children:
        fingerprint["children"] = children
    return fingerprint


def load_with_factory(path: Path) -> Dict[str, Any]:
    """Load *path* with the Python factory in this process."""

    from yasmin_factory import YasminFactory

    return factory_fingerprint(YasminFactory().create_sm_from_file(str(path)))


def factory_load_error(path: Path, timeout: float = 60.0) -> Optional[str]:
    """Load *path* with the Python factory in a fresh process.

    Returns ``None`` on success and the error text otherwise. Used by the GUI
    tests, whose fixtures replace ``yasmin_factory`` with stubs in-process.
    """

    script = textwrap.dedent(f"""
        import sys
        sys.path.insert(0, {str(FACTORY_SOURCE_DIR)!r})
        for name in [n for n in sys.modules if n == "test" or n.startswith("test.")]:
            del sys.modules[name]
        from yasmin_factory import YasminFactory
        try:
            YasminFactory().create_sm_from_file({str(path)!r}).validate()
        except Exception as exc:
            print("FACTORY-ERROR:", exc)
        else:
            print("FACTORY-OK")
        """)
    completed = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    output = completed.stdout + completed.stderr
    if "FACTORY-OK" in completed.stdout:
        return None
    for line in completed.stdout.splitlines():
        if line.startswith("FACTORY-ERROR:"):
            return line[len("FACTORY-ERROR:") :].strip()
    return output.strip() or f"exit code {completed.returncode}"
