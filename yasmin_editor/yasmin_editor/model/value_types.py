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

"""Default-value typing shared with ``yasmin_factory``.

The editor must accept and reject exactly the ``default_type``/``default_value``
pairs that the factory accepts, so it reuses ``yasmin_factory.type_utils``.
That module only depends on ``json``. It is loaded straight from its file so
that importing the editor model does not execute ``yasmin_factory/__init__.py``
(which pulls in the factory, lxml and the YASMIN bindings).
"""

from __future__ import annotations

import importlib.machinery
import json
import importlib.util
import os
import sys
from types import ModuleType
from typing import Any, Optional

_TYPE_UTILS: Optional[ModuleType] = None


def _load_factory_type_utils() -> ModuleType:
    module = sys.modules.get("yasmin_factory.type_utils")
    if module is not None:
        return module

    spec = importlib.machinery.PathFinder.find_spec("yasmin_factory")
    locations = list(getattr(spec, "submodule_search_locations", None) or [])
    for location in locations:
        candidate = os.path.join(location, "type_utils.py")
        if not os.path.isfile(candidate):
            continue
        module_spec = importlib.util.spec_from_file_location(
            "yasmin_editor._factory_type_utils", candidate
        )
        if module_spec is None or module_spec.loader is None:
            continue
        module = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(module)
        return module

    raise ImportError("yasmin_factory.type_utils could not be located")


def factory_type_utils() -> ModuleType:
    """Return the ``yasmin_factory.type_utils`` module (loaded once)."""

    global _TYPE_UTILS
    if _TYPE_UTILS is None:
        _TYPE_UTILS = _load_factory_type_utils()
    return _TYPE_UTILS


def normalize_value_type(type_name: Optional[str]) -> str:
    """Normalize a default type like the factory (aliases, case, spaces).

    An empty type stays empty because the editor uses it for "no default".
    """

    if not str(type_name or "").strip():
        return ""
    try:
        return factory_type_utils().normalize_type(str(type_name))
    except ImportError:
        return str(type_name).strip().lower().replace(" ", "")


def default_value_error(value: Any, type_name: str) -> Optional[str]:
    """Return why the factory would reject a default, or ``None`` if it parses."""

    text = value if isinstance(value, str) else format_default_value(value, type_name)
    try:
        factory_type_utils().parse_key_value(text, type_name)
    except (ValueError, TypeError) as exc:
        return str(exc) or exc.__class__.__name__
    return None


def format_default_value(value: Any, type_name: Optional[str] = "") -> str:
    """Return the text the factory parses back into *value*.

    Plugin metadata and editor models may hold typed Python values. ``str()``
    would produce Python reprs (``['a']``, ``{'k': 1}``, ``True``) that
    ``yasmin_factory.type_utils.parse_key_value`` rejects, and ``str(x or "")``
    would drop falsy defaults such as ``0``. This mirrors
    ``yasmin_factory.type_utils.format_default_value``: JSON for containers,
    ``true``/``false`` for booleans and ``str`` for scalars. Text is returned
    unchanged.
    """

    if value is None:
        return ""
    if isinstance(value, str):
        return value
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, tuple, dict)):
        return json.dumps(value, ensure_ascii=False, separators=(",", ":"))
    return str(value)
