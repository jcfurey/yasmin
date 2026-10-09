# Copyright (C) 2025 Miguel Ángel González Santamarta
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

import importlib
import json
import os
import re
from pathlib import Path
from typing import Any, Dict, List, Optional

import yasmin

from ament_index_python import get_package_share_path
from lxml import etree as ET

_XML_VALUE_TYPE_ALIASES = {
    "string": "str",
    "integer": "int",
    "double": "float",
    "boolean": "bool",
    "list[string]": "list[str]",
    "list[double]": "list[float]",
    "list[boolean]": "list[bool]",
    "dict[string,str]": "dict[str,str]",
    "dict[str,string]": "dict[str,str]",
    "dict[string,string]": "dict[str,str]",
    "dict[string,int]": "dict[str,int]",
    "dict[string,integer]": "dict[str,int]",
    "dict[str,integer]": "dict[str,int]",
    "dict[string,float]": "dict[str,float]",
    "dict[string,double]": "dict[str,float]",
    "dict[str,double]": "dict[str,float]",
    "dict[string,bool]": "dict[str,bool]",
    "dict[string,boolean]": "dict[str,bool]",
    "dict[str,boolean]": "dict[str,bool]",
}
_XML_TRUE_VALUES = {"true", "1", "yes", "on"}
_XML_FALSE_VALUES = {"false", "0", "no", "off"}

_UNSERIALIZABLE = object()
_shared_cpp_factory = None


def _get_cpp_state_factory():
    """
    Return a shared C++ state factory.

    The pybind bridge loads pluginlib and rclcpp, so it is imported only when a
    C++ plugin is actually instantiated (normally inside the discovery worker).
    """
    global _shared_cpp_factory
    if _shared_cpp_factory is None:
        from yasmin_pybind_bridge import CppStateFactory

        _shared_cpp_factory = CppStateFactory()
    return _shared_cpp_factory


class PluginInfo:
    """Store metadata of a discovered plugin."""

    @staticmethod
    def _split_cpp_template_args(args: str) -> List[str]:
        """Split a C++ template argument list at top-level commas."""
        parts: List[str] = []
        current: List[str] = []
        depth = 0

        for char in args:
            if char == "<":
                depth += 1
                current.append(char)
            elif char == ">":
                depth = max(0, depth - 1)
                current.append(char)
            elif char == "," and depth == 0:
                part = "".join(current).strip()
                if part:
                    parts.append(part)
                current = []
            else:
                current.append(char)

        tail = "".join(current).strip()
        if tail:
            parts.append(tail)

        return parts

    @staticmethod
    def _normalize_cpp_metadata_type(type_name: str) -> str:
        """Normalize common C++ metadata type names for user-facing output."""
        normalized_type = " ".join(type_name.strip().split())

        string_fragment_pattern = re.compile(
            r"std::(?:__cxx11::)?basic_string<char(?:,\s*std::char_traits<char>)?(?:,\s*std::allocator<char>)?\s*>"
        )
        integer_type_pattern = re.compile(
            r"^(unsigned\s+)?(short|int|long|long\s+long)(\s+int)?$"
        )

        normalized_type = re.sub(r"\s*<\s*", "<", normalized_type)
        normalized_type = re.sub(r"\s*>\s*", ">", normalized_type)
        normalized_type = re.sub(r"\s*,\s*", ", ", normalized_type)

        if normalized_type == "std::string":
            return "str"

        canonical_type = string_fragment_pattern.sub("std::string", normalized_type)

        if canonical_type == "std::string":
            return "str"

        if canonical_type == "bool":
            return "bool"

        if canonical_type in {"float", "double", "long double"}:
            return "float"

        if integer_type_pattern.match(canonical_type):
            return "int"

        if canonical_type.startswith("const "):
            return PluginInfo._normalize_cpp_metadata_type(canonical_type[6:])

        if canonical_type.endswith("&"):
            return PluginInfo._normalize_cpp_metadata_type(canonical_type[:-1].strip())

        if canonical_type.endswith("*"):
            pointee_type = PluginInfo._normalize_cpp_metadata_type(
                canonical_type[:-1].strip()
            )
            return f"{pointee_type}*"

        template_match = re.match(r"^([a-zA-Z0-9_:]+)<(.+)>$", canonical_type)
        if not template_match:
            return canonical_type

        template_name = template_match.group(1)
        template_args = PluginInfo._split_cpp_template_args(template_match.group(2))

        if template_name == "std::vector" and len(template_args) >= 1:
            element_type = PluginInfo._normalize_cpp_metadata_type(template_args[0])
            return f"list[{element_type}]"

        if (
            template_name in {"std::unordered_map", "std::map"}
            and len(template_args) >= 2
        ):
            key_type = PluginInfo._normalize_cpp_metadata_type(template_args[0])
            value_type = PluginInfo._normalize_cpp_metadata_type(template_args[1])
            return f"dict[{key_type}, {value_type}]"

        if (
            template_name in {"std::shared_ptr", "std::unique_ptr"}
            and len(template_args) >= 1
        ):
            return PluginInfo._normalize_cpp_metadata_type(template_args[0])

        if template_name == "std::optional" and len(template_args) >= 1:
            value_type = PluginInfo._normalize_cpp_metadata_type(template_args[0])
            return f"optional[{value_type}]"

        return canonical_type

    @staticmethod
    def _is_primitive_metadata_value(value) -> bool:
        """Return whether a metadata default value is a JSON scalar."""
        return value is None or isinstance(value, (str, int, float, bool))

    @classmethod
    def _to_json_metadata_value(cls, value) -> Any:
        """
        Convert a metadata default value to a JSON-serializable value.

        Scalars, lists and string-keyed dictionaries are kept (tuples become
        lists). ``_UNSERIALIZABLE`` is returned for values that cannot be
        represented in the cache.
        """
        if cls._is_primitive_metadata_value(value):
            return value

        if isinstance(value, (list, tuple)):
            items = [cls._to_json_metadata_value(item) for item in value]
            if any(item is _UNSERIALIZABLE for item in items):
                return _UNSERIALIZABLE
            return items

        if isinstance(value, dict):
            result = {}
            for key, item in value.items():
                converted = cls._to_json_metadata_value(item)
                if not isinstance(key, str) or converted is _UNSERIALIZABLE:
                    return _UNSERIALIZABLE
                result[key] = converted
            return result

        return _UNSERIALIZABLE

    @staticmethod
    def _describe_metadata_value_type(value) -> str:
        """Return a stable type name for non-primitive metadata defaults."""
        return type(value).__name__

    @classmethod
    def _normalize_cpp_metadata_entries(cls, metadata_entries: List[dict]) -> List[dict]:
        """Normalize metadata dictionaries and sanitize non-serializable defaults."""
        normalized_entries: List[dict] = []

        for metadata_entry in metadata_entries:
            normalized_entry = dict(metadata_entry)

            default_value = normalized_entry.get("default_value")
            default_value_type = normalized_entry.get("default_value_type")

            if isinstance(default_value_type, str):
                normalized_entry["default_value_type"] = cls._normalize_cpp_metadata_type(
                    default_value_type
                )
            elif default_value is not None:
                normalized_entry["default_value_type"] = (
                    cls._describe_metadata_value_type(default_value)
                )

            if not cls._is_primitive_metadata_value(default_value):
                json_value = cls._to_json_metadata_value(default_value)
                if json_value is _UNSERIALIZABLE:
                    json_value = ""
                normalized_entry["default_value"] = json_value

            normalized_entries.append(normalized_entry)

        return normalized_entries

    @staticmethod
    def _normalize_xml_value_type(type_name: Optional[str]) -> str:
        """Normalize an XML default type name exactly like ``YasminFactory`` does."""
        normalized = "".join((type_name or "").split()).lower()
        normalized = _XML_VALUE_TYPE_ALIASES.get(normalized, normalized)
        return normalized or "str"

    @staticmethod
    def _display_xml_value_type(normalized_type: str) -> str:
        """Format a normalized XML type like the C++ metadata types are formatted."""
        return normalized_type.replace(",", ", ")

    @staticmethod
    def _parse_xml_value(value: str, normalized_type: str) -> Any:
        """
        Convert an XML default value to the value the factory stores.

        Values the factory would reject are returned unchanged as strings so
        that metadata extraction never fails on them.
        """
        try:
            if normalized_type == "str":
                return value
            if normalized_type == "int":
                return int(value)
            if normalized_type == "float":
                return float(value)
            if normalized_type == "bool":
                lowered = value.lower()
                if lowered in _XML_TRUE_VALUES:
                    return True
                if lowered in _XML_FALSE_VALUES:
                    return False
                return value
            return json.loads(value)
        except (TypeError, ValueError):
            return value

    @classmethod
    def _xml_key_info(
        cls,
        name: str,
        description: str,
        default_value: Optional[str] = None,
        default_type: Optional[str] = None,
    ) -> dict:
        """Build a key or parameter dictionary like ``key_info_to_dict`` does."""
        key_info = {
            "name": name,
            "description": description,
            "has_default": default_value is not None,
        }
        if default_value is not None:
            normalized_type = cls._normalize_xml_value_type(default_type)
            key_info["default_value_type"] = cls._display_xml_value_type(normalized_type)
            key_info["default_value"] = cls._to_json_metadata_value(
                cls._parse_xml_value(default_value, normalized_type)
            )
            if key_info["default_value"] is _UNSERIALIZABLE:
                key_info["default_value"] = default_value
        return key_info

    def __init__(
        self,
        plugin_type: str,
        class_name: Optional[str] = None,
        module: Optional[str] = None,
        file_name: Optional[str] = None,
        package_name: Optional[str] = None,
        relative_path: Optional[str] = None,
    ) -> None:
        """
        Load plugin metadata immediately.

        .. warning::

           **Trust model**: For ``python`` and ``cpp`` plugins this constructor
           imports the Python module or loads the C++ plugin library and creates
           a state instance to read its metadata, in the calling process.
           ``PluginManager`` therefore runs these constructors in a separate
           discovery worker process.

        Parameters
        ----------
        plugin_type : str
            Plugin type identifier. Supported values are ``python``, ``cpp`` and ``xml``.
        class_name : Optional[str]
            Class or exported plugin name.
        module : Optional[str]
            Python module containing the state class.
        file_name : Optional[str]
            XML file name.
        package_name : Optional[str]
            Package containing the plugin.
        relative_path : Optional[str]
            Relative path of an XML file inside the package share directory.
        """
        self._cpp_factory = None

        self.plugin_type: str = plugin_type
        self.class_name: Optional[str] = class_name
        self.module: Optional[str] = module
        self.file_name: Optional[str] = file_name
        self.package_name: Optional[str] = package_name
        self.relative_path: Optional[str] = (
            Path(relative_path).as_posix() if relative_path else relative_path
        )
        self.outcomes: List[str] = []
        self.description: str = ""
        self.outcome_descriptions: Dict[str, str] = {}
        self.input_keys: List[dict] = []
        self.output_keys: List[dict] = []
        self.parameters: List[dict] = []

        if self.plugin_type == "python":
            loaded_module = importlib.import_module(self.module)
            state_class = getattr(loaded_module, self.class_name)
            instance = state_class()
            self._load_instance_metadata(instance)

        elif self.plugin_type == "cpp":
            self._cpp_factory = _get_cpp_state_factory()
            instance = self._cpp_factory.create(self.class_name)
            self._load_instance_metadata(instance)

        elif self.plugin_type == "xml":
            self._load_xml_metadata()

    def _load_xml_metadata(self) -> None:
        """Load metadata of an XML state machine as ``YasminFactory`` builds it."""
        package_path = get_package_share_path(self.package_name)

        if self.relative_path:
            file_path = os.path.join(package_path, self.relative_path)
        else:
            file_path = ""
            if self.file_name:
                for root, dirs, files in os.walk(package_path):
                    dirs.sort()
                    if self.file_name in files:
                        file_path = os.path.join(root, self.file_name)
                        self.relative_path = Path(
                            os.path.relpath(file_path, package_path)
                        ).as_posix()
                        break

        if not file_path or not os.path.exists(file_path):
            raise ValueError(
                f"Could not find XML file {self.file_name} in package {self.package_name}"
            )

        if self.file_name is None:
            self.file_name = os.path.basename(file_path)

        tree = ET.parse(file_path)
        root = tree.getroot()
        outcomes_str: str = root.attrib.get("outcomes", "")
        self.outcomes = list(dict.fromkeys(outcomes_str.split()))
        self.description = root.attrib.get("description", "")
        self.outcome_descriptions = {}

        for outcome_elem in root.findall("FinalOutcome"):
            outcome_name = outcome_elem.attrib.get("name", "")
            outcome_description = outcome_elem.attrib.get("description", "")
            if outcome_name and outcome_description:
                self.outcome_descriptions[outcome_name] = outcome_description

        # Parameters: declare_parameter() replaces an existing declaration.
        parameters: Dict[str, dict] = {}
        for param_elem in root.findall("Param"):
            param_name = param_elem.attrib.get("name", "")
            if not param_name:
                continue
            parameters[param_name] = self._xml_key_info(
                param_name,
                param_elem.attrib.get("description", ""),
                param_elem.attrib.get("default_value"),
                param_elem.attrib.get("default_type", "str"),
            )
        self.parameters = list(parameters.values())

        # Keys: <Key> elements first, then every <Default> element as an input key.
        for key_elem in root.findall("Key"):
            key_name = key_elem.attrib.get("name", "")
            if not key_name:
                continue

            key_description = key_elem.attrib.get("description", "")
            key_type = key_elem.attrib.get("type", "in").lower()

            if key_type in ("in", "in/out"):
                self.input_keys.append(
                    self._xml_key_info(
                        key_name,
                        key_description,
                        key_elem.attrib.get("default_value"),
                        key_elem.attrib.get("default_type", "str"),
                    )
                )
            if key_type in ("out", "in/out"):
                self.output_keys.append(self._xml_key_info(key_name, key_description))

        for default_elem in root.findall("Default"):
            key_name = default_elem.attrib.get("key", "")
            value = default_elem.attrib.get("value")
            if not key_name or value is None:
                continue
            self.input_keys.append(
                self._xml_key_info(
                    key_name,
                    default_elem.attrib.get("description", ""),
                    value,
                    default_elem.attrib.get("type", "str"),
                )
            )

    @staticmethod
    def _safe_get(instance, attr: str, getter: str, transform=None, fallback=None):
        try:
            value = getattr(instance, getter)()
            if transform:
                value = transform(value)
            return value
        except Exception:
            yasmin.YASMIN_LOG_DEBUG(
                f"Failed to read {attr} via {getter} on {type(instance).__name__}"
            )
            return fallback

    def _load_instance_metadata(self, instance) -> None:
        """
        Load metadata from a Python or C++ state instance.

        Each metadata field is queried independently so one failing accessor
        does not suppress the remaining metadata.
        """
        # Outcomes are a set; sort them so the order is stable across processes.
        self.outcomes = self._safe_get(instance, "outcomes", "get_outcomes", sorted, [])
        self.description = self._safe_get(
            instance, "description", "get_description", None, ""
        )
        self.outcome_descriptions = self._safe_get(
            instance,
            "outcome_descriptions",
            "get_outcome_descriptions",
            lambda value: dict(sorted(dict(value).items())),
            {},
        )

        raw_input = self._safe_get(instance, "input_keys", "get_input_keys", list, [])
        self.input_keys = self._normalize_cpp_metadata_entries(raw_input)

        raw_output = self._safe_get(instance, "output_keys", "get_output_keys", list, [])
        self.output_keys = self._normalize_cpp_metadata_entries(raw_output)

        raw_params = self._safe_get(instance, "parameters", "get_parameters", list, [])
        self.parameters = self._normalize_cpp_metadata_entries(raw_params)

    def to_cache_dict(self) -> dict:
        """Serialize the full plugin metadata for caching."""
        return {
            "plugin_type": self.plugin_type,
            "class_name": self.class_name,
            "module": self.module,
            "file_name": self.file_name,
            "package_name": self.package_name,
            "relative_path": self.relative_path,
            "outcomes": self.outcomes,
            "description": self.description,
            "outcome_descriptions": self.outcome_descriptions,
            "input_keys": self.input_keys,
            "output_keys": self.output_keys,
            "parameters": self.parameters,
        }

    @classmethod
    def from_cache_dict(cls, data: dict) -> "PluginInfo":
        """
        Create a plugin info object from cached metadata.

        No plugin factory instance is created for cached entries.
        """
        instance = cls.__new__(cls)
        instance._cpp_factory = None
        instance.plugin_type = data.get("plugin_type")
        instance.class_name = data.get("class_name")
        instance.module = data.get("module")
        instance.file_name = data.get("file_name")
        instance.package_name = data.get("package_name")
        instance.relative_path = data.get("relative_path")
        instance.outcomes = list(data.get("outcomes") or [])
        instance.description = data.get("description") or ""
        instance.outcome_descriptions = dict(data.get("outcome_descriptions") or {})
        instance.input_keys = list(data.get("input_keys") or [])
        instance.output_keys = list(data.get("output_keys") or [])
        instance.parameters = list(data.get("parameters") or [])

        instance.input_keys = cls._normalize_cpp_metadata_entries(instance.input_keys)
        instance.output_keys = cls._normalize_cpp_metadata_entries(instance.output_keys)
        instance.parameters = cls._normalize_cpp_metadata_entries(instance.parameters)

        return instance

    @property
    def display_name(self) -> str:
        """Return the display name of the plugin."""
        if self.plugin_type in ("python", "cpp"):
            return self.class_name or ""
        return self.file_name or ""

    @property
    def unique_id(self) -> str:
        """
        Return a stable unique identifier for the plugin.

        XML state machines are identified by their path relative to the package
        share directory, because one package may contain several files with the
        same name in different directories.
        """
        if self.plugin_type == "python":
            return f"python:{self.module}:{self.class_name}"
        if self.plugin_type == "cpp":
            return f"cpp:{self.class_name}"
        return f"xml:{self.package_name}:{self.relative_path or self.file_name}"

    @property
    def dedup_key(self) -> str:
        """Return a stable key used for discovery deduplication."""
        return self.unique_id
