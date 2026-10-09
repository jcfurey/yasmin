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

import os
import sys
import time
from pathlib import Path
from typing import Dict, List, NamedTuple, Optional

import yasmin
from ament_index_python import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_path,
    get_packages_with_prefixes,
    get_search_paths,
)
from ament_index_python.resources import get_resource, get_resource_types, get_resources
from lxml import etree as ET
from tqdm import tqdm
from yasmin import LogLevel, get_log_level, set_log_level

from yasmin_plugins_manager.cache import (
    CACHE_VERSION,
    IGNORE_PACKAGES_ENV_VAR,
    build_environment_fingerprint,
    get_ignored_packages_from_env,
    is_cache_shape_valid,
    is_stat_signature_valid,
    load_cache,
    missing_signature,
    recursive_dir_signature,
    save_cache,
    stat_signature,
)
from yasmin_plugins_manager.discovery_worker import (
    SKIPPED_PYTHON_PACKAGES,
    DiscoveryWorkerSession,
    is_state_constructible_without_arguments,
    make_failure,
)
from yasmin_plugins_manager.plugin_info import PluginInfo

PACKAGE_IGNORE_EXPORT_TAG = "yasmin_plugins_manager"
PACKAGE_IGNORE_EXPORT_ATTRIBUTE = "ignore"
XML_DISCOVERY_IGNORE_COMMENT = "<!-- YASMIN_IGNORE_DISCOVERY -->"
TRUTHY_VALUES = {"1", "true", "yes", "on"}
DEPENDENCY_TAGS = {
    "depend",
    "build_depend",
    "build_export_depend",
    "exec_depend",
    "run_depend",
}
RESOURCE_INDEX_DIR = os.path.join("share", "ament_index", "resource_index")

__all__ = ["PluginInfo", "PluginManager"]


def _strip_namespace(tag: str) -> str:
    if "}" in tag:
        return tag.split("}", 1)[1]
    return tag


class PackageManifest(NamedTuple):
    """Discovery-relevant information from a package.xml."""

    ignored: bool = False
    depends_on_yasmin: bool = False


class PluginManager:
    """
    Discover and cache available YASMIN plugins.

    XML state machines and plugin description files are parsed in this process.
    Python modules are imported and C++ and Python states are instantiated in a
    separate discovery worker process (see ``discovery_worker``), so plugin code
    can neither initialize ROS in, block nor crash the calling process. A
    package that times out or crashes the worker is reported in ``failures``
    and the remaining packages are still discovered.
    """

    def __init__(
        self,
        cache_dir: Optional[Path] = None,
        max_cache_age_sec: int = 0,
        package_timeout_sec: Optional[float] = None,
    ) -> None:
        """
        Initialize the plugin manager.

        Parameters
        ----------
        cache_dir : Optional[Path]
            Optional custom cache directory.
        max_cache_age_sec : int
            Maximum cache age in seconds. A value of 0 disables age-based invalidation.
        package_timeout_sec : Optional[float]
            Time a single package may take to load in the discovery worker. Defaults
            to ``$YASMIN_DISCOVERY_PACKAGE_TIMEOUT`` or 10 seconds.
        """
        self.cache_dir = cache_dir
        self.max_cache_age_sec = max_cache_age_sec
        self.package_timeout_sec = package_timeout_sec
        self.cpp_plugins: List[PluginInfo] = []
        self.python_plugins: List[PluginInfo] = []
        self.xml_files: List[PluginInfo] = []
        self.failures: List[dict] = []

    def load_all_plugins(
        self,
        hide_progress: bool = False,
        force_refresh: bool = False,
    ) -> None:
        """
        Load all plugins from cache or perform a full discovery run.

        The cache is used when possible. If the cache is missing, outdated or invalid,
        a full discovery is performed and the result is written back to the cache.
        """
        saved_log_level = get_log_level()
        set_log_level(LogLevel.WARN)
        try:
            if not force_refresh and self._load_from_cache():
                return

            self.cpp_plugins = []
            self.python_plugins = []
            self.xml_files = []
            self.failures = []

            tracked_files: List[dict] = []
            tracked_dirs: List[dict] = []
            ignored_packages_env = set(get_ignored_packages_from_env())
            packages = sorted(get_packages_with_prefixes())
            cpp_resource_map = self._get_cpp_plugin_resource_map(tracked_files)

            with DiscoveryWorkerSession(self.package_timeout_sec) as session:
                for package in tqdm(
                    packages, desc="Loading plugins", disable=hide_progress
                ):
                    try:
                        self._discover_package(
                            package,
                            session,
                            ignored_packages_env,
                            cpp_resource_map,
                            tracked_files,
                            tracked_dirs,
                        )
                    except Exception as exc:
                        self._record_failure(
                            make_failure(
                                package,
                                "package",
                                None,
                                f"{type(exc).__name__}: {exc}",
                                True,
                            )
                        )

            self._dedup_plugins()
            self._save_to_cache(tracked_files, tracked_dirs)
        finally:
            set_log_level(saved_log_level)

    def _discover_package(
        self,
        package: str,
        session: DiscoveryWorkerSession,
        ignored_packages_env: set,
        cpp_resource_map: Dict[str, List[str]],
        tracked_files: List[dict],
        tracked_dirs: List[dict],
    ) -> None:
        """Discover the plugins of one package."""
        package_share_path: Optional[Path] = None
        try:
            package_share_path = get_package_share_path(package)
        except Exception:
            package_share_path = None

        manifest = PackageManifest()
        if package_share_path is not None:
            # Covers package.xml, plugin description files and XML state machines.
            share_signature = recursive_dir_signature(
                str(package_share_path), suffixes=(".xml",), skip_dirs=()
            )
            tracked_dirs.append(
                share_signature or missing_signature(str(package_share_path))
            )
            manifest = self._read_package_manifest(package_share_path / "package.xml")

        if package in ignored_packages_env or manifest.ignored:
            return

        cpp_classes = self._collect_cpp_plugin_classes(
            package, cpp_resource_map, tracked_files
        )

        if package_share_path is not None and package_share_path.is_dir():
            self.load_xml_state_machines_from_package(package)

        scan_python = self._should_scan_python_package(package)
        if not cpp_classes and not scan_python:
            return

        relevant = package.startswith("yasmin") or manifest.depends_on_yasmin
        self._run_worker_task(
            session,
            {
                "package": package,
                "cpp_classes": cpp_classes,
                "python": scan_python,
                "relevant": relevant,
            },
            tracked_dirs,
        )

    def _run_worker_task(
        self,
        session: DiscoveryWorkerSession,
        task: dict,
        tracked_dirs: Optional[List[dict]] = None,
    ) -> None:
        """Run one package task in the discovery worker and merge its result."""
        result, tracked, worker_failures = session.run(task)

        if tracked_dirs is not None:
            tracked_dirs.extend(tracked)

        if result is not None:
            for data in result.get("cpp_plugins", []):
                self.cpp_plugins.append(PluginInfo.from_cache_dict(data))
            for data in result.get("python_plugins", []):
                self.python_plugins.append(PluginInfo.from_cache_dict(data))
            for failure in result.get("failures", []):
                self._record_failure(failure)

        for failure in worker_failures:
            self._record_failure(failure)

    @staticmethod
    def _should_scan_python_package(package_name: str) -> bool:
        return not (
            package_name in SKIPPED_PYTHON_PACKAGES or package_name.startswith("_")
        )

    def _record_failure(self, failure: dict) -> None:
        """Store a discovery failure and report it."""
        self.failures.append(failure)
        message = self.format_failure(failure)
        if failure.get("relevant"):
            yasmin.YASMIN_LOG_WARN(message)
        else:
            yasmin.YASMIN_LOG_DEBUG(message)

    @staticmethod
    def format_failure(failure: dict) -> str:
        """Return a one-line description of a discovery failure."""
        package = failure.get("package")
        target = failure.get("target")
        reason = failure.get("reason")
        if failure.get("kind") == "package" or not target:
            return f"Discovery of package '{package}' failed: {reason}"
        return (
            f"Discovery of {failure.get('kind')} '{target}' in package '{package}' "
            f"failed: {reason}"
        )

    def _dedup_plugins(self) -> None:
        seen: set = set()
        for plugin_list in (self.cpp_plugins, self.python_plugins, self.xml_files):
            deduped: List[PluginInfo] = []
            for plugin in plugin_list:
                key = plugin.dedup_key
                if key not in seen:
                    seen.add(key)
                    deduped.append(plugin)
            plugin_list[:] = deduped

    def _load_from_cache(self) -> bool:
        """
        Load cached plugins if the cache is still valid.

        Returns
        -------
        bool
            True if the cache was successfully loaded, otherwise False.
        """
        cache = load_cache(self.cache_dir)
        if cache is None or not is_cache_shape_valid(cache):
            return False

        if cache.get("cache_version") != CACHE_VERSION:
            return False

        if self.max_cache_age_sec > 0:
            created_at = cache.get("created_at", 0.0)
            if time.time() - created_at > self.max_cache_age_sec:
                return False

        current_env = build_environment_fingerprint()
        if cache.get("environment_hash") != current_env["hash"]:
            return False

        if cache.get("ignored_packages_env") != current_env["payload"].get(
            "ignored_packages_env", []
        ):
            return False

        for key in ("tracked_files", "tracked_dirs"):
            for signature in cache.get(key, []):
                if not is_stat_signature_valid(signature):
                    return False

        try:
            cpp_plugins = [
                PluginInfo.from_cache_dict(data) for data in cache.get("cpp_plugins", [])
            ]
            python_plugins = [
                PluginInfo.from_cache_dict(data)
                for data in cache.get("python_plugins", [])
            ]
            xml_files = [
                PluginInfo.from_cache_dict(data) for data in cache.get("xml_files", [])
            ]
        except Exception:
            return False

        self.cpp_plugins = cpp_plugins
        self.python_plugins = python_plugins
        self.xml_files = xml_files
        self.failures = [dict(failure) for failure in cache.get("failures", [])]
        self._dedup_plugins()
        return True

    def _save_to_cache(
        self,
        tracked_files: List[dict],
        tracked_dirs: List[dict],
    ) -> None:
        """
        Save the discovered plugin metadata to the cache.

        Parameters
        ----------
        tracked_files : List[dict]
            File signatures used to invalidate the cache when discovery-relevant files change.
        tracked_dirs : List[dict]
            Directory signatures used to invalidate the cache when package contents change.
        """
        environment = build_environment_fingerprint()
        data = {
            "cache_version": CACHE_VERSION,
            "created_at": time.time(),
            "environment_hash": environment["hash"],
            "ignored_packages_env": environment["payload"].get(
                "ignored_packages_env", []
            ),
            "ignore_packages_env_var": IGNORE_PACKAGES_ENV_VAR,
            "tracked_files": self._unique_signatures(tracked_files),
            "tracked_dirs": self._unique_signatures(tracked_dirs),
            "cpp_plugins": [plugin.to_cache_dict() for plugin in self.cpp_plugins],
            "python_plugins": [plugin.to_cache_dict() for plugin in self.python_plugins],
            "xml_files": [plugin.to_cache_dict() for plugin in self.xml_files],
            "failures": self.failures,
        }
        save_cache(data, self.cache_dir)

    @staticmethod
    def _unique_signatures(signatures: List[dict]) -> List[dict]:
        unique: Dict[tuple, dict] = {}
        for signature in signatures:
            if not signature:
                continue
            key = (
                signature.get("path"),
                bool(signature.get("recursive")),
                tuple(signature.get("suffixes", [])),
            )
            unique.setdefault(key, signature)
        return list(unique.values())

    def _is_plugin_resource_type(self, resource_type: str) -> bool:
        """
        Check whether an ament resource type belongs to pluginlib.

        Parameters
        ----------
        resource_type : str
            Ament resource type name.

        Returns
        -------
        bool
            True if the resource type contains pluginlib plugin exports.
        """
        return "__pluginlib__plugin" in resource_type

    def _get_registered_plugin_resource_list(self) -> List[str]:
        """
        Return all pluginlib-related resource types from the ament index.

        Returns
        -------
        List[str]
            List of pluginlib resource type names.
        """
        return sorted(filter(self._is_plugin_resource_type, get_resource_types()))

    def _get_cpp_plugin_resource_map(
        self, tracked_files: Optional[List[dict]] = None
    ) -> Dict[str, List[str]]:
        """
        Build a mapping from package name to exported plugin XML resource paths.

        Parameters
        ----------
        tracked_files : Optional[List[dict]]
            Optional list that receives signatures of the resource index entries,
            so that newly registered or removed plugin exports invalidate the cache.

        Returns
        -------
        Dict[str, List[str]]
            Mapping from package name to plugin XML paths relative to the package prefix.
        """
        resource_map: Dict[str, List[str]] = {}
        resource_types = self._get_registered_plugin_resource_list()

        if tracked_files is not None:
            for prefix in get_search_paths():
                index_dir = os.path.join(prefix, RESOURCE_INDEX_DIR)
                tracked_files.append(
                    stat_signature(index_dir) or missing_signature(index_dir)
                )
                for resource_type in resource_types:
                    type_dir = os.path.join(index_dir, resource_type)
                    signature = stat_signature(type_dir)
                    if signature:
                        tracked_files.append(signature)

        for plugin_resource in resource_types:
            for package_name, prefix in get_resources(plugin_resource).items():
                resource_map.setdefault(package_name, [])

                if tracked_files is not None:
                    signature = stat_signature(
                        os.path.join(
                            prefix, RESOURCE_INDEX_DIR, plugin_resource, package_name
                        )
                    )
                    if signature:
                        tracked_files.append(signature)

                try:
                    component_registry, _ = get_resource(plugin_resource, package_name)
                except Exception:
                    continue

                resource_map[package_name] += [
                    line.split(";")[0].strip()
                    for line in component_registry.splitlines()
                    if line.strip()
                ]

        return resource_map

    def _is_truthy(self, value: Optional[str]) -> bool:
        """Return whether an XML attribute value should be interpreted as true."""
        if value is None:
            return False
        return value.strip().lower() in TRUTHY_VALUES

    def _read_package_manifest(self, package_xml_path: Path) -> PackageManifest:
        """Read the ignore export and the YASMIN dependencies from a package.xml."""
        try:
            if not package_xml_path.is_file():
                return PackageManifest()
            root = ET.parse(str(package_xml_path)).getroot()
        except Exception:
            return PackageManifest()

        ignored = False
        export_elem = root.find("export")
        if export_elem is not None:
            for child in export_elem:
                if child.tag != PACKAGE_IGNORE_EXPORT_TAG:
                    continue
                if self._is_truthy(child.attrib.get(PACKAGE_IGNORE_EXPORT_ATTRIBUTE)):
                    ignored = True

        depends_on_yasmin = any(
            child.tag in DEPENDENCY_TAGS
            and (child.text or "").strip().startswith("yasmin")
            for child in root
        )
        return PackageManifest(ignored=ignored, depends_on_yasmin=depends_on_yasmin)

    def _package_has_discovery_ignore_export(self, package_xml_path: Path) -> bool:
        """
        Check whether a package.xml export block disables discovery for the package.

        A package is ignored when it contains the following export tag:

        <export>
          <yasmin_plugins_manager ignore="true"/>
        </export>
        """
        return self._read_package_manifest(Path(package_xml_path)).ignored

    def _xml_file_has_discovery_ignore_comment(self, xml_file: str) -> bool:
        """
        Check whether the first non-empty line disables XML discovery.

        XML discovery is skipped when the first non-empty line is exactly:
        <!-- YASMIN_IGNORE_DISCOVERY -->

        The file is read as bytes, so files in any encoding are handled.
        """
        marker = XML_DISCOVERY_IGNORE_COMMENT.encode("ascii")
        try:
            with open(xml_file, "rb") as handle:
                for line in handle:
                    first_line = line.lstrip(b"\xef\xbb\xbf").strip()
                    if not first_line:
                        continue
                    return first_line == marker
        except OSError:
            return False

        return False

    @staticmethod
    def _plugin_library_signatures(prefix: str, library_path: str) -> List[dict]:
        """
        Return signatures of the shared library a pluginlib ``<library>`` refers to.

        A rebuilt library changes the metadata of its states, so it must
        invalidate the cache. If the library does not exist, its future
        appearance invalidates the cache instead.
        """
        base_name = os.path.basename(library_path)
        if sys.platform == "win32":
            extension, directories = ".dll", ("bin", "lib")
        elif sys.platform == "darwin":
            extension, directories = ".dylib", ("lib",)
        else:
            extension, directories = ".so", ("lib",)

        candidates = []
        for directory in directories:
            for name in (f"lib{base_name}", base_name):
                for suffix in (extension, ""):
                    candidates.append(os.path.join(prefix, directory, name + suffix))
        candidates.append(os.path.join(prefix, library_path + extension))

        signatures = [
            signature
            for signature in (stat_signature(path) for path in dict.fromkeys(candidates))
            if signature and os.path.isfile(signature["path"])
        ]
        if signatures:
            return signatures
        return [missing_signature(candidates[0])]

    def _collect_cpp_plugin_classes(
        self,
        package_name: str,
        cpp_resource_map: Dict[str, List[str]],
        tracked_files: Optional[List[dict]] = None,
    ) -> List[str]:
        """
        Parse the pluginlib description files of a package.

        Returns
        -------
        List[str]
            Lookup names of all exported classes with base class ``yasmin::State``.
        """
        resource_paths = cpp_resource_map.get(package_name, [])
        if not resource_paths:
            return []

        try:
            package_prefix = get_package_prefix(package_name)
        except PackageNotFoundError:
            return []

        class_names: List[str] = []
        for resource_path in dict.fromkeys(resource_paths):
            plugin_xml = os.path.join(package_prefix, resource_path)

            if tracked_files is not None:
                tracked_files.append(
                    stat_signature(plugin_xml) or missing_signature(plugin_xml)
                )

            if not os.path.isfile(plugin_xml):
                continue

            try:
                tree = ET.parse(plugin_xml)
            except Exception as exc:
                self._record_failure(
                    make_failure(
                        package_name,
                        "cpp",
                        resource_path,
                        f"cannot parse plugin description: {exc}",
                        True,
                    )
                )
                continue

            for elem in tree.iter("class"):
                if elem.attrib.get("base_class_type", "").strip() != "yasmin::State":
                    continue

                plugin_type = elem.attrib.get("name") or elem.attrib.get("type")
                if not plugin_type:
                    continue
                class_names.append(plugin_type)

                library = next(elem.iterancestors("library"), None)
                library_path = library.attrib.get("path") if library is not None else None
                if library_path and tracked_files is not None:
                    tracked_files.extend(
                        self._plugin_library_signatures(package_prefix, library_path)
                    )

        return list(dict.fromkeys(class_names))

    def load_cpp_plugins_from_package(
        self,
        package_name: str,
        tracked_files: Optional[List[dict]] = None,
        cpp_resource_map: Optional[Dict[str, List[str]]] = None,
    ) -> None:
        """
        Discover YASMIN C++ plugins from pluginlib exports.

        This uses the ament resource index. Only plugin classes exported with base class
        ``yasmin::State`` are loaded. The classes are instantiated in a discovery
        worker process.

        Parameters
        ----------
        package_name : str
            Package name to inspect.
        tracked_files : Optional[List[dict]]
            Optional list that receives signatures of parsed plugin XML files.
        cpp_resource_map : Optional[Dict[str, List[str]]]
            Optional precomputed map of package names to plugin XML resource paths.
        """
        if cpp_resource_map is None:
            cpp_resource_map = self._get_cpp_plugin_resource_map()

        class_names = self._collect_cpp_plugin_classes(
            package_name, cpp_resource_map, tracked_files
        )
        if not class_names:
            return

        with DiscoveryWorkerSession(self.package_timeout_sec) as session:
            self._run_worker_task(
                session,
                {
                    "package": package_name,
                    "cpp_classes": class_names,
                    "python": False,
                    "relevant": True,
                },
            )

    def _is_python_state_constructible_without_arguments(self, state_class: type) -> bool:
        """
        Check whether a Python state class can be instantiated without arguments.

        Parameters
        ----------
        state_class : type
            State class to inspect.

        Returns
        -------
        bool
            True if ``state_class()`` is expected to work without providing arguments.
        """
        return is_state_constructible_without_arguments(state_class)

    def load_python_plugins_from_package(
        self,
        package_name: str,
        tracked_files: Optional[List[dict]] = None,
        tracked_dirs: Optional[List[dict]] = None,
    ) -> None:
        """
        Discover Python plugins by scanning a Python package in a discovery worker.

        Parameters
        ----------
        package_name : str
            Package name to inspect.
        tracked_files : Optional[List[dict]]
            Unused; kept for API compatibility.
        tracked_dirs : Optional[List[dict]]
            Optional list that receives the package directory signature.
        """
        if not self._should_scan_python_package(package_name):
            return

        with DiscoveryWorkerSession(self.package_timeout_sec) as session:
            self._run_worker_task(
                session,
                {
                    "package": package_name,
                    "cpp_classes": [],
                    "python": True,
                    "relevant": True,
                },
                tracked_dirs,
            )

    def load_xml_state_machines_from_package(
        self,
        package_name: str,
        tracked_files: Optional[List[dict]] = None,
    ) -> None:
        """
        Discover XML state machines from a package share directory.

        Parameters
        ----------
        package_name : str
            Package name to inspect.
        tracked_files : Optional[List[dict]]
            Optional list that receives signatures of relevant XML files.
        """
        package_share_path = get_package_share_path(package_name)

        for root, dirs, files in os.walk(package_share_path):
            dirs.sort()
            for filename in sorted(files):
                if not filename.endswith(".xml"):
                    continue

                xml_file: str = os.path.join(root, filename)

                if tracked_files is not None:
                    signature = stat_signature(xml_file)
                    if signature:
                        tracked_files.append(signature)

                if self._xml_file_has_discovery_ignore_comment(xml_file):
                    continue

                try:
                    xml_root = ET.parse(xml_file).getroot()
                except Exception:
                    # Not every XML file in a share directory is well formed.
                    continue

                if _strip_namespace(str(xml_root.tag)) != "StateMachine":
                    continue

                relative_path = Path(
                    os.path.relpath(xml_file, package_share_path)
                ).as_posix()
                self.load_xml_state_machine(
                    filename,
                    package_name,
                    relative_path=relative_path,
                )

    def load_cpp_plugin(
        self, class_name: str, package_name: Optional[str] = None
    ) -> None:
        """
        Load one C++ plugin in this process.

        Parameters
        ----------
        class_name : str
            Fully qualified exported C++ class name.
        package_name : Optional[str]
            Package that exports the plugin.
        """
        try:
            plugin_info: PluginInfo = PluginInfo(
                plugin_type="cpp",
                class_name=class_name,
                package_name=package_name,
            )
        except Exception as e:
            self._record_failure(
                make_failure(str(package_name), "cpp", class_name, str(e), True)
            )
            return
        self.cpp_plugins.append(plugin_info)

    def load_python_plugin(
        self, module: str, class_name: str, package_name: Optional[str] = None
    ) -> None:
        """
        Load one Python plugin in this process.

        Parameters
        ----------
        module : str
            Python module containing the state class.
        class_name : str
            Python class name of the state.
        package_name : Optional[str]
            Package that contains the module.
        """
        try:
            plugin_info: PluginInfo = PluginInfo(
                plugin_type="python",
                class_name=class_name,
                module=module,
                package_name=package_name,
            )
        except Exception as e:
            self._record_failure(
                make_failure(
                    str(package_name), "python", f"{module}.{class_name}", str(e), True
                )
            )
            return
        self.python_plugins.append(plugin_info)

    def load_xml_state_machine(
        self,
        xml_file: str,
        package_name: Optional[str] = None,
        relative_path: Optional[str] = None,
    ) -> None:
        """
        Load one XML state machine.

        Parameters
        ----------
        xml_file : str
            File name of the XML state machine.
        package_name : Optional[str]
            Package that contains the XML state machine.
        relative_path : Optional[str]
            Relative path inside the package share directory.
        """
        try:
            plugin_info: PluginInfo = PluginInfo(
                plugin_type="xml",
                file_name=xml_file,
                package_name=package_name,
                relative_path=relative_path,
            )
        except Exception as e:
            self._record_failure(
                make_failure(
                    str(package_name), "xml", relative_path or xml_file, str(e), True
                )
            )
            return
        self.xml_files.append(plugin_info)
