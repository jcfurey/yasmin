# Copyright (C) 2026 Maik Knof
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

import hashlib
import json
import os
import platform
import re
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import yasmin

from ament_index_python import get_packages_with_prefixes

CACHE_VERSION = 5
IGNORE_PACKAGES_ENV_VAR = "YASMIN_DISCOVERY_IGNORE_PACKAGES"
CACHE_DIR_ENV_VAR = "YASMIN_CACHE"
CACHE_FILE_PREFIX = "plugins_cache_"
CACHE_FILE_SUFFIX = ".json"
# Cache files of other environments that were not used for this long are removed.
STALE_CACHE_FILE_AGE_SEC = 30 * 24 * 3600

_CACHE_LIST_KEYS = (
    "tracked_files",
    "tracked_dirs",
    "cpp_plugins",
    "python_plugins",
    "xml_files",
    "failures",
)


def get_default_cache_dir() -> Path:
    """
    Return the default cache directory.

    ``YASMIN_CACHE`` takes precedence, then ``$XDG_CACHE_HOME/yasmin_plugins_manager``
    and finally ``~/.cache/yasmin_plugins_manager``.
    """
    cache_dir = os.environ.get(CACHE_DIR_ENV_VAR)
    if cache_dir:
        return Path(cache_dir)

    xdg_cache_home = os.environ.get("XDG_CACHE_HOME", "")
    if not os.path.isabs(xdg_cache_home):
        # The XDG specification requires an absolute path; ignore anything else.
        xdg_cache_home = os.path.join(os.path.expanduser("~"), ".cache")

    return Path(xdg_cache_home) / "yasmin_plugins_manager"


def ensure_cache_dir(cache_dir: Optional[Path] = None) -> Path:
    """Create the cache directory if it does not exist."""
    path = Path(cache_dir) if cache_dir else get_default_cache_dir()
    path.mkdir(parents=True, exist_ok=True)
    return path


def get_environment_key() -> str:
    """
    Return a short key identifying the active workspace environment.

    Each environment gets its own cache file, so terminals with different
    workspaces sourced do not keep invalidating each other's cache.
    """
    payload = {
        "ament_prefix_path": os.environ.get("AMENT_PREFIX_PATH", ""),
        "pythonpath": os.environ.get("PYTHONPATH", ""),
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "python_version": f"{sys.version_info.major}.{sys.version_info.minor}",
    }
    payload_json = json.dumps(payload, sort_keys=True)
    return hashlib.sha256(payload_json.encode("utf-8")).hexdigest()[:16]


def get_cache_file(cache_dir: Optional[Path] = None) -> Path:
    """Return the cache file path of the active environment."""
    file_name = f"{CACHE_FILE_PREFIX}{get_environment_key()}{CACHE_FILE_SUFFIX}"
    return ensure_cache_dir(cache_dir) / file_name


def get_ignored_packages_from_env() -> List[str]:
    """Return the sorted package ignore list configured through the environment."""
    raw_value = os.environ.get(IGNORE_PACKAGES_ENV_VAR, "")
    if not raw_value.strip():
        return []

    normalized_value = raw_value
    for separator in [",", ";", os.pathsep]:
        normalized_value = normalized_value.replace(separator, " ")

    packages = {
        item.strip() for item in re.split(r"\s+", normalized_value) if item.strip()
    }
    return sorted(packages)


def build_environment_fingerprint() -> Dict[str, Any]:
    """Build a fingerprint for the active ROS and Python environment."""
    packages = get_packages_with_prefixes()
    ignored_packages = get_ignored_packages_from_env()
    payload = {
        "cache_version": CACHE_VERSION,
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "ament_prefix_path": os.environ.get("AMENT_PREFIX_PATH", ""),
        "pythonpath": os.environ.get("PYTHONPATH", ""),
        "python_version": f"{sys.version_info.major}.{sys.version_info.minor}",
        "platform": platform.platform(),
        "ignored_packages_env": ignored_packages,
        "packages": dict(sorted(packages.items())),
    }
    payload_json = json.dumps(payload, sort_keys=True)
    payload_hash = hashlib.sha256(payload_json.encode("utf-8")).hexdigest()
    return {
        "hash": payload_hash,
        "payload": payload,
    }


def _is_signature(value: Any) -> bool:
    return isinstance(value, dict) and isinstance(value.get("path"), str)


def is_cache_shape_valid(cache: Any) -> bool:
    """
    Check that a loaded cache has the structure written by ``save_cache``.

    A cache with an unexpected shape is treated like a missing cache, which
    triggers a full discovery instead of an exception.
    """
    if not isinstance(cache, dict):
        return False

    for key in _CACHE_LIST_KEYS:
        value = cache.get(key, [])
        if not isinstance(value, list):
            return False
        if not all(isinstance(item, dict) for item in value):
            return False

    for key in ("tracked_files", "tracked_dirs"):
        if not all(_is_signature(item) for item in cache.get(key, [])):
            return False

    if not isinstance(cache.get("ignored_packages_env", []), list):
        return False

    return isinstance(cache.get("created_at", 0.0), (int, float))


def load_cache(cache_dir: Optional[Path] = None) -> Optional[Dict[str, Any]]:
    """Load the cache file if it exists and has a valid structure."""
    try:
        cache_file = get_cache_file(cache_dir)
    except OSError:
        yasmin.YASMIN_LOG_DEBUG("Failed to create cache directory")
        return None

    if not cache_file.exists():
        return None

    try:
        with cache_file.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
    except Exception:
        yasmin.YASMIN_LOG_DEBUG("Failed to load cache file")
        return None

    if not is_cache_shape_valid(data):
        yasmin.YASMIN_LOG_DEBUG("Ignoring cache file with an unexpected structure")
        return None

    return data


def _remove_stale_cache_files(cache_dir: Path, keep: Path) -> None:
    """Remove cache files of other environments that were not used for a long time."""
    now = time.time()
    try:
        entries = list(cache_dir.iterdir())
    except OSError:
        return

    for entry in entries:
        if entry == keep or not entry.name.startswith(CACHE_FILE_PREFIX):
            continue
        if not entry.name.endswith(CACHE_FILE_SUFFIX):
            continue
        try:
            if now - entry.stat().st_mtime > STALE_CACHE_FILE_AGE_SEC:
                entry.unlink()
        except OSError:
            continue


def save_cache(data: Dict[str, Any], cache_dir: Optional[Path] = None) -> None:
    """
    Write the cache file atomically.

    Each writer uses its own temporary file, so concurrent writers never share
    a partially written file and readers only ever see complete cache files.
    """
    tmp_path: Optional[str] = None
    try:
        cache_file = get_cache_file(cache_dir)
        fd, tmp_path = tempfile.mkstemp(
            dir=str(cache_file.parent),
            prefix=f".{cache_file.stem}.",
            suffix=".tmp",
        )
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2, sort_keys=True)
            handle.flush()
            os.fsync(handle.fileno())

        os.replace(tmp_path, cache_file)
        tmp_path = None
        _remove_stale_cache_files(cache_file.parent, cache_file)
    except (OSError, TypeError, ValueError) as exc:
        yasmin.YASMIN_LOG_DEBUG(f'Failed to save cache file: "{exc}"')
    finally:
        if tmp_path is not None:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass


def stat_signature(path: str) -> Optional[Dict[str, Any]]:
    """Return a small signature for a file or directory."""
    try:
        stat_result = os.stat(path)
    except OSError:
        return None

    return {
        "path": path,
        "mtime_ns": stat_result.st_mtime_ns,
        "size": stat_result.st_size,
    }


def missing_signature(path: str) -> Dict[str, Any]:
    """Return a signature that stays valid only while ``path`` does not exist."""
    return {
        "path": path,
        "missing": True,
    }


def _recursive_digest(
    dir_path: str,
    suffixes: Iterable[str],
    skip_dirs: Iterable[str],
) -> str:
    suffixes = tuple(suffixes)
    skip_dirs = set(skip_dirs)
    hasher = hashlib.sha256()

    for root, dirs, files in os.walk(dir_path):
        dirs[:] = sorted(d for d in dirs if d not in skip_dirs)
        for file_name in sorted(files):
            if suffixes and not file_name.endswith(suffixes):
                continue
            file_path = os.path.join(root, file_name)
            try:
                stat_result = os.stat(file_path)
            except OSError:
                continue
            rel_path = os.path.relpath(file_path, dir_path)
            hasher.update(
                f"{rel_path}:{stat_result.st_mtime_ns}:{stat_result.st_size}\n".encode()
            )

    return hasher.hexdigest()


def recursive_dir_signature(
    dir_path: str,
    suffixes: Iterable[str] = (".py",),
    skip_dirs: Iterable[str] = ("__pycache__",),
) -> Optional[Dict[str, Any]]:
    """
    Return a combined signature of all matching files below a directory.

    The signature changes when a matching file is added, removed or modified
    anywhere below ``dir_path``.
    """
    if not os.path.isdir(dir_path):
        return None

    suffixes = list(suffixes)
    skip_dirs = list(skip_dirs)
    return {
        "path": dir_path,
        "recursive": True,
        "suffixes": suffixes,
        "skip_dirs": skip_dirs,
        "digest": _recursive_digest(dir_path, suffixes, skip_dirs),
    }


def is_stat_signature_valid(signature: Dict[str, Any]) -> bool:
    """Check whether a cached file signature is still valid."""
    if not _is_signature(signature):
        return False

    path = signature["path"]

    if signature.get("missing"):
        return not os.path.exists(path)

    if signature.get("recursive"):
        suffixes = signature.get("suffixes", [".py"])
        skip_dirs = signature.get("skip_dirs", ["__pycache__"])
        if not isinstance(suffixes, list) or not isinstance(skip_dirs, list):
            return False
        if not os.path.isdir(path):
            return False
        return _recursive_digest(path, suffixes, skip_dirs) == signature.get("digest")

    current = stat_signature(path)
    if current is None:
        return False

    return current["mtime_ns"] == signature.get("mtime_ns") and current[
        "size"
    ] == signature.get("size")
