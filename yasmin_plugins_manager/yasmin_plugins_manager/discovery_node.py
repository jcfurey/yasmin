#!/usr/bin/env python3

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

import argparse
import json
from pathlib import Path
from typing import List

from rclpy.logging import get_logger
from rclpy.utilities import remove_ros_args

from .plugin_manager import PluginManager

LOGGER_NAME = "yasmin_plugins_discovery"


def parse_args(args=None):
    """Parse command line arguments for plugin discovery."""
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument(
        "--force-refresh",
        action="store_true",
        help="Ignore cache and perform a full rescan.",
    )
    parser.add_argument(
        "--max-cache-age-sec",
        type=int,
        default=0,
        help="Invalidate cache after this age in seconds. 0 disables age-based invalidation.",
    )
    parser.add_argument(
        "--package-timeout-sec",
        type=float,
        default=None,
        help=(
            "Time one package may take to load before it is skipped. Defaults to "
            "$YASMIN_DISCOVERY_PACKAGE_TIMEOUT or 10 seconds."
        ),
    )
    parser.add_argument(
        "--cache-dir",
        type=Path,
        default=None,
        help="Cache directory. Defaults to $YASMIN_CACHE or the user cache directory.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print full plugin metadata and every discovery failure.",
    )
    argv = remove_ros_args()[1:] if args is None else args
    return parser.parse_args(argv)


def _format_plugin_header(plugin) -> str:
    """Format the compact plugin header line."""
    if plugin.plugin_type == "python":
        source = f"module={plugin.module}"
    else:
        source = f"package={plugin.package_name}"

    return (
        f"[{plugin.plugin_type}] {plugin.display_name} | id={plugin.unique_id} | {source}"
    )


def _format_plugin_details(plugin) -> List[str]:
    """Format detailed plugin metadata lines."""
    details = [
        f"  class_name: {plugin.class_name}",
        f"  module: {plugin.module}",
        f"  file_name: {plugin.file_name}",
        f"  package_name: {plugin.package_name}",
        f"  relative_path: {getattr(plugin, 'relative_path', None)}",
        f"  description: {plugin.description or '-'}",
        f"  outcomes: {json.dumps(plugin.outcomes, ensure_ascii=False)}",
        f"  outcome_descriptions: {json.dumps(plugin.outcome_descriptions, ensure_ascii=False)}",
        f"  input_keys: {json.dumps(plugin.input_keys, ensure_ascii=False)}",
        f"  output_keys: {json.dumps(plugin.output_keys, ensure_ascii=False)}",
        f"  parameters: {json.dumps(plugin.parameters, ensure_ascii=False)}",
    ]
    return details


def _log_plugins(logger, title: str, plugins: list, verbose: bool) -> None:
    """Log a group of plugins."""
    logger.info(f"{title}: {len(plugins)}")

    for plugin in plugins:
        logger.info(_format_plugin_header(plugin))

        if not verbose:
            continue

        for line in _format_plugin_details(plugin):
            logger.info(line)


def _log_failures(logger, failures: list, verbose: bool) -> None:
    """Log discovery failures; YASMIN-related ones always, all with --verbose."""
    relevant = [failure for failure in failures if failure.get("relevant")]
    logger.info(
        f"Discovery failures: {len(failures)} ({len(relevant)} YASMIN-related"
        + ("" if verbose else "; use --verbose to list all")
        + ")"
    )

    for failure in failures if verbose else relevant:
        message = PluginManager.format_failure(failure)
        if failure.get("relevant"):
            logger.warning(message)
        else:
            logger.info(message)


def main() -> int:
    """Run plugin discovery and print the discovered plugins."""
    args = parse_args()
    logger = get_logger(LOGGER_NAME)

    # Discovery itself never initializes ROS in this process: plugin code runs in
    # a separate worker process.
    manager = PluginManager(
        cache_dir=args.cache_dir,
        max_cache_age_sec=args.max_cache_age_sec,
        package_timeout_sec=args.package_timeout_sec,
    )
    manager.load_all_plugins(
        force_refresh=args.force_refresh,
    )

    _log_plugins(logger, "C++ plugins", manager.cpp_plugins, args.verbose)
    _log_plugins(logger, "Python plugins", manager.python_plugins, args.verbose)
    _log_plugins(logger, "XML state machines", manager.xml_files, args.verbose)
    _log_failures(logger, manager.failures, args.verbose)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
