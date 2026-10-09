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

"""Run a forced discovery and print the result and the parent's state as JSON."""

import json
import sys
from pathlib import Path

from yasmin_plugins_manager.plugin_manager import PluginManager


def main() -> None:
    cache_dir = Path(sys.argv[1])
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else None

    kwargs = {"package_timeout_sec": timeout} if timeout is not None else {}
    manager = PluginManager(cache_dir=cache_dir, **kwargs)
    manager.load_all_plugins(hide_progress=True, force_refresh=True)

    rclpy = sys.modules.get("rclpy")
    print(
        json.dumps(
            {
                "cpp": sorted(plugin.unique_id for plugin in manager.cpp_plugins),
                "python": sorted(plugin.unique_id for plugin in manager.python_plugins),
                "xml": sorted(plugin.unique_id for plugin in manager.xml_files),
                "failures": getattr(manager, "failures", []),
                "parent": {
                    "rclpy_ok": bool(rclpy is not None and rclpy.ok()),
                    "bridge_loaded": "yasmin_pybind_bridge" in sys.modules,
                    "fake_modules": sorted(
                        name for name in sys.modules if name.startswith("fake")
                    ),
                },
            }
        )
    )


if __name__ == "__main__":
    main()
