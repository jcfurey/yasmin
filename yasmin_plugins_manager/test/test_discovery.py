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

import os
import subprocess
import sys
import time

import pytest

from conftest import plugin_ids, state_machine_xml, state_source


def test_python_states_are_discovered_with_package_name(fake_ws):
    fake_ws.add_package("fake_basic")
    fake_ws.add_python_file("fake_basic/states.py", state_source("BasicState"))
    fake_ws.add_python_file(
        "fake_basic/needs_args.py",
        """
        from yasmin import State

        class NeedsArgsState(State):
            def __init__(self, outcomes):
                super().__init__(outcomes)

            def execute(self, blackboard):
                return "done"
        """,
    )

    manager = fake_ws.discover()

    assert plugin_ids(manager.python_plugins) == ["python:fake_basic.states:BasicState"]
    plugin = manager.python_plugins[0]
    assert plugin.package_name == "fake_basic"
    assert plugin.outcomes == ["done"]
    assert plugin.description == "BasicState description"


def test_prefix_below_a_test_directory_is_scanned(fake_ws):
    # M03: "/test" anywhere in the absolute path used to hide every module.
    prefix = fake_ws.prefix("test_ws/install")
    fake_ws.add_package("fake_tw", prefix=prefix)
    fake_ws.add_python_file("fake_tw/states.py", state_source("TwState"), prefix)
    fake_ws.add_python_file(
        "fake_tw/testing_utils/helpers.py", state_source("HelperState"), prefix
    )
    fake_ws.add_python_file("fake_tw/test/test_x.py", state_source("TestOnly"), prefix)
    fake_ws.add_python_file("fake_tw/tests/t.py", state_source("TestsOnly"), prefix)

    manager = fake_ws.discover()

    assert plugin_ids(manager.python_plugins) == [
        "python:fake_tw.states:TwState",
        "python:fake_tw.testing_utils.helpers:HelperState",
    ]


def test_system_exit_does_not_abort_discovery(fake_ws):
    # M02: __main__.py is skipped and SystemExit raised by a module is recorded.
    fake_ws.add_package("fake_exit")
    fake_ws.add_python_file("fake_exit/__main__.py", "import sys\nsys.exit(3)\n")
    fake_ws.add_python_file(
        "fake_exit/cli.py",
        "import argparse\nargparse.ArgumentParser().parse_args(['--bad'])\n",
    )
    fake_ws.add_python_file("fake_exit/states.py", state_source("ExitState"))
    fake_ws.add_package("fake_later")
    fake_ws.add_python_file("fake_later/states.py", state_source("LaterState"))

    manager = fake_ws.discover()

    assert plugin_ids(manager.python_plugins) == [
        "python:fake_exit.states:ExitState",
        "python:fake_later.states:LaterState",
    ]
    targets = {failure["target"]: failure for failure in manager.failures}
    assert "fake_exit.__main__" not in targets
    assert targets["fake_exit.cli"]["reason"] == "SystemExit(2)"


def test_hanging_module_only_loses_its_package(fake_ws):
    # M01: a module that blocks forever must not block discovery.
    fake_ws.add_package("fake_a_before")
    fake_ws.add_python_file("fake_a_before/states.py", state_source("BeforeState"))
    fake_ws.add_package("fake_hang")
    fake_ws.add_python_file(
        "fake_hang/blocking.py", "import threading\nthreading.Event().wait()\n"
    )
    fake_ws.add_python_file("fake_hang/states.py", state_source("LostState"))
    fake_ws.add_package("fake_z_after")
    fake_ws.add_python_file("fake_z_after/states.py", state_source("AfterState"))

    started = time.monotonic()
    result = fake_ws.run_discovery_subprocess(timeout=90, package_timeout_sec=2)
    elapsed = time.monotonic() - started

    assert result["python"] == [
        "python:fake_a_before.states:BeforeState",
        "python:fake_z_after.states:AfterState",
    ]
    failures = [f for f in result["failures"] if f["package"] == "fake_hang"]
    assert len(failures) == 1
    assert failures[0]["kind"] == "package"
    assert "timed out after 2 s while loading fake_hang.blocking" in failures[0]["reason"]
    assert elapsed < 60


def test_crashing_module_only_loses_its_package(fake_ws):
    # M01: a segfault in plugin code must not take the discovering process down.
    fake_ws.add_package("fake_crash")
    fake_ws.add_python_file(
        "fake_crash/boom.py", "import os, signal\nos.kill(os.getpid(), signal.SIGSEGV)\n"
    )
    fake_ws.add_package("fake_survivor")
    fake_ws.add_python_file("fake_survivor/states.py", state_source("SurvivorState"))

    result = fake_ws.run_discovery_subprocess(timeout=90)

    assert result["python"] == ["python:fake_survivor.states:SurvivorState"]
    failures = [f for f in result["failures"] if f["package"] == "fake_crash"]
    assert len(failures) == 1
    assert "SIGSEGV" in failures[0]["reason"]
    assert "fake_crash.boom" in failures[0]["reason"]


def test_plugin_code_does_not_run_in_the_calling_process(fake_ws, tmp_path):
    # M01: imports, ROS initialization and stdout/stdin use stay in the worker.
    marker = tmp_path / "imported.txt"
    fake_ws.add_package("fake_ros", depends=["yasmin"])
    fake_ws.add_python_file(
        "fake_ros/states.py",
        f"""
        import sys
        import rclpy
        from yasmin import State

        open({str(marker)!r}, "w").write("imported")
        print("noise on stdout")
        print("noise on stderr", file=sys.stderr)
        try:
            input()
        except EOFError:
            pass
        rclpy.init()

        class RosState(State):
            def __init__(self):
                super().__init__(["done"])

            def execute(self, blackboard):
                return "done"
        """,
    )

    result = fake_ws.run_discovery_subprocess(timeout=90)

    assert marker.read_text() == "imported"
    assert result["python"] == ["python:fake_ros.states:RosState"]
    assert result["parent"] == {
        "rclpy_ok": False,
        "bridge_loaded": False,
        "fake_modules": [],
    }


def test_unreadable_and_non_utf8_files_do_not_abort_discovery(fake_ws):
    # M04: one bad file must not break the discovery of everything else.
    fake_ws.add_package("fake_files")
    fake_ws.add_share_file(
        "fake_files",
        "config/latin1.xml",
        b'<?xml version="1.0" encoding="ISO-8859-1"?>\n<config>caf\xe9</config>\n',
    )
    fake_ws.add_share_file("fake_files", "config/broken.xml", "<StateMachine")
    fake_ws.add_share_file("fake_files", "sm/main.xml", state_machine_xml())
    plugins_xml = fake_ws.add_plugin_description(
        "fake_files",
        "plugins.xml",
        '<library path="x"><class name="fake/X" type="X" base_class_type="yasmin::State"/>'
        "</library>",
    )
    fake_ws.add_package("fake_other")
    fake_ws.add_python_file("fake_other/states.py", state_source("OtherState"))

    plugins_xml.chmod(0)
    try:
        if os.access(plugins_xml, os.R_OK):
            pytest.skip("running with permissions that ignore file modes")
        manager = fake_ws.discover()
    finally:
        plugins_xml.chmod(0o644)

    assert plugin_ids(manager.xml_files) == ["xml:fake_files:sm/main.xml"]
    assert plugin_ids(manager.python_plugins) == ["python:fake_other.states:OtherState"]
    failure = next(f for f in manager.failures if f["package"] == "fake_files")
    assert (
        failure["kind"] == "cpp" and failure["target"] == "share/fake_files/plugins.xml"
    )
    assert failure["relevant"] is True


def test_single_file_module_is_scanned_without_its_directory(fake_ws):
    # M09: a package that is a single module must not scan all of site-packages.
    fake_ws.add_package("fake_single")
    fake_ws.add_python_file("fake_single.py", state_source("SingleState"))
    fake_ws.add_python_file("unrelated_module.py", state_source("UnrelatedState"))

    manager = fake_ws.discover()

    assert plugin_ids(manager.python_plugins) == ["python:fake_single:SingleState"]
    assert not [f for f in manager.failures if "unrelated_module" in str(f["target"])]

    from yasmin_plugins_manager.cache import load_cache

    cache = load_cache(fake_ws.cache_dir)
    site_packages = str(fake_ws.site_packages())
    assert site_packages not in [signature["path"] for signature in cache["tracked_dirs"]]


def test_import_failures_are_reported(fake_ws, capfd):
    # M10: failed imports are recorded; YASMIN-related ones are logged as warnings.
    fake_ws.add_package("fake_broken")
    fake_ws.add_python_file(
        "fake_broken/my_states.py",
        "from yasmin import State\nimport does_not_exist_dependency\n",
    )
    fake_ws.add_package("fake_unrelated")
    fake_ws.add_python_file("fake_unrelated/tool.py", "import another_missing_module\n")

    manager = fake_ws.discover()

    failures = {failure["target"]: failure for failure in manager.failures}
    assert failures["fake_broken.my_states"]["relevant"] is True
    assert "does_not_exist_dependency" in failures["fake_broken.my_states"]["reason"]
    assert failures["fake_unrelated.tool"]["relevant"] is False

    err = capfd.readouterr().err
    assert "fake_broken.my_states" in err
    assert "fake_unrelated.tool" not in err


def test_ignore_mechanisms(fake_ws, monkeypatch):
    fake_ws.add_package("fake_env_ignored")
    fake_ws.add_python_file("fake_env_ignored/states.py", state_source("EnvState"))
    fake_ws.add_package(
        "fake_export_ignored", export='<yasmin_plugins_manager ignore="true"/>'
    )
    fake_ws.add_python_file("fake_export_ignored/states.py", state_source("ExportState"))
    fake_ws.add_package("fake_xml")
    fake_ws.add_share_file(
        "fake_xml",
        "sm/hidden.xml",
        "<!-- YASMIN_IGNORE_DISCOVERY -->\n" + state_machine_xml().split("\n", 1)[1],
    )
    fake_ws.add_share_file("fake_xml", "sm/visible.xml", state_machine_xml())
    monkeypatch.setenv("YASMIN_DISCOVERY_IGNORE_PACKAGES", "fake_env_ignored")

    manager = fake_ws.discover()

    assert plugin_ids(manager.python_plugins) == []
    assert plugin_ids(manager.xml_files) == ["xml:fake_xml:sm/visible.xml"]


def test_xml_state_machines_with_same_name_get_distinct_ids(fake_ws):
    # M12: XML ids use the path relative to the share directory.
    fake_ws.add_package("fake_dup_names")
    fake_ws.add_share_file("fake_dup_names", "sm/a/main.xml", state_machine_xml("a"))
    fake_ws.add_share_file("fake_dup_names", "sm/b/main.xml", state_machine_xml("b"))

    manager = fake_ws.discover()

    assert plugin_ids(manager.xml_files) == [
        "xml:fake_dup_names:sm/a/main.xml",
        "xml:fake_dup_names:sm/b/main.xml",
    ]
    assert {plugin.relative_path for plugin in manager.xml_files} == {
        "sm/a/main.xml",
        "sm/b/main.xml",
    }


def test_plugin_description_parsing(fake_ws):
    fake_ws.add_package("fake_desc")
    fake_ws.add_plugin_description(
        "fake_desc",
        "plugins.xml",
        """<class_libraries>
          <library path="lib_a">
            <class name="fake_desc/A" type="ns::A" base_class_type="yasmin::State"/>
            <class type="ns::TypeOnly" base_class_type="yasmin::State"/>
            <class name="fake_desc/Other" type="ns::O" base_class_type="other::Base"/>
          </library>
          <library path="lib_b">
            <class name="fake_desc/B" type="ns::B" base_class_type="yasmin::State"/>
          </library>
        </class_libraries>""",
    )
    fake_ws.add_plugin_description(
        "fake_desc",
        "more/plugins.xml",
        '<library path="lib_b"><class name="fake_desc/C" type="ns::C"'
        ' base_class_type="yasmin::State"/></library>',
    )
    manager = fake_ws.manager()

    resource_map = manager._get_cpp_plugin_resource_map()
    classes = manager._collect_cpp_plugin_classes("fake_desc", resource_map, [])

    assert classes == ["fake_desc/A", "ns::TypeOnly", "fake_desc/B", "fake_desc/C"]


def test_real_cpp_plugins_are_loaded_in_the_worker(fake_ws):
    # M12/M11: C++ plugins carry their package name and sorted outcomes.
    fake_ws.prefix()
    fake_ws.use_real_packages("yasmin", "yasmin_ros")

    result = fake_ws.run_discovery_subprocess(timeout=120)
    assert "cpp:yasmin_ros/TfBufferState" in result["cpp"]
    assert result["parent"]["bridge_loaded"] is False

    manager = fake_ws.discover()
    plugin = next(
        p for p in manager.cpp_plugins if p.unique_id == "cpp:yasmin_ros/TfBufferState"
    )
    assert plugin.package_name == "yasmin_ros"
    assert plugin.outcomes == sorted(plugin.outcomes)
    assert "cache_time_sec" in [parameter["name"] for parameter in plugin.parameters]


def test_python_list_and_dict_defaults_survive_discovery(fake_ws):
    # M12: JSON-serializable defaults are kept instead of being blanked.
    fake_ws.add_package("fake_defaults")
    fake_ws.add_python_file(
        "fake_defaults/states.py",
        """
        from yasmin import State

        class DefaultsState(State):
            def __init__(self):
                super().__init__(["done"])
                self.declare_parameter("waypoints", "list default", [1.0, 2.5])
                self.declare_parameter("limits", "dict default", {"speed": 2})
                self.add_input_key("pair", "tuple default", (1, 2))

            def execute(self, blackboard):
                return "done"
        """,
    )

    manager = fake_ws.discover()
    plugin = manager.python_plugins[0]

    parameters = {parameter["name"]: parameter for parameter in plugin.parameters}
    assert parameters["waypoints"]["default_value"] == [1.0, 2.5]
    assert parameters["waypoints"]["default_value_type"] == "list[float]"
    assert parameters["limits"]["default_value"] == {"speed": 2}
    assert parameters["limits"]["default_value_type"] == "dict[str, int]"
    assert plugin.input_keys[0]["default_value"] == [1, 2]


def test_discover_plugins_cli_verbose_output(fake_ws, tmp_path):
    # M10/M12: --verbose prints parameters and every failure.
    fake_ws.add_package("fake_cli", depends=["yasmin"])
    fake_ws.add_python_file(
        "fake_cli/states.py",
        """
        from yasmin import State

        class CliState(State):
            def __init__(self):
                super().__init__(["done"])
                self.declare_parameter("rate_hz", "Loop rate", 5)

            def execute(self, blackboard):
                return "done"
        """,
    )
    fake_ws.add_python_file("fake_cli/broken.py", "import missing_cli_dependency\n")
    fake_ws.activate()

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "yasmin_plugins_manager.discovery_node",
            "--force-refresh",
            "--verbose",
            "--cache-dir",
            str(tmp_path / "cli_cache"),
        ],
        capture_output=True,
        text=True,
        timeout=120,
        env=dict(os.environ),
    )

    output = completed.stdout + completed.stderr
    assert completed.returncode == 0, output
    assert "Python plugins: 1" in output
    assert "parameters:" in output and '"rate_hz"' in output
    assert "Discovery failures: 1 (1 YASMIN-related)" in output
    assert "missing_cli_dependency" in output
    assert (tmp_path / "cli_cache").is_dir()
