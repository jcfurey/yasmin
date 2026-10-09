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

import pytest

from yasmin_plugins_manager.plugin_info import PluginInfo

KEYS_XML = """<?xml version="1.0"?>
<StateMachine outcomes="done failed done" description="keys demo" start_state="S">
  <FinalOutcome name="done" description="finished"/>
  <Key name="k_in" type="in" description="plain input"/>
  <Key name="k_out" type="Out" description="capitalised output"/>
  <Key name="k_both" type="IN/OUT" description="both" default_type="int" default_value="7"/>
  <Key name="k_default" description="defaulted" default_type="Double" default_value="2.5"/>
  <Default key="d_rate" value="5" type="integer" description="defaulted input"/>
  <Default key="d_flag" value="Yes" type="bool"/>
  <Default key="d_names" value='["a", "b"]' type="list[string]"/>
  <Param name="p_mode" description="mode" default_value="fast"/>
  <Param name="p_gain" default_type="float" default_value="0.5"/>
  <Param name="p_required" description="no default"/>
  <State name="S" type="py" module="yasmin_ros.tf_buffer_state" class="TfBufferState">
    <Transition from="succeeded" to="done"/>
    <Transition from="aborted" to="failed"/>
  </State>
</StateMachine>
"""

FACTORY_SCRIPT = textwrap.dedent("""
    import json, sys
    from yasmin_factory import YasminFactory

    sm = YasminFactory().create_sm_from_file(sys.argv[1])
    print(json.dumps({
        "input_keys": sm.get_input_keys(),
        "output_keys": sm.get_output_keys(),
        "parameters": sm.get_parameters(),
    }, default=str))
    """)


@pytest.fixture
def keys_package(fake_ws):
    fake_ws.add_package("fake_keys")
    path = fake_ws.add_share_file("fake_keys", "sm/keys.xml", KEYS_XML)
    fake_ws.activate()
    return path


def _xml_info(relative_path="sm/keys.xml"):
    return PluginInfo(
        plugin_type="xml",
        file_name=os.path.basename(relative_path),
        package_name="fake_keys",
        relative_path=relative_path,
    )


def test_xml_keys_follow_factory_rules(keys_package):
    # M06: <Default> keys are always added and the key type is case-insensitive.
    info = _xml_info()

    assert [key["name"] for key in info.input_keys] == [
        "k_in",
        "k_both",
        "k_default",
        "d_rate",
        "d_flag",
        "d_names",
    ]
    assert [key["name"] for key in info.output_keys] == ["k_out", "k_both"]
    assert all(not key["has_default"] for key in info.output_keys)

    inputs = {key["name"]: key for key in info.input_keys}
    assert inputs["k_in"] == {
        "name": "k_in",
        "description": "plain input",
        "has_default": False,
    }
    assert (
        inputs["k_both"]["default_value"],
        inputs["k_both"]["default_value_type"],
    ) == (
        7,
        "int",
    )
    assert inputs["k_default"]["default_value"] == 2.5
    assert inputs["k_default"]["default_value_type"] == "float"
    assert inputs["d_rate"]["default_value"] == 5
    assert inputs["d_flag"]["default_value"] is True
    assert inputs["d_names"]["default_value"] == ["a", "b"]
    assert inputs["d_names"]["default_value_type"] == "list[str]"

    parameters = {parameter["name"]: parameter for parameter in info.parameters}
    assert parameters["p_mode"]["default_value"] == "fast"
    assert parameters["p_gain"]["default_value"] == 0.5
    assert parameters["p_required"] == {
        "name": "p_required",
        "description": "no default",
        "has_default": False,
    }

    assert info.outcomes == ["done", "failed"]
    assert info.outcome_descriptions == {"done": "finished"}
    assert info.unique_id == "xml:fake_keys:sm/keys.xml"


def test_xml_keys_match_the_factory(fake_ws, keys_package):
    # M06: compare with what YasminFactory actually builds from the same file.
    fake_ws.use_real_packages("yasmin", "yasmin_ros", "yasmin_factory")
    fake_ws.activate()

    completed = subprocess.run(
        [sys.executable, "-c", FACTORY_SCRIPT, str(keys_package)],
        capture_output=True,
        text=True,
        timeout=120,
        env=dict(os.environ),
    )
    assert completed.returncode == 0, completed.stderr[-2000:]
    factory = json.loads(completed.stdout.strip().splitlines()[-1])

    info = _xml_info()
    for field in ("input_keys", "output_keys", "parameters"):
        expected = PluginInfo._normalize_cpp_metadata_entries(factory[field])
        assert getattr(info, field) == expected, field


def test_xml_lookup_by_file_name_sets_posix_relative_path(keys_package):
    info = PluginInfo(plugin_type="xml", file_name="keys.xml", package_name="fake_keys")
    assert info.relative_path == "sm/keys.xml"


def test_python_outcomes_are_sorted(tmp_path, monkeypatch):
    # M11: get_outcomes() returns a set whose order depends on the hash seed.
    module = tmp_path / "fake_outcomes_module.py"
    module.write_text(textwrap.dedent("""
            from yasmin import State

            class ManyOutcomes(State):
                def __init__(self):
                    super().__init__(["zeta", "alpha", "mid", "beta", "omega",
                                      "kappa", "delta", "gamma"])

                def execute(self, blackboard):
                    return "zeta"
            """))
    monkeypatch.syspath_prepend(str(tmp_path))

    info = PluginInfo(
        plugin_type="python", class_name="ManyOutcomes", module="fake_outcomes_module"
    )

    assert info.outcomes == [
        "alpha",
        "beta",
        "delta",
        "gamma",
        "kappa",
        "mid",
        "omega",
        "zeta",
    ]


def test_metadata_defaults_keep_json_values():
    # M12: list and dict defaults were replaced by an empty string.
    entries = PluginInfo._normalize_cpp_metadata_entries(
        [
            {
                "name": "vector",
                "has_default": True,
                "default_value": [1.0, 2.0],
                "default_value_type": "std::vector<double, std::allocator<double> >",
            },
            {"name": "mapping", "has_default": True, "default_value": {"a": [1, 2]}},
            {"name": "pair", "has_default": True, "default_value": (1, "x")},
            {"name": "opaque", "has_default": True, "default_value": object()},
            {"name": "int_keys", "has_default": True, "default_value": {1: 2}},
            {"name": "none", "has_default": False},
        ]
    )
    values = {entry["name"]: entry.get("default_value") for entry in entries}

    assert values == {
        "vector": [1.0, 2.0],
        "mapping": {"a": [1, 2]},
        "pair": [1, "x"],
        "opaque": "",
        "int_keys": "",
        "none": None,
    }
    assert entries[0]["default_value_type"] == "list[float]"
    json.dumps(entries)


def test_cache_round_trip_preserves_metadata(keys_package):
    info = _xml_info()
    restored = PluginInfo.from_cache_dict(json.loads(json.dumps(info.to_cache_dict())))

    assert restored.to_cache_dict() == info.to_cache_dict()
    assert restored.unique_id == info.unique_id == restored.dedup_key
