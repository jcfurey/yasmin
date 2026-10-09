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

"""Editor operations must not silently drop or redirect user data."""

from types import SimpleNamespace

import pytest

from yasmin_editor.editor_gui.blackboard_logic import (
    dicts_to_keys,
    keys_to_dicts,
    merge_container_keys,
    metadata_map_to_keys,
)
from yasmin_editor.editor_gui.model_parameters import apply_parameter_overwrites
from yasmin_editor.editor_gui.selection_bundle_collect import collect_selection_bundle
from yasmin_editor.editor_gui.selection_bundle_paste import paste_bundle_into_model
from yasmin_editor.editor_gui.transition_rules import (
    TransitionRuleError,
    ensure_single_outcome_rule,
)
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.key import Key
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.parameter import Parameter
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.transition import Transition


def plugin_with_keys(inputs=(), outputs=()):
    return SimpleNamespace(
        input_keys=[{"name": name, "description": ""} for name in inputs],
        output_keys=[{"name": name, "description": ""} for name in outputs],
    )


def test_merge_keeps_declared_keys_without_defaults_or_usage():
    # E02: syncing after any add/delete/edit dropped every key without a default
    # that no resolved plugin used (e.g. all documented 'out' keys).
    root = StateMachine(
        name="root",
        keys=[
            Key(name="root_result", key_type="out", description="Root output"),
            Key(name="goal", key_type="in", description="provided by caller"),
            Key(name="stale", key_type="in", derived=True),
        ],
    )
    root.add_state(State(name="worker", state_type="cpp", class_name="pkg/W"))

    merged = merge_container_keys(root, lambda _model: plugin_with_keys(outputs=["goal"]))

    assert set(merged) == {"root_result", "goal"}
    assert merged["root_result"]["key_type"] == "out"
    assert merged["root_result"]["description"] == "Root output"
    # Declared 'in' + derived 'out' usage is widened, not overwritten.
    assert merged["goal"]["key_type"] == "in/out"
    assert "derived" not in merged["goal"]


def test_derived_keys_are_tracked_and_dropped_when_unused():
    root = StateMachine(name="root")
    root.add_state(State(name="worker", state_type="cpp", class_name="pkg/W"))

    merged = merge_container_keys(root, lambda _model: plugin_with_keys(inputs=["auto"]))
    assert merged["auto"]["derived"] is True

    # The flag survives the dict round trips used by the blackboard sidebar.
    root.keys = metadata_map_to_keys(merged)
    assert root.keys[0].derived is True
    assert dicts_to_keys(keys_to_dicts(root.keys))[0].derived is True

    # Once the usage disappears, the editor-derived key goes away again.
    root.remove_state("worker")
    assert merge_container_keys(root, lambda _model: plugin_with_keys()) == {}


def test_parameter_overwrites_keep_declarations_no_child_released():
    # E02: adding or editing any child pruned container parameters that no
    # child mapped yet (e.g. loaded from XML).
    container = StateMachine(
        name="root",
        parameters=[Parameter(name="speed", default_type="float", default_value="1.5")],
    )
    child = State(name="worker", state_type="cpp", class_name="pkg/W")
    container.add_state(child)

    apply_parameter_overwrites(container, child, [])
    assert [parameter.name for parameter in container.parameters] == ["speed"]

    # A declaration released by this child is still cleaned up.
    apply_parameter_overwrites(
        container,
        child,
        [{"name": "speed", "child_parameter": "max_speed"}],
    )
    apply_parameter_overwrites(container, child, [])
    assert container.parameters == []


def test_pasted_transitions_follow_renamed_outcomes():
    # E09: a pasted outcome was renamed ("done" -> "done2") but the transition
    # kept targeting "done" while pointing at the "done2" placement.
    root = StateMachine(name="Root", start_state="S")
    root.add_state(State(name="S", state_type="cpp", class_name="pkg/S"))
    root.layout.set_state_position("S", 0, 0)
    root.add_outcome(Outcome("done"))
    instance_id = root.layout.create_outcome_alias("done", 300, 0)
    root.add_transition("S", Transition("succeeded", "done", instance_id))

    bundle = collect_selection_bundle(root, {"S"}, {instance_id}, [])
    paste_bundle_into_model(root, bundle, 0, 400)

    (pasted,) = root.transitions["S2"]
    placement = root.layout.get_outcome_placement(pasted.target_instance_id)
    assert pasted.target == "done2"
    assert placement is not None and placement.outcome_name == "done2"


def test_single_outcome_rule_guard():
    # E08: Concurrence outcome maps keep one outcome per state and final outcome.
    concurrence = Concurrence(name="cc")
    ensure_single_outcome_rule(concurrence, "worker", "finished", ["done"])
    concurrence.set_outcome_rule("finished", "worker", "done")
    ensure_single_outcome_rule(concurrence, "worker", "finished", ["done"])

    with pytest.raises(TransitionRuleError):
        ensure_single_outcome_rule(concurrence, "worker", "finished", ["failed"])
    with pytest.raises(TransitionRuleError):
        ensure_single_outcome_rule(Concurrence(name="x"), "w", "f", ["a", "b"])
    # State machines map each outcome to its own transition, so no restriction.
    ensure_single_outcome_rule(StateMachine(name="sm"), "w", "f", ["a", "b"])


def test_remove_child_state_outcome_cleans_parent_links():
    # E04: helper used when a nested container loses an outcome.
    sm = StateMachine(name="root")
    sm.add_state(State(name="N"))
    sm.add_transition("N", Transition("a", "x"))
    sm.add_transition("N", Transition("b", "y"))
    sm.remove_child_state_outcome("N", "b")
    assert [t.source_outcome for t in sm.transitions["N"]] == ["a"]

    concurrence = Concurrence(name="cc")
    concurrence.add_state(State(name="N"))
    concurrence.set_outcome_rule("ok", "N", "a")
    concurrence.set_outcome_rule("bad", "N", "b")
    concurrence.remove_child_state_outcome("N", "b")
    assert concurrence.outcome_map == {"ok": {"N": ["a"]}}


@pytest.mark.parametrize(
    "value, default_type, expected",
    [
        (["a", "b"], "list[str]", '["a","b"]'),
        ({"kp": 1.5, "on": True}, "dict[str,float]", '{"kp":1.5,"on":true}'),
        ([1, 2], "", "[1,2]"),
        (True, "bool", "true"),
        (0, "int", "0"),
        (0.0, "float", "0.0"),
        ("['kept', 'as typed']", "str", "['kept', 'as typed']"),
        (None, "int", ""),
    ],
)
def test_default_values_are_formatted_like_the_factory_expects(
    value, default_type, expected
):
    # Plugin metadata now carries list/dict defaults; str() produced Python
    # reprs the factory cannot parse, and str(x or "") dropped 0/False.
    from yasmin_editor.model.value_types import format_default_value

    assert format_default_value(value, default_type) == expected


def test_typed_plugin_defaults_survive_editor_rows_and_reach_the_factory():
    from yasmin_editor.editor_gui.state_properties_logic import (
        normalize_parameter_overwrite_row,
    )
    from yasmin_editor.io.xml_converter import model_from_xml, model_to_xml
    from yasmin_editor.model.validation import validate_model

    type_utils = pytest.importorskip("yasmin_factory.type_utils")

    # Key rows used by the blackboard sidebar.
    (key,) = dicts_to_keys(
        [{"name": "ids", "default_type": "list[int]", "default_value": [1, 2]}]
    )
    (zero,) = dicts_to_keys([{"name": "n", "default_type": "int", "default_value": 0}])
    assert (key.default_value, zero.default_value) == ("[1,2]", "0")
    row = keys_to_dicts(
        [Key(name="m", default_type="dict[str,int]", default_value={"a": 1})]
    )
    assert row[0]["default_value"] == '{"a":1}'

    # Parameter-overwrite rows filled from plugin metadata.
    overwrite = normalize_parameter_overwrite_row(
        {
            "name": "names",
            "child_parameter": "names",
            "default_type": "list[str]",
            "default_value": ["a", "b"],
        }
    )
    assert overwrite["default_value"] == '["a","b"]'

    container = StateMachine(name="root", outcomes=[Outcome("done")])
    child = State(name="worker", state_type="cpp", class_name="pkg/W")
    container.add_state(child)
    apply_parameter_overwrites(container, child, [overwrite])
    container.keys = [key, zero]

    reloaded = model_from_xml(model_to_xml(container))
    assert validate_model(reloaded).errors == []
    assert [
        type_utils.parse_key_value(item.default_value, item.default_type)
        for item in reloaded.parameters + reloaded.keys
    ] == [["a", "b"], [1, 2], 0]
