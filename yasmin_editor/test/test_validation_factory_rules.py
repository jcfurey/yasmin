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

"""Validation must accept what the factory accepts and reject what it rejects."""

from yasmin_editor.io.xml_converter import model_from_xml
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.join_state import JoinState
from yasmin_editor.model.key import Key
from yasmin_editor.model.orthogonal_state import OrthogonalState
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.parameter import Parameter
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.transition import Transition
from yasmin_editor.model.validation import validate_model


def cpp_state(name: str, outcomes=("ok", "fail"), **kwargs) -> State:
    return State(
        name=name,
        state_type="cpp",
        class_name="pkg/State",
        outcomes=[Outcome(item) for item in outcomes],
        **kwargs,
    )


def messages(result, kind: str = "errors"):
    return {f"{item.path}: {item.message}" for item in getattr(result, kind)}


def test_missing_start_state_and_unnamed_root_are_valid():
    # E10: the factory starts with the first added state and allows an unnamed root.
    root = StateMachine(name="", outcomes=[Outcome("end")])
    root.add_state(cpp_state("A"))
    root.add_state(cpp_state("B"))
    root.add_transition("A", Transition("ok", "B"))
    root.add_transition("A", Transition("fail", "end"))
    root.add_transition("B", Transition("ok", "end"))
    root.add_transition("B", Transition("fail", "end"))

    result = validate_model(root)

    assert result.errors == []
    assert result.warnings == []


def test_unreachable_states_are_errors():
    # E11: yasmin rejects states that cannot be reached from the start state.
    root = StateMachine(name="R", outcomes=[Outcome("end")], start_state="A")
    root.add_state(cpp_state("A"))
    root.add_state(cpp_state("B"))
    root.add_transition("A", Transition("ok", "end"))

    assert "R: State 'B' is unreachable from start state 'A'" in messages(
        validate_model(root)
    )


def test_parameter_remaps_must_reference_declared_parameters():
    # E11: ParamRemap to a parameter the container does not declare fails at
    # configure() time; a nested container must declare the remapped parameter.
    root = StateMachine(
        name="R",
        outcomes=[Outcome("end")],
        parameters=[Parameter(name="declared", default_type="int", default_value="1")],
    )
    root.add_state(cpp_state("A", parameter_mappings={"speed": "not_declared"}))
    nested = StateMachine(
        name="N",
        outcomes=[Outcome("end")],
        parameter_mappings={"missing_child_param": "declared"},
    )
    nested.add_state(cpp_state("I"))
    root.add_state(nested)
    root.add_transition("A", Transition("ok", "N"))

    errors = messages(validate_model(root))

    assert "R/A: Parameter remap target 'not_declared' is not declared by 'R'" in errors
    assert (
        "R/N: Parameter remap source 'missing_child_param' is not declared by 'N'"
        in errors
    )


def test_defaults_must_parse_for_their_type():
    # E11: these defaults make the factory raise while loading the file.
    root = StateMachine(
        name="R",
        outcomes=[Outcome("end")],
        parameters=[
            Parameter(name="count", default_type="int", default_value=""),
            Parameter(name="ids", default_type="list[int]", default_value="1, 2"),
            Parameter(name="rate", default_type="double", default_value="0.5"),
        ],
        keys=[
            Key(name="flag", default_type="bool", default_value="maybe"),
            Key(name="kind", default_type="complex", default_value="1j"),
            Key(name="out_only", key_type="out", default_type="int", default_value="x"),
        ],
    )
    root.add_state(cpp_state("A"))

    errors = " | ".join(sorted(messages(validate_model(root))))

    assert "Parameter 'count' has an invalid default value" in errors
    assert "Parameter 'ids' has an invalid default value" in errors
    assert "Parameter 'rate'" not in errors  # "double" is a factory alias of float
    assert "Key 'flag' has an invalid default value" in errors
    assert "Unsupported default_type 'complex'" in errors
    assert "out_only" not in errors  # the factory ignores defaults of output keys


def test_outcome_names_with_whitespace_are_errors():
    # E11: container outcomes are stored space-separated in XML.
    root = StateMachine(name="R", outcomes=[Outcome("task done")])
    root.add_state(cpp_state("A"))
    root.add_transition("A", Transition("ok", "task done"))

    assert "R: Outcome name 'task done' must not contain whitespace" in messages(
        validate_model(root)
    )


def test_remaps_on_concurrence_children_and_regions_are_errors():
    # E11: yasmin's Concurrence/OrthogonalState take no blackboard remappings.
    concurrence = Concurrence(name="C", default_outcome="done")
    concurrence.add_outcome(Outcome("done"))
    concurrence.add_state(cpp_state("S", remappings={"input": "data"}))
    orthogonal = OrthogonalState(name="O", default_outcome="done")
    orthogonal.add_outcome(Outcome("done"))
    region = StateMachine(
        name="A",
        outcomes=[Outcome("finished")],
        remappings={"a": "b"},
        parameter_mappings={"p": "q"},
    )
    region.add_state(cpp_state("w"))
    orthogonal.add_state(region)
    root = StateMachine(name="R", outcomes=[Outcome("end")])
    root.add_state(concurrence)
    root.add_state(orthogonal)
    root.add_transition("C", Transition("done", "O"))

    errors = messages(validate_model(root))

    assert (
        "R/C/S: Blackboard remappings on a child state of a concurrence are ignored "
        "by YASMIN" in errors
    )
    assert (
        "R/O/A: Blackboard remappings on a region of a orthogonal state are ignored "
        "by YASMIN" in errors
    )
    assert "R/O/A: Parameter remappings on a region are ignored by YASMIN" in errors


def test_xml_includes_need_a_resolvable_reference():
    # E11: file_name without package cannot be resolved by the factory.
    root = StateMachine(name="R", outcomes=[Outcome("end")])
    root.add_state(State(name="ByName", state_type="xml", file_name="sub.xml"))
    root.add_state(State(name="ByPath", state_type="xml", file_path="sub.xml"))
    root.add_state(State(name="Odd", state_type="lua", class_name="x"))
    root.add_transition("ByName", Transition("done", "ByPath"))
    root.add_transition("ByPath", Transition("done", "Odd"))

    result = validate_model(root)
    errors = messages(result)

    assert (
        "R/ByName: XML state with 'file_name' requires 'package_name' "
        "(or use 'file_path')" in errors
    )
    assert not any(item.startswith("R/ByPath:") for item in errors)
    assert "R/Odd: Unknown state type 'lua'" in errors


def test_transitions_must_use_outcomes_the_child_produces():
    # E04: a nested container or join state that no longer produces an outcome
    # makes the factory reject the parent's transition.
    nested = StateMachine(name="N", outcomes=[Outcome("a")])
    nested.add_state(cpp_state("I"))
    nested.add_transition("I", Transition("ok", "a"))
    nested.add_transition("I", Transition("fail", "a"))
    root = StateMachine(name="R", outcomes=[Outcome("end")], start_state="N")
    root.add_state(nested)
    root.add_state(JoinState(name="J", join_outcome="synced"))
    root.add_transition("N", Transition("a", "J"))
    root.add_transition("N", Transition("b", "end"))
    root.add_transition("J", Transition("joined", "end"))

    errors = messages(validate_model(root))

    assert "R/N: Transition uses unknown outcome 'b'" in errors
    assert "R/J: Transition uses unknown outcome 'joined'" in errors


def test_one_outcome_with_two_transitions_is_an_error():
    root = StateMachine(name="R", outcomes=[Outcome("end"), Outcome("other")])
    root.add_state(cpp_state("A"))
    root.add_transition("A", Transition("ok", "end"))
    root.add_transition("A", Transition("ok", "other"))

    assert "R/A: Outcome 'ok' has more than one transition" in messages(
        validate_model(root)
    )


def test_outcome_map_rule_with_several_outcomes_for_one_state_is_an_error():
    # E08: the factory keeps only the last <Item> per state in an OutcomeMap.
    concurrence = Concurrence(name="C", default_outcome="other")
    concurrence.add_outcome(Outcome("other"))
    concurrence.add_state(cpp_state("S"))
    concurrence.set_outcome_rule("ok", "S", "ok")
    concurrence.set_outcome_rule("ok", "S", "fail")
    root = StateMachine(name="R", outcomes=[Outcome("end")])
    root.add_state(concurrence)

    assert (
        "R/C/S: Outcome map rule 'ok' lists several outcomes (ok, fail); YASMIN keeps "
        "only one per state" in messages(validate_model(root))
    )


def test_join_states_do_not_warn_about_a_missing_state_type():
    root = model_from_xml("""<StateMachine name="R" outcomes="done" start_state="O">
  <OrthogonalState name="O" default_outcome="done">
    <Transition from="done" to="done"/>
    <Region name="A" outcomes="joined" start_state="J">
      <JoinState name="J" sync_id="s" outcome="joined"/>
    </Region>
  </OrthogonalState>
</StateMachine>""")

    result = validate_model(root)

    assert result.errors == []
    assert result.warnings == []
