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

"""XML round trips through the editor, checked against the real factory.

Every factory example XML is loaded by the editor, written back, and loaded
again with the Python ``YasminFactory``; the resulting state trees must be
identical (includes, keys, parameters, region keys, outcome maps, ...).
"""

import os
import shutil
import stat
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from yasmin_factory_test_support import (  # noqa: E402
    FACTORY_TEST_DIR,
    factory_example_xmls,
    load_with_factory,
    use_factory_test_states,
)
from yasmin_editor.io.xml_converter import (  # noqa: E402
    absolutize_include_paths,
    model_from_xml,
    model_to_xml,
)
from yasmin_editor.model.key import Key  # noqa: E402
from yasmin_editor.model.outcome import Outcome  # noqa: E402
from yasmin_editor.model.parameter import Parameter  # noqa: E402
from yasmin_editor.model.state import State  # noqa: E402
from yasmin_editor.model.state_machine import StateMachine  # noqa: E402
from yasmin_editor.model.text_block import TextBlock  # noqa: E402
from yasmin_editor.model.transition import Transition  # noqa: E402
from yasmin_editor.model.validation import validate_model  # noqa: E402


@pytest.fixture
def factory(monkeypatch):
    """Provide the real factory loader with the factory's test states."""

    pytest.importorskip("yasmin_factory")
    use_factory_test_states(monkeypatch)
    return load_with_factory


def _copy_factory_examples(tmp_path: Path) -> Path:
    target = tmp_path / "examples"
    shutil.copytree(FACTORY_TEST_DIR, target, ignore=shutil.ignore_patterns("*.py*"))
    return target


def _load_or_skip(factory, path: Path):
    try:
        return factory(path)
    except Exception as exc:  # environment cannot build the original machine
        pytest.skip(f"factory cannot load {path.name} here: {exc}")


def _roundtrip(path: Path, out_name: str = "editor_out.xml") -> Path:
    out = path.with_name(out_name)
    model_to_xml(model_from_xml(path), out)
    return out


@pytest.mark.parametrize("example", factory_example_xmls(), ids=lambda path: path.name)
def test_factory_examples_survive_an_editor_roundtrip(factory, tmp_path, example):
    examples = _copy_factory_examples(tmp_path)
    original = examples / example.name
    expected = _load_or_skip(factory, original)

    out = _roundtrip(original)

    assert factory(out) == expected
    # A second round trip must not change the editor output any further.
    assert model_to_xml(model_from_xml(out)) == model_to_xml(model_from_xml(original))


@pytest.mark.parametrize("example", factory_example_xmls(), ids=lambda path: path.name)
def test_factory_examples_pass_editor_validation(example):
    # E10: no false errors (missing start_state, unnamed root) on valid files.
    result = validate_model(model_from_xml(example))
    assert result.errors == []


def test_file_path_include_is_kept(factory, tmp_path):
    # E01: <StateMachine file_path=...> was parsed as an empty inline machine.
    examples = _copy_factory_examples(tmp_path)
    original = examples / "test_file_path_sm.xml"
    model = model_from_xml(original)
    included = model.states["IncludedSM"]

    assert not included.is_container
    assert included.state_type == "xml"
    assert included.file_path == "test_included_sm.xml"
    assert 'file_path="test_included_sm.xml"' in model_to_xml(model)
    assert factory(_roundtrip(original)) == _load_or_skip(factory, original)


def test_untyped_state_keeps_module_and_untyped_include_stays_an_include(
    factory, tmp_path
):
    # E01: a <State> without type is a Python state for the factory; a
    # <StateMachine file_name package> without type="xml" is an include.
    xml = tmp_path / "untyped.xml"
    xml.write_text("""<StateMachine name="Root" outcomes="end">
  <State name="S" module="test.test_simple_state" class="TestSimpleState">
    <Transition from="outcome1" to="Inc"/><Transition from="outcome2" to="end"/>
  </State>
  <StateMachine name="Inc" file_name="test_included_sm.xml" package="yasmin_factory">
    <Transition from="included_end" to="end"/>
  </StateMachine>
</StateMachine>""")
    model = model_from_xml(xml)
    assert model.states["S"].state_type == "py"
    assert model.states["S"].module == "test.test_simple_state"
    assert model.states["Inc"].file_name == "test_included_sm.xml"

    text = model_to_xml(model)
    assert 'module="test.test_simple_state"' in text
    assert '<StateMachine name="Inc" type="xml" file_name="test_included_sm.xml"' in text
    assert factory(_roundtrip(xml)) == _load_or_skip(factory, xml)


def test_defaults_follow_the_factory_presence_rules(factory, tmp_path):
    # E03: a default exists iff default_value is present (type defaults to str);
    # a default_type without default_value declares no default.
    xml = tmp_path / "defaults.xml"
    xml.write_text("""<StateMachine name="Root" outcomes="done">
  <Param name="greeting" default_value="hello" description="untyped default"/>
  <Param name="speed" default_type="int" description="declared, no default"/>
  <Key name="goal" type="in" default_value="home"/>
  <State name="S" type="py" module="test.test_simple_state" class="TestParameterizedState">
    <Param name="sleep_ms" default_value="5" default_type="int"/>
    <Transition from="done" to="done"/>
  </State>
</StateMachine>""")
    model = model_from_xml(xml)
    greeting, speed = model.parameters
    assert (greeting.has_default, greeting.default_type, greeting.default_value) == (
        True,
        "str",
        "hello",
    )
    assert (speed.has_default, speed.default_value) == (False, None)
    assert model.keys[0].default_type == "str"

    text = model_to_xml(model)
    assert 'default_value=""' not in text
    assert factory(_roundtrip(xml)) == _load_or_skip(factory, xml)


def test_non_string_default_values_are_written_as_factory_literals():
    # E03: str() of a Python list/dict is not the JSON the factory parses.
    type_utils = pytest.importorskip("yasmin_factory.type_utils")
    root = StateMachine(name="Root", outcomes=[Outcome("done")])
    root.parameters.append(
        Parameter(name="names", default_type="list[str]", default_value=["a", "b"])
    )
    root.keys.append(
        Key(name="gains", default_type="dict[str,float]", default_value={"kp": 1.5})
    )
    root.keys.append(Key(name="flag", default_type="bool", default_value=True))

    reloaded = model_from_xml(model_to_xml(root))
    parsed = [
        type_utils.parse_key_value(item.default_value, item.default_type)
        for item in reloaded.parameters + reloaded.keys
    ]

    assert parsed == [["a", "b"], {"kp": 1.5}, True]
    assert validate_model(reloaded).errors == []


def test_region_keys_leaf_keys_and_legacy_defaults_are_kept(factory, tmp_path):
    # E05: Keys on Regions and leaf states were dropped on save, and the legacy
    # <Default key value type> syntax became a nameless state.
    xml = tmp_path / "keys.xml"
    xml.write_text("""<StateMachine name="Root" outcomes="done fail" start_state="O">
  <Default key="legacy" value="3" type="int" description="legacy default"/>
  <OrthogonalState name="O" default_outcome="fail">
    <Transition from="ok" to="done"/>
    <Transition from="fail" to="fail"/>
    <OutcomeMap outcome="ok"><Item state="A" outcome="done"/></OutcomeMap>
    <Region name="A" outcomes="done" start_state="w">
      <Key name="region_key" type="in" default_value="7" default_type="int"/>
      <State name="w" type="py" module="test.test_simple_state" class="TestParameterizedState">
        <Key name="leaf_key" type="out" description="written by w"/>
      </State>
    </Region>
  </OrthogonalState>
</StateMachine>""")
    with _no_unsupported_warning():
        model = model_from_xml(xml)

    assert [state for state in model.states] == ["O"]
    assert [(key.name, key.default_type, key.default_value) for key in model.keys] == [
        ("legacy", "int", "3")
    ]
    region = model.states["O"].states["A"]
    assert [key.name for key in region.keys] == ["region_key"]
    assert [key.name for key in region.states["w"].keys] == ["leaf_key"]

    out = _roundtrip(xml)
    text = out.read_text()
    assert '<Key name="region_key"' in text
    assert '<Key name="leaf_key" type="out"' in text
    assert factory(out) == _load_or_skip(factory, xml)


def test_legacy_concurrence_outcome_rules_become_outcome_maps(factory, tmp_path):
    # E05: the C++-only <Outcome to=...><Transition state= outcome=/> syntax
    # was parsed as a nameless state. It is now read as an outcome-map rule.
    xml = tmp_path / "legacy_concurrence.xml"
    xml.write_text("""<StateMachine name="Root" outcomes="done" start_state="C">
  <Concurrence name="C" default_outcome="done">
    <State name="S" type="py" module="test.test_simple_state" class="TestParameterizedState"/>
    <Outcome to="ok"><Transition state="S" outcome="done"/></Outcome>
    <Transition from="ok" to="done"/>
    <Transition from="done" to="done"/>
  </Concurrence>
</StateMachine>""")
    concurrence = model_from_xml(xml).states["C"]

    assert list(concurrence.states) == ["S"]
    assert concurrence.outcome_map == {"ok": {"S": ["done"]}}

    out = _roundtrip(xml)
    assert '<OutcomeMap outcome="ok">' in out.read_text()
    fingerprint = factory(out)
    assert fingerprint["children"]["C"]["outcome_map"] == {"ok": {"S": "done"}}


def test_unknown_elements_are_skipped_with_a_warning(tmp_path):
    xml = tmp_path / "unknown.xml"
    xml.write_text("""<StateMachine name="Root" outcomes="done">
  <Bogus name="x"/>
  <State name="S" type="cpp" class="pkg/S"><Transition from="ok" to="done"/></State>
</StateMachine>""")
    with pytest.warns(UserWarning, match="Bogus"):
        model = model_from_xml(xml)
    assert list(model.states) == ["S"]


def test_saving_strips_characters_xml_cannot_represent(tmp_path):
    # E12: control characters made the saved file unreadable.
    root = StateMachine(name="Root", outcomes=[Outcome("end")])
    root.description = "bell\x07 and escape \x1b[31mred\x1b[0m"
    root.add_text_block(TextBlock(x=0, y=0, content="log: \x1b[31mERROR\x1b[0m\x00"))
    target = tmp_path / "ctrl.xml"

    model_to_xml(root, target)
    reloaded = model_from_xml(target)

    assert reloaded.description == "bell and escape [31mred[0m"
    assert reloaded.text_blocks[0].content == "log: [31mERROR[0m"


def test_saving_through_a_symlink_updates_the_target_and_keeps_its_mode(tmp_path):
    # E12: os.replace() replaced the symlink (e.g. colcon --symlink-install)
    # with a regular file and reset the permissions.
    source = tmp_path / "src" / "sm.xml"
    source.parent.mkdir()
    model_to_xml(StateMachine(name="Before", outcomes=[Outcome("end")]), source)
    os.chmod(source, 0o640)
    link = tmp_path / "install" / "sm.xml"
    link.parent.mkdir()
    link.symlink_to(source)

    model_to_xml(StateMachine(name="After", outcomes=[Outcome("end")]), link)

    assert link.is_symlink()
    assert model_from_xml(source).name == "After"
    assert stat.S_IMODE(source.stat().st_mode) == 0o640


def test_absolutize_include_paths_rewrites_only_relative_includes(tmp_path):
    root = StateMachine(name="Root", outcomes=[Outcome("end")])
    root.add_state(State(name="Rel", state_type="xml", file_path="sub/inc.xml"))
    root.add_state(State(name="Abs", state_type="xml", file_path="/abs/inc.xml"))
    nested = StateMachine(name="Nested", outcomes=[Outcome("end")])
    nested.add_state(State(name="Deep", state_type="xml", file_path="../up.xml"))
    root.add_state(nested)
    root.add_transition("Rel", Transition("done", "end"))

    absolutize_include_paths(root, tmp_path / "doc")

    assert root.states["Rel"].file_path == str(tmp_path / "doc" / "sub" / "inc.xml")
    assert root.states["Abs"].file_path == "/abs/inc.xml"
    assert nested.states["Deep"].file_path == str(tmp_path / "up.xml")


class _no_unsupported_warning:
    """Context manager failing on 'unsupported XML element' warnings."""

    def __enter__(self):
        import warnings

        self._catcher = warnings.catch_warnings(record=True)
        self._records = self._catcher.__enter__()
        warnings.simplefilter("always")
        return self

    def __exit__(self, *exc_info):
        self._catcher.__exit__(*exc_info)
        unsupported = [r for r in self._records if "unsupported XML" in str(r.message)]
        assert unsupported == []
        return False


@pytest.mark.parametrize("fixture_name", ["test.xml", "runtime_test.xml"])
def test_editor_fixture_files_roundtrip_stably(fixture_name):
    # The editor's own example documents (layouts, aliases, text blocks,
    # includes by package) must not change when re-saved.
    path = Path(__file__).resolve().parent / fixture_name
    first = model_to_xml(model_from_xml(path))
    assert model_to_xml(model_from_xml(first)) == first
