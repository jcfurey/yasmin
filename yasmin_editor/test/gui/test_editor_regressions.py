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

"""Regression tests for editor-window flows that used to lose or corrupt data."""

import sys
from pathlib import Path

import pytest

pytest.importorskip("yasmin_editor.qt_compat")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from yasmin_factory_test_support import (  # noqa: E402
    FACTORY_TEST_DIR,
    factory_load_error,
)
from yasmin_editor.io.xml_converter import model_from_xml  # noqa: E402
from yasmin_editor.model.join_state import JoinState  # noqa: E402
from yasmin_editor.model.outcome import Outcome  # noqa: E402
from yasmin_editor.model.state import State  # noqa: E402
from yasmin_editor.model.state_machine import StateMachine  # noqa: E402
from yasmin_editor.model.transition import Transition  # noqa: E402
from yasmin_editor.model.validation import validate_model  # noqa: E402


def load(editor, tmp_path, xml: str, name: str = "in.xml") -> Path:
    path = tmp_path / name
    path.write_text(xml)
    editor.load_from_xml(str(path))
    return path


def assert_factory_loads(path: Path) -> None:
    """Load *path* with the real factory, skipping when it is unavailable."""

    if not (FACTORY_TEST_DIR / "test_simple_state.py").is_file():
        pytest.skip("yasmin_factory test states are not available")
    error = factory_load_error(path)
    if error and ("No module named" in error or "MultiLibraryClassLoader" in error):
        pytest.skip(f"factory unavailable here: {error}")
    assert error is None, error


METADATA_XML = """<StateMachine name="Root" outcomes="done">
  <Param name="speed" description="max speed" default_type="float" default_value="1.5"/>
  <Key name="result" type="out" description="written by S"/>
  <Key name="goal" type="in" description="provided by caller"/>
  <State name="S" type="py" module="test.test_simple_state" class="TestParameterizedState">
    <Transition from="done" to="done"/>
  </State>
</StateMachine>"""


def test_adding_and_deleting_states_keeps_container_keys_and_params(
    editor_window, tmp_path
):
    # E02: every add/delete/edit synced the blackboard and dropped keys without
    # defaults plus every parameter no child mapped.
    editor = editor_window
    load(editor, tmp_path, METADATA_XML)

    editor.add_model_state(JoinState(name="J"), x=0.0, y=0.0)
    editor.delete_state_item(editor.state_nodes["J"])
    out = tmp_path / "out.xml"
    editor.save_to_xml(str(out))

    saved = model_from_xml(out)
    assert [parameter.name for parameter in saved.parameters] == ["speed"]
    assert sorted((key.name, key.key_type) for key in saved.keys) == [
        ("goal", "in"),
        ("result", "out"),
    ]


def test_key_and_parameter_dialogs_accept_factory_type_aliases(qapp):
    # E03: "double", "String", ... are valid factory types but were reset to
    # "No default", so pressing OK erased the default.
    from yasmin_editor.editor_gui.dialogs.blackboard_key_dialog import (
        BlackboardKeyDialog,
    )
    from yasmin_editor.editor_gui.dialogs.parameter_overwrite_dialog import (
        ParameterOverwriteDialog,
    )

    key_dialog = BlackboardKeyDialog(
        {
            "name": "rate",
            "key_type": "in",
            "description": "",
            "default_type": "double",
            "default_value": "3.5",
        },
        edit_mode=True,
    )
    assert key_dialog.get_key_data()["default_type"] == "float"
    assert key_dialog.get_key_data()["default_value"] == "3.5"

    parameter_dialog = ParameterOverwriteDialog(
        declared_parameters=[{"name": "label"}],
        param_data={
            "name": "label",
            "child_parameter": "label",
            "default_type": " String ",
            "default_value": "hi",
        },
    )
    data = parameter_dialog.get_parameter_data()
    assert (data["default_type"], data["default_value"]) == ("str", "hi")


NESTED_XML = """<StateMachine name="Root" outcomes="end">
  <StateMachine name="N" outcomes="a b" start_state="I">
    <State name="I" type="cpp" class="yasmin_factory/TestSimpleState">
      <Transition from="outcome1" to="a"/><Transition from="outcome2" to="a"/>
    </State>
    <Transition from="a" to="end"/>
    <Transition from="b" to="end"/>
  </StateMachine>
</StateMachine>"""


def test_deleting_a_nested_final_outcome_removes_the_parent_transition(
    editor_window, tmp_path, qapp
):
    # E04: the parent kept <Transition from="b">, which the factory rejects.
    editor = editor_window
    load(editor, tmp_path, NESTED_XML)
    editor.enter_container(editor.state_nodes["N"])
    qapp.processEvents()

    editor.delete_final_outcome_item(editor.get_primary_final_outcome_view("b"))
    editor.navigate_up_one_level()
    qapp.processEvents()

    assert [t.source_outcome for t in editor.root_model.transitions["N"]] == ["a"]
    assert validate_model(editor.root_model).errors == []
    out = tmp_path / "out.xml"
    editor.save_to_xml(str(out))
    assert_factory_loads(out)


def test_renaming_a_join_outcome_renames_its_transitions(
    editor_window, tmp_path, qapp, monkeypatch
):
    # E04: renaming the join outcome left <Transition from="joined">.
    import yasmin_editor.editor_gui.editor_mixin.editor_ui_mixin as ui_mixin

    class RenameJoinDialog:
        def __init__(self, **_kwargs):
            pass

        def get_join_state_data(self):
            return ("J", "s", "synced", "")

    editor = editor_window
    load(
        editor,
        tmp_path,
        """<StateMachine name="Root" outcomes="done" start_state="J">
  <JoinState name="J" sync_id="s" outcome="joined"><Transition from="joined" to="done"/></JoinState>
</StateMachine>""",
    )
    monkeypatch.setattr(ui_mixin, "JoinStateDialog", RenameJoinDialog)
    monkeypatch.setattr(ui_mixin, "exec_dialog", lambda _dialog: True)
    node = editor.state_nodes["J"]
    editor.canvas.scene.clearSelection()
    node.setSelected(True)

    editor.edit_state()
    qapp.processEvents()

    assert [(t.source_outcome, t.target) for t in editor.root_model.transitions["J"]] == [
        ("synced", "done")
    ]
    assert [c.outcome for c in editor.state_nodes["J"].connections] == ["synced"]
    assert validate_model(editor.root_model).errors == []


def test_extract_selection_keeps_one_outcome_and_its_transition(
    editor_window, tmp_path, qapp, monkeypatch
):
    # E09: Extract added the outcomes first, so the paste renamed them to
    # "done2" while the transition still targeted "done".
    import yasmin_editor.editor_gui.editor_mixin.editor_clipboard_mixin as clipboard

    editor = editor_window
    load(
        editor,
        tmp_path,
        """<StateMachine name="Root" outcomes="done" start_state="A">
  <State name="A" type="cpp" class="pkg/A" x="0" y="0"><Transition from="ok" to="B"/></State>
  <State name="B" type="cpp" class="pkg/B" x="200" y="0"><Transition from="ok" to="done"/></State>
  <FinalOutcome name="done" x="400" y="0"/>
</StateMachine>""",
    )
    editor.canvas.scene.clearSelection()
    editor.state_nodes["B"].setSelected(True)
    for view in editor.final_outcomes.values():
        view.setSelected(True)
    monkeypatch.setattr(
        clipboard.QtWidgets.QInputDialog,
        "getText",
        staticmethod(lambda *_args, **_kwargs: ("Sub", True)),
    )

    editor.extract_selected_items()
    qapp.processEvents()

    sub = editor.root_model.states["Sub"]
    assert [outcome.name for outcome in sub.outcomes] == ["done"]
    (transition,) = sub.transitions["B"]
    assert transition.target == "done"
    # Every visible outcome placement belongs to the outcome the transition uses.
    assert {p.outcome_name for p in sub.layout.get_outcome_placements()} == {"done"}


def test_saving_a_valid_factory_file_does_not_prompt_and_reports_warnings(
    editor_window, tmp_path, monkeypatch
):
    # E10: an unnamed root without start_state triggered "Cannot save ...".
    import yasmin_editor.editor_gui.editor_mixin.editor_model_mixin as model_mixin

    editor = editor_window
    load(
        editor,
        tmp_path,
        """<StateMachine outcomes="end">
  <State name="State1" type="cpp" class="yasmin_factory/TestSimpleState">
    <Transition from="outcome1" to="State2"/><Transition from="outcome2" to="end"/>
  </State>
  <State name="State2" type="cpp" class="yasmin_factory/TestSimpleState">
    <Transition from="outcome1" to="Empty"/><Transition from="outcome2" to="end"/>
  </State>
  <StateMachine name="Empty" outcomes="end"><Transition from="end" to="end"/></StateMachine>
</StateMachine>""",
    )
    dialogs = []
    monkeypatch.setattr(
        model_mixin.QtWidgets.QMessageBox,
        "critical",
        staticmethod(lambda *args: dialogs.append(args)),
    )

    assert editor.save_state_machine() is True
    assert dialogs == []
    # Warnings (here: the empty nested machine) are surfaced in the status bar.
    message = editor.statusBar().currentMessage()
    assert "validation warning" in message
    assert "State machine has no child states" in message


def test_closing_the_window_cancels_a_running_machine(editor_window, qapp):
    # E13: closeEvent marked the runtime disposed first, so shutdown() returned
    # early and never canceled or joined the running machine.
    runtime_module = sys.modules["yasmin_editor.runtime.runtime"]
    canceled = []

    class RunningStateMachine(runtime_module.StateMachine):
        def cancel_state_machine(self):
            canceled.append(True)

        def cancel_state(self):
            canceled.append(True)

    editor = editor_window
    runtime = runtime_module.Runtime()
    runtime.sm = RunningStateMachine()
    runtime._running = True
    editor.runtime = runtime

    editor.close()
    qapp.processEvents()

    assert canceled
    assert runtime.sm is None


def test_runtime_log_view_is_bounded_and_refreshes_are_coalesced(editor_window, qapp):
    # E15: unbounded log view and a full control refresh per state change.
    from yasmin_editor.editor_gui.runtime_ui import RUNTIME_LOG_VIEW_MAX_BLOCKS

    editor = editor_window
    assert (
        editor.runtime_log_view.document().maximumBlockCount()
        == RUNTIME_LOG_VIEW_MAX_BLOCKS
    )

    calls = []
    editor.update_runtime_actions = lambda: calls.append(True)
    for index in range(100):
        editor.on_runtime_active_state_changed(("S", str(index)))
    assert calls == []
    qapp.processEvents()
    assert calls == [True]

    # Log lines are appended in batches (one document edit per event-loop
    # turn) and a backlog never exceeds the view limit.
    editor.clear_runtime_log_view()
    for index in range(RUNTIME_LOG_VIEW_MAX_BLOCKS + 10):
        editor.append_runtime_log(f"[INFO] line {index}")
    assert editor.runtime_log_view.document().toPlainText() == ""
    qapp.processEvents()
    document = editor.runtime_log_view.document()
    assert document.blockCount() == RUNTIME_LOG_VIEW_MAX_BLOCKS
    assert document.lastBlock().text().endswith(f"line {RUNTIME_LOG_VIEW_MAX_BLOCKS + 9}")


def test_runtime_snapshot_resolves_relative_includes_from_the_document(
    editor_window, tmp_path
):
    # The runtime snapshot lives in the temp dir; a relative file_path include
    # would resolve against it instead of the document's directory.
    editor = editor_window
    document = tmp_path / "doc" / "main.xml"
    document.parent.mkdir()
    root = StateMachine(name="Root", outcomes=[Outcome("end")])
    root.add_state(State(name="Inc", state_type="xml", file_path="sub/inc.xml"))
    root.add_transition("Inc", Transition("done", "end"))
    editor.model_adapter.apply_root_model(root, current_file_path=str(document))

    snapshot = Path(editor.create_runtime_xml_snapshot())
    try:
        included = model_from_xml(snapshot).states["Inc"]
        assert included.file_path == str(tmp_path / "doc" / "sub" / "inc.xml")
        assert editor.root_model.states["Inc"].file_path == "sub/inc.xml"
    finally:
        editor._delete_runtime_snapshot()


def test_entering_a_relative_file_path_include_resolves_from_the_document(
    editor_window, tmp_path
):
    editor = editor_window
    included = tmp_path / "doc" / "sub" / "inc.xml"
    included.parent.mkdir(parents=True)
    included.write_text('<StateMachine outcomes="done"/>')
    root = StateMachine(name="Root", outcomes=[Outcome("end")])
    root.add_state(State(name="Inc", state_type="xml", file_path="sub/inc.xml"))
    editor.model_adapter.apply_root_model(
        root, current_file_path=str(tmp_path / "doc" / "main.xml")
    )

    resolved = editor._resolve_xml_state_file_path(editor.state_nodes["Inc"])

    assert resolved == str(included)


def test_dialogs_show_typed_plugin_defaults_as_factory_literals(qapp):
    # Plugin metadata may hold list/dict/falsy defaults; the dialogs must show
    # (and return) text the factory can parse, not Python reprs.
    from yasmin_editor.editor_gui.dialogs.blackboard_key_dialog import (
        BlackboardKeyDialog,
    )
    from yasmin_editor.editor_gui.dialogs.parameter_overwrite_dialog import (
        ParameterOverwriteDialog,
    )

    parameter_dialog = ParameterOverwriteDialog(
        declared_parameters=[{"name": "names"}],
        param_data={
            "name": "names",
            "child_parameter": "names",
            "default_type": "list[str]",
            "default_value": ["a", "b"],
        },
    )
    assert parameter_dialog.get_parameter_data()["default_value"] == '["a","b"]'

    key_dialog = BlackboardKeyDialog(
        {
            "name": "count",
            "key_type": "in",
            "description": "",
            "default_type": "int",
            "default_value": 0,
        },
        edit_mode=True,
    )
    assert key_dialog.get_key_data()["default_value"] == "0"
