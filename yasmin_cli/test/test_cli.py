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

import itertools
import os
import signal
import subprocess
import sys
import textwrap
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from yasmin_cli.completer import _xml_path_matches
from yasmin_cli.verb.print import _render_state_machine
from yasmin_cli.verb.run import (
    _absolutize_includes,
    _find_input_key_map,
    ros_parameter_argument,
)

TEST_DIR = Path(__file__).resolve().parent
DOMAIN_IDS = itertools.count(150)


# --- Unit tests -----------------------------------------------------------


def test_input_keys_include_untyped_and_required_keys():
    root = ET.fromstring("""<StateMachine outcomes="end">
             <Key name="with_default" type="in" default_type="int" default_value="1"/>
             <Key name="required" type="in"/>
             <Key name="untyped" default_value="x"/>
             <Key name="both" type="in/out"/>
             <Key name="result" type="out"/>
           </StateMachine>""")
    assert sorted(_find_input_key_map(root)) == [
        "both",
        "required",
        "untyped",
        "with_default",
    ]


def test_relative_includes_point_to_the_original_directory(tmp_path):
    root = ET.fromstring("""<StateMachine outcomes="end">
             <StateMachine name="A" file_path="sub.xml"/>
             <StateMachine name="B" file_path="/abs/sub.xml"/>
             <State name="C" file_path="ignored.xml"/>
           </StateMachine>""")
    _absolutize_includes(root, tmp_path)
    paths = {
        e.attrib["name"]: e.attrib["file_path"] for e in root.iter() if "name" in e.attrib
    }
    assert paths == {
        "A": str(tmp_path / "sub.xml"),
        "B": "/abs/sub.xml",
        "C": "ignored.xml",
    }


@pytest.mark.parametrize("path", ["dir/sm #2.xml", "dir/mission: alpha.xml", "ü/ß.xml"])
def test_parameter_argument_value_parses_as_the_same_string(path):
    name, value = ros_parameter_argument("state_machine_file", path).split(":=", 1)
    assert name == "state_machine_file"
    # rcl parses parameter values as YAML.
    assert yaml.safe_load(value) == path


def test_print_renders_orthogonal_states_regions_and_joins():
    root = ET.fromstring("""<StateMachine name="Main" outcomes="done">
             <OrthogonalState name="parallel" default_outcome="timeout">
               <Transition from="success" to="done"/>
               <OutcomeMap outcome="success"><Item state="A" outcome="done"/></OutcomeMap>
               <Region name="A" outcomes="done" start_state="work">
                 <State name="work" type="py" module="m" class="C"/>
                 <JoinState name="sync" sync_id="s1"/>
               </Region>
             </OrthogonalState>
           </StateMachine>""")
    text = _render_state_machine(root)
    assert "OrthogonalState(name='parallel'" in text
    assert "Region(name='A'" in text
    assert "sync (JoinState, sync_id=s1, outcome=joined)" in text
    assert "success: A=done" in text


def test_xml_path_completion_follows_directories(tmp_path, monkeypatch):
    (tmp_path / "sub").mkdir()
    (tmp_path / "sm.xml").write_text('<StateMachine outcomes="end"/>')
    (tmp_path / "other.xml").write_text("<launch/>")
    monkeypatch.chdir(tmp_path / "sub")
    assert _xml_path_matches("../") == ["../sm.xml", "../sub/"]
    assert _xml_path_matches(f"{tmp_path}/s") == [
        f"{tmp_path}/sm.xml",
        f"{tmp_path}/sub/",
    ]


CHILD = textwrap.dedent("""
    import os, signal, sys, time
    if sys.argv[1] == "trap":
        signal.signal(signal.SIGTERM, lambda *_: sys.exit(3))
        signal.signal(signal.SIGINT, lambda *_: sys.exit(4))
    print("ready", os.getpid(), flush=True)
    time.sleep(30)
    """)

PARENT = textwrap.dedent("""
    import sys
    from yasmin_cli.verb.run import run_node
    sys.exit(run_node([sys.executable, "-c", sys.argv[1], sys.argv[2]]))
    """)


@pytest.mark.parametrize(
    "mode, signum, expected",
    [
        ("trap", signal.SIGTERM, 3),  # forwarded; the node's exit code is kept
        ("trap", signal.SIGINT, 4),  # SIGINT to this process alone, as launch does
        ("plain", signal.SIGTERM, 128 + signal.SIGTERM),  # killed by the signal
    ],
)
def test_run_node_forwards_signals_and_leaves_no_orphan(mode, signum, expected):
    parent = subprocess.Popen(
        [sys.executable, "-c", PARENT, CHILD, mode],
        stdout=subprocess.PIPE,
        text=True,
    )
    word, child_pid = parent.stdout.readline().split()
    assert word == "ready"
    parent.send_signal(signum)
    assert parent.wait(timeout=10) == expected
    # The node process itself must be gone, not merely reparented.
    time.sleep(0.2)
    assert not Path(f"/proc/{child_pid}").exists()


# --- End-to-end tests with the factory nodes ------------------------------


@pytest.fixture
def ros_env(tmp_path):
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(next(DOMAIN_IDS))
    env["PYTHONPATH"] = os.pathsep.join([str(TEST_DIR), env.get("PYTHONPATH", "")])
    env["CLI_TEST_OUTPUT"] = str(tmp_path / "output.txt")
    return env


def write(path: Path, content: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(textwrap.dedent(content))
    return path


def yasmin(env, *arguments, timeout=60):
    return subprocess.run(
        ["ros2", "yasmin", *arguments],
        env=env,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def state_machine(state_class: str) -> str:
    return f"""\
        <StateMachine name="M" outcomes="end" start_state="S">
          <Key name="val" type="in" default_type="int" default_value="1"/>
          <State name="S" type="py" module="cli_states" class="{state_class}">
            <Transition from="done" to="end"/>
          </State>
        </StateMachine>
        """


@pytest.mark.parametrize("factory", [[], ["--py"]])
def test_run_input_override_keeps_relative_includes(tmp_path, ros_env, factory):
    main = write(
        tmp_path / "missions" / "main.xml",
        """\
        <StateMachine name="Main" outcomes="end" start_state="SUB">
          <Key name="val" type="in" default_type="int" default_value="1"/>
          <StateMachine name="SUB" file_path="sub.xml">
            <Transition from="done" to="end"/>
          </StateMachine>
        </StateMachine>
        """,
    )
    write(
        tmp_path / "missions" / "sub.xml",
        """\
        <StateMachine name="Sub" outcomes="done" start_state="W">
          <State name="W" type="py" module="cli_states" class="WriteValue">
            <Transition from="done" to="done"/>
          </State>
        </StateMachine>
        """,
    )
    result = yasmin(
        ros_env, "run", str(main), "--input", "val=7", "--disable-viewer-pub", *factory
    )
    assert result.returncode == 0, result.stderr
    assert Path(ros_env["CLI_TEST_OUTPUT"]).read_text() == "7\n"


@pytest.mark.parametrize("factory", [[], ["--py"]])
def test_run_reports_a_failing_state_machine(tmp_path, ros_env, factory):
    xml = write(tmp_path / "fail.xml", state_machine("Fail"))
    result = yasmin(ros_env, "run", str(xml), "--disable-viewer-pub", *factory)
    assert result.returncode == 1


@pytest.mark.parametrize("factory", [[], ["--py"]])
@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
def test_run_cancels_on_signal_without_orphans(tmp_path, ros_env, factory, signum):
    xml = write(tmp_path / "wait.xml", state_machine("WaitForCancel"))
    output = Path(ros_env["CLI_TEST_OUTPUT"])
    cli = subprocess.Popen(
        ["ros2", "yasmin", "run", str(xml), "--disable-viewer-pub", *factory],
        env=ros_env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    deadline = time.monotonic() + 30
    while not (output.exists() and "waiting" in output.read_text()):
        assert time.monotonic() < deadline, "state machine did not start"
        time.sleep(0.1)
    node_pids = subprocess.run(
        ["pgrep", "-f", str(xml)], capture_output=True, text=True
    ).stdout.split()
    cli.send_signal(signum)
    assert cli.wait(timeout=15) == 130
    time.sleep(0.5)
    alive = [
        pid for pid in node_pids if int(pid) != cli.pid and Path(f"/proc/{pid}").exists()
    ]
    assert alive == []


def test_run_passes_ros_arguments(tmp_path, ros_env):
    xml = write(tmp_path / "ok.xml", state_machine("WriteValue"))
    result = yasmin(
        ros_env,
        "run",
        str(xml),
        "--disable-viewer-pub",
        "--ros-args",
        "-r",
        "__ns:=/cli_test",
        "-r",
        "__node:=cli_fsm",
    )
    assert result.returncode == 0, result.stderr
    assert "[cli_test.cli_fsm]" in result.stdout + result.stderr


def test_validate_and_run_accept_sibling_includes(tmp_path, ros_env):
    write(tmp_path / "common" / "sub.xml", state_machine("WriteValue"))
    main = write(
        tmp_path / "main" / "main.xml",
        """\
        <StateMachine name="Main" outcomes="end" start_state="SUB">
          <Key name="val" type="in" default_type="int" default_value="1"/>
          <StateMachine name="SUB" file_path="../common/sub.xml">
            <Transition from="end" to="end"/>
          </StateMachine>
        </StateMachine>
        """,
    )
    for factory in ([], ["--py"]):
        assert yasmin(ros_env, "validate", str(main), *factory).returncode == 0, factory
        run = yasmin(ros_env, "run", str(main), "--disable-viewer-pub", *factory)
        assert run.returncode == 0, (factory, run.stderr)


def test_validate_uses_the_same_factory_as_run(tmp_path, ros_env):
    # C++ stores XML ints as int and rejects this value; Python accepts it.
    xml = write(
        tmp_path / "big.xml",
        state_machine("WriteValue").replace(
            'default_value="1"', 'default_value="3000000000"'
        ),
    )
    results = {}
    for factory in ([], ["--py"]):
        validate = yasmin(ros_env, "validate", str(xml), *factory).returncode
        run = yasmin(
            ros_env, "run", str(xml), "--disable-viewer-pub", *factory
        ).returncode
        results[tuple(factory)] = (validate, run)
    assert results == {(): (1, 1), ("--py",): (0, 0)}
