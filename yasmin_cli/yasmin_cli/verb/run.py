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

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Union

from ros2run.api import PackageNotFound, get_executable_path

from yasmin_factory.type_utils import (
    format_default_value,
    normalize_type,
    parse_key_value,
)

from yasmin_cli.completer import (
    run_input_completer,
    run_param_completer,
    strip_namespace,
    xml_file_completer,
)
from yasmin_cli.verb._xml_utils import indent_xml, parse_assignments

INPUT_KEY_TYPES = {"in", "in/out"}


def _find_input_key_map(root: ET.Element) -> Dict[str, ET.Element]:
    """Root keys that --input may set, including required ones (no default)."""
    result: Dict[str, ET.Element] = {}

    for child in root:
        if strip_namespace(child.tag) != "Key":
            continue

        # The factories treat a key without a type as an input key.
        key_type = (child.attrib.get("type") or "in").strip().lower()
        if key_type not in INPUT_KEY_TYPES:
            continue

        key_name = child.attrib.get("name", "").strip()
        if not key_name:
            continue

        result[key_name] = child

    return result


def _find_parameter_map(root: ET.Element) -> Dict[str, ET.Element]:
    result: Dict[str, ET.Element] = {}

    for child in root:
        if strip_namespace(child.tag) != "Param":
            continue

        parameter_name = child.attrib.get("name", "").strip()
        if not parameter_name:
            continue

        result[parameter_name] = child

    return result


def _find_default_element(root: ET.Element, key_name: str) -> Union[ET.Element, None]:
    for child in root:
        if strip_namespace(child.tag) != "Default":
            continue
        if child.attrib.get("key", "").strip() == key_name:
            return child
    return None


def _inject_overrides_into_root(
    root: ET.Element,
    provided_inputs: Dict[str, str],
    provided_parameters: Dict[str, str],
) -> str:
    input_key_map = _find_input_key_map(root)
    parameter_map = _find_parameter_map(root)

    for key_name, raw_value in provided_inputs.items():
        key_element = input_key_map[key_name]
        default_type = key_element.attrib.get("default_type", "").strip() or "str"
        typed_value = parse_key_value(raw_value, default_type)
        serialized_value = format_default_value(typed_value, default_type)

        key_element.set("default_value", serialized_value)
        key_element.set("default_type", normalize_type(default_type))

        default_element = _find_default_element(root, key_name)
        if default_element is not None:
            default_element.set("value", serialized_value)
            default_element.set("type", normalize_type(default_type))

    for parameter_name, raw_value in provided_parameters.items():
        parameter_element = parameter_map[parameter_name]
        default_type = parameter_element.attrib.get("default_type", "").strip() or "str"
        typed_value = parse_key_value(raw_value, default_type)
        serialized_value = format_default_value(typed_value, default_type)

        parameter_element.set("default_value", serialized_value)
        parameter_element.set("default_type", normalize_type(default_type))

    indent_xml(root)
    return ET.tostring(root, encoding="unicode")


def _absolutize_includes(root: ET.Element, base_dir: Path) -> None:
    """Resolve relative includes against the original file's directory."""
    for element in root.iter():
        if strip_namespace(element.tag) != "StateMachine":
            continue
        file_path = element.attrib.get("file_path")
        if file_path and not os.path.isabs(file_path):
            element.set("file_path", str(base_dir / file_path))


def ros_parameter_argument(name: str, value: str) -> str:
    """A -p argument whose value stays a string even with ': ' or ' #'."""
    # rcl parses parameter values as YAML; a JSON string is a YAML string.
    return f"{name}:={json.dumps(value)}"


def run_node(command: List[str]) -> int:
    """
    Run a node until it exits, forwarding termination signals to it.

    The node gets its own process group, so a terminal Ctrl-C reaches it only
    once, through this process; SIGINT/SIGTERM sent to this process alone (as
    launch and process managers do) reach it as well.
    """
    try:
        process = subprocess.Popen(command, start_new_session=True)
    except OSError as exc:
        print(f"Failed to execute '{command[0]}': {exc}", file=sys.stderr)
        return 1

    def forward(signum, _frame):
        if process.poll() is None:
            process.send_signal(signum)

    forwarded = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
    previous = {signum: signal.signal(signum, forward) for signum in forwarded}
    try:
        return_code = process.wait()
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
    # Like a shell: a node killed by signal N exits with 128 + N.
    return 128 - return_code if return_code < 0 else return_code


def run_package_executable(package: str, executable: str, arguments: List[str]) -> int:
    """Run a package executable directly (as `ros2 run` does) via run_node()."""
    try:
        path = get_executable_path(package_name=package, executable_name=executable)
    except PackageNotFound:
        path = None
    if path is None:
        print(f"Executable '{executable}' of {package} not found", file=sys.stderr)
        return 1
    return run_node([path, *arguments])


def run_factory_node(
    state_machine_file: str,
    disable_viewer_pub: bool = False,
    use_python: bool = False,
    ros_args: Optional[List[str]] = None,
) -> int:
    executable = "yasmin_factory_node.py" if use_python else "yasmin_factory_node"
    arguments = [
        "--ros-args",
        "-p",
        ros_parameter_argument("state_machine_file", str(state_machine_file)),
        "-p",
        f"enable_viewer_pub:={'false' if disable_viewer_pub else 'true'}",
    ]
    if ros_args:
        arguments += ["--ros-args", *ros_args]
    return run_package_executable("yasmin_factory", executable, arguments)


def add_run_verb(subparsers):
    parser = subparsers.add_parser(
        "run",
        help="Run a YASMIN state machine from an XML file",
        description="Run a YASMIN state machine from an XML file",
    )

    xml_arg = parser.add_argument(
        "state_machine_file",
        help="Path to the XML state machine file",
    )
    xml_arg.completer = xml_file_completer

    input_arg = parser.add_argument(
        "--input",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="Override an input key from the XML state machine, may be given multiple times",
    )
    input_arg.completer = run_input_completer

    param_arg = parser.add_argument(
        "--param",
        action="append",
        default=[],
        metavar="PARAM=VALUE",
        help="Override a declared parameter from the XML state machine, may be given multiple times",
    )
    param_arg.completer = run_param_completer

    parser.add_argument(
        "--disable-viewer-pub",
        action="store_true",
        help="Disable FSM viewer publisher",
    )
    parser.add_argument(
        "--py",
        action="store_true",
        help="Use the Python factory node instead of the C++ factory node",
    )
    parser.add_argument(
        "--ros-args",
        dest="ros_args",
        nargs=argparse.REMAINDER,
        default=[],
        help="ROS arguments for the factory node (namespace, remappings, "
        "parameters); everything after --ros-args is passed on",
    )

    parser.set_defaults(main=_main_run)


def _error(message: str) -> int:
    print(message, file=sys.stderr)
    return 1


def _main_run(args):
    xml_path = Path(args.state_machine_file)
    if not xml_path.is_file():
        return _error(f"File does not exist: {args.state_machine_file}")

    try:
        tree = ET.parse(xml_path)
    except ET.ParseError as exc:
        return _error(f"Failed to parse XML file '{args.state_machine_file}': {exc}")
    except OSError as exc:
        return _error(f"Failed to read XML file '{args.state_machine_file}': {exc}")

    root = tree.getroot()
    if strip_namespace(root.tag) != "StateMachine":
        return _error(
            f"Not a valid YASMIN state machine XML file: {args.state_machine_file}"
        )

    try:
        provided_inputs = parse_assignments(args.input, "input")
        provided_parameters = parse_assignments(args.param, "parameter")
    except ValueError as exc:
        return _error(str(exc))

    if not provided_inputs and not provided_parameters:
        return run_factory_node(
            state_machine_file=args.state_machine_file,
            disable_viewer_pub=args.disable_viewer_pub,
            use_python=args.py,
            ros_args=args.ros_args,
        )

    input_key_map = _find_input_key_map(root)
    valid_input_names = set(input_key_map.keys())
    unknown_inputs = sorted(
        name for name in provided_inputs if name not in valid_input_names
    )
    if unknown_inputs:
        return _error(
            f"Unknown input keys for state machine '{args.state_machine_file}': "
            f"{', '.join(unknown_inputs)}"
        )

    parameter_map = _find_parameter_map(root)
    valid_parameter_names = set(parameter_map.keys())
    unknown_parameters = sorted(
        name for name in provided_parameters if name not in valid_parameter_names
    )
    if unknown_parameters:
        return _error(
            f"Unknown parameters for state machine '{args.state_machine_file}': "
            f"{', '.join(unknown_parameters)}"
        )

    # The modified copy lives in a temporary directory, so relative includes
    # must point back to the original file's directory.
    _absolutize_includes(root, xml_path.resolve().parent)
    try:
        xml_content = _inject_overrides_into_root(
            root,
            provided_inputs,
            provided_parameters,
        )
    except ValueError as exc:
        return _error(str(exc))

    with tempfile.TemporaryDirectory(prefix="yasmin_run_") as temp_dir:
        temp_path = Path(temp_dir) / xml_path.name
        temp_path.write_text(xml_content, encoding="utf-8")

        return run_factory_node(
            state_machine_file=temp_path.as_posix(),
            disable_viewer_pub=args.disable_viewer_pub,
            use_python=args.py,
            ros_args=args.ros_args,
        )
