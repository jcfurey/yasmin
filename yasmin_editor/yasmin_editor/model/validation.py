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

from yasmin_editor.dataclass_compat import dataclass, field
from collections import deque
from typing import Dict, Iterable, List, Optional, Set, Union

from .container_state import ContainerState, iter_outcome_rule_values
from .join_state import JoinState
from .orthogonal_state import OrthogonalState
from .parameter import Parameter
from .state import State
from .state_machine import StateMachine
from .value_types import default_value_error


@dataclass(slots=True)
class ValidationMessage:
    """Represents one validation message."""

    path: str
    message: str


@dataclass(slots=True)
class ValidationResult:
    """Collects validation errors and warnings."""

    errors: List[ValidationMessage] = field(default_factory=list)
    warnings: List[ValidationMessage] = field(default_factory=list)

    @property
    def is_valid(self) -> bool:
        """Return whether the validated model contains no errors."""

        return not self.errors

    def add_error(self, path: str, message: str) -> None:
        """Add one validation error."""

        self.errors.append(ValidationMessage(path=path, message=message))

    def add_warning(self, path: str, message: str) -> None:
        """Add one validation warning."""

        self.warnings.append(ValidationMessage(path=path, message=message))

    def extend(self, other: "ValidationResult") -> None:
        """Merge another validation result into this one."""

        self.errors.extend(other.errors)
        self.warnings.extend(other.warnings)

    def __str__(self) -> str:
        """Return a readable validation summary."""

        lines: List[str] = []

        if self.errors:
            lines.append("Errors:")
            for error in self.errors:
                lines.append(f"  - {error.path}: {error.message}")

        if self.warnings:
            if lines:
                lines.append("")
            lines.append("Warnings:")
            for warning in self.warnings:
                lines.append(f"  - {warning.path}: {warning.message}")

        if not lines:
            lines.append("Validation successful")

        return "\n".join(lines)

    __repr__ = __str__


def validate_model(model: State) -> ValidationResult:
    """Validate a model tree against the rules enforced by the YASMIN factories."""

    result = ValidationResult()
    _validate_state(model, result, model.name, parent_targets=None, is_root=True)
    return result


def _validate_state(
    state: State,
    result: ValidationResult,
    path: str,
    parent_targets: Union[Set[str], None],
    *,
    is_root: bool = False,
) -> None:
    """Validate one state recursively."""

    _validate_common_state_fields(state, result, path, is_root=is_root)

    if isinstance(state, StateMachine):
        _validate_state_machine(state, result, path, parent_targets)
    elif isinstance(state, ContainerState):
        _validate_container_state(state, result, path, parent_targets)
    elif isinstance(state, JoinState):
        _validate_join_state(state, result, path)
    else:
        _validate_leaf_state(state, result, path)


def _validate_unique_named_items(
    items: Iterable[object],
    *,
    path: str,
    result: ValidationResult,
    field_name: str,
) -> None:
    """Validate that a sequence of named model items is non-empty and unique."""

    seen_names: Set[str] = set()

    for item in items:
        name = getattr(item, "name", "")
        if not name:
            result.add_error(path, f"{field_name} name must not be empty")
            continue

        if name in seen_names:
            result.add_error(path, f"Duplicate {field_name.lower()} '{name}'")
        seen_names.add(name)


def _validate_common_state_fields(
    state: State,
    result: ValidationResult,
    path: str,
    *,
    is_root: bool = False,
) -> None:
    """Validate fields common to all states."""

    # The factories accept an unnamed root state machine.
    if not state.name and not is_root:
        result.add_error(path, "State name must not be empty")

    _validate_unique_named_items(
        state.outcomes,
        path=path,
        result=result,
        field_name="Outcome",
    )
    _validate_unique_named_items(
        state.keys,
        path=path,
        result=result,
        field_name="Key",
    )

    for key in state.keys:
        if key.key_type not in ("in", "out", "in/out"):
            result.add_error(
                path,
                f"Key '{key.name}' has unknown type '{key.key_type}' "
                "(expected 'in', 'out' or 'in/out')",
            )
    _validate_default_values(
        [key for key in state.keys if key.key_type in ("in", "in/out")],
        kind="Key",
        path=path,
        result=result,
    )
    _validate_default_values(state.parameters, kind="Parameter", path=path, result=result)


def _validate_default_values(
    items: Iterable[Parameter],
    *,
    kind: str,
    path: str,
    result: ValidationResult,
) -> None:
    """Report defaults that the factory cannot parse for their declared type."""

    for item in items:
        if not item.has_default:
            continue
        try:
            error = default_value_error(item.default_value, item.default_type)
        except ImportError:
            return
        if error:
            result.add_error(
                path,
                f"{kind} '{item.name}' has an invalid default value "
                f"'{item.default_value}' for type '{item.default_type}': {error}",
            )


def _validate_outcome_names_without_whitespace(
    container: State,
    result: ValidationResult,
    path: str,
) -> None:
    """Container outcomes are stored space-separated, so names cannot contain spaces."""

    for outcome in container.outcomes:
        if any(character.isspace() for character in outcome.name):
            result.add_error(
                path,
                f"Outcome name '{outcome.name}' must not contain whitespace",
            )


def _validate_leaf_state(state: State, result: ValidationResult, path: str) -> None:
    """Validate a leaf state."""

    if state.state_type == "py":
        if not state.module:
            result.add_error(path, "Python state requires 'module'")
        if not state.class_name:
            result.add_error(path, "Python state requires 'class_name'")

    elif state.state_type == "cpp":
        if not state.class_name:
            result.add_error(path, "C++ state requires 'class_name'")

    elif state.state_type == "xml":
        if not state.file_path and not state.file_name:
            result.add_error(path, "XML state requires 'file_name'")
        elif not state.file_path and not state.package_name:
            result.add_error(
                path,
                "XML state with 'file_name' requires 'package_name' "
                "(or use 'file_path')",
            )

    elif state.state_type is None:
        result.add_warning(path, "Leaf state has no 'state_type'")

    else:
        result.add_error(path, f"Unknown state type '{state.state_type}'")


def _validate_join_state(state: JoinState, result: ValidationResult, path: str) -> None:
    """Validate a join state."""

    if not state.join_outcome:
        result.add_error(path, "Join state requires an outcome")


def _validate_conflicting_container_names(
    *,
    path: str,
    result: ValidationResult,
    state_names: Set[str],
    outcome_names: Set[str],
) -> None:
    """Validate that container child states and final outcomes do not collide."""

    conflicting_names = sorted(state_names & outcome_names)
    for name in conflicting_names:
        result.add_error(
            path,
            f"Name '{name}' is used by both a child state and a final outcome",
        )


def _validate_child_state_binding(
    *,
    container_path: str,
    result: ValidationResult,
    state_name: str,
    child_state: State,
) -> str:
    """Validate one child-state dictionary binding and return its child path."""

    child_path = f"{container_path}/{state_name}"

    if not child_state.name:
        result.add_error(child_path, "Child state name must not be empty")
    elif child_state.name != state_name:
        result.add_error(
            child_path,
            f"Dictionary key '{state_name}' does not match state name '{child_state.name}'",
        )

    return child_path


def _known_child_outcomes(child_state: State) -> Set[str]:
    """Return the outcomes a child is known to produce (empty when unknown)."""

    outcomes = {outcome.name for outcome in child_state.outcomes}
    if isinstance(child_state, ContainerState) and child_state.default_outcome:
        outcomes.add(child_state.default_outcome)
    if isinstance(child_state, JoinState) and child_state.join_outcome:
        outcomes.add(child_state.join_outcome)
    return outcomes


def _declared_child_parameters(child_state: State) -> Optional[Set[str]]:
    """Return the parameters a child declares, or ``None`` when unknown.

    Leaf states declare parameters in their implementation, which the model
    does not know, so they are not checked.
    """

    if isinstance(child_state, (StateMachine, ContainerState)):
        return {parameter.name for parameter in child_state.parameters}
    if isinstance(child_state, JoinState):
        return set()
    return None


def _validate_parameter_mappings(
    container: State,
    child_state: State,
    child_path: str,
    result: ValidationResult,
) -> None:
    """Validate ``ParamRemap`` entries of one child against the declarations."""

    declared = {parameter.name for parameter in container.parameters}
    child_declared = _declared_child_parameters(child_state)
    owner_label = container.name or "root"

    for child_parameter, parent_parameter in child_state.parameter_mappings.items():
        if not child_parameter or not parent_parameter:
            continue
        if parent_parameter not in declared:
            result.add_error(
                child_path,
                f"Parameter remap target '{parent_parameter}' is not declared "
                f"by '{owner_label}'",
            )
        if child_declared is not None and child_parameter not in child_declared:
            result.add_error(
                child_path,
                f"Parameter remap source '{child_parameter}' is not declared "
                f"by '{child_state.name}'",
            )


def _validate_reachability(
    state_machine: StateMachine,
    result: ValidationResult,
    path: str,
) -> None:
    """Report child states that YASMIN rejects as unreachable from the start."""

    if not state_machine.states:
        return

    start_state = state_machine.start_state or next(iter(state_machine.states))
    if start_state not in state_machine.states:
        return

    reachable: Set[str] = {start_state}
    pending = deque([start_state])
    while pending:
        current = pending.popleft()
        for transition in state_machine.transitions.get(current, []):
            target = transition.target
            if target in state_machine.states and target not in reachable:
                reachable.add(target)
                pending.append(target)

    for state_name in state_machine.states:
        if state_name not in reachable:
            result.add_error(
                path,
                f"State '{state_name}' is unreachable from start state '{start_state}'",
            )


def _validate_state_machine(
    state_machine: StateMachine,
    result: ValidationResult,
    path: str,
    parent_targets: Union[Set[str], None],
) -> None:
    """Validate a state machine recursively."""

    if not state_machine.states:
        result.add_warning(path, "State machine has no child states")

    state_names = set(state_machine.states)
    outcome_names = {outcome.name for outcome in state_machine.outcomes}
    _validate_conflicting_container_names(
        path=path,
        result=result,
        state_names=state_names,
        outcome_names=outcome_names,
    )
    _validate_outcome_names_without_whitespace(state_machine, result, path)
    local_targets = state_names | outcome_names
    nested_parent_targets = local_targets | (parent_targets or set())

    if not outcome_names:
        result.add_error(path, "State machine requires at least one outcome")

    # Without 'start_state' YASMIN starts with the first added child state.
    if state_machine.start_state and state_machine.start_state not in state_names:
        result.add_error(
            path,
            f"Start state '{state_machine.start_state}' does not exist",
        )

    for state_name, child_state in state_machine.states.items():
        child_path = _validate_child_state_binding(
            container_path=path,
            result=result,
            state_name=state_name,
            child_state=child_state,
        )

        _validate_state(child_state, result, child_path, nested_parent_targets)

        known_outcomes = _known_child_outcomes(child_state)
        targets_by_outcome: Dict[str, str] = {}
        transitions = state_machine.transitions.get(state_name, [])
        for transition in transitions:
            if transition.target not in local_targets:
                result.add_error(
                    child_path,
                    f"Transition target '{transition.target}' does not exist",
                )
            if known_outcomes and transition.source_outcome not in known_outcomes:
                result.add_error(
                    child_path,
                    f"Transition uses unknown outcome '{transition.source_outcome}'",
                )
            previous_target = targets_by_outcome.setdefault(
                transition.source_outcome, transition.target
            )
            if previous_target != transition.target:
                result.add_error(
                    child_path,
                    f"Outcome '{transition.source_outcome}' has more than one transition",
                )

        for source_key, target_key in child_state.remappings.items():
            if not source_key:
                result.add_error(child_path, "Remapping source key must not be empty")
            if not target_key:
                result.add_error(child_path, "Remapping target key must not be empty")

        _validate_parameter_mappings(state_machine, child_state, child_path, result)

    for owner_name, transitions in state_machine.transitions.items():
        if owner_name == state_machine.name:
            for transition in transitions:
                if transition.source_outcome not in outcome_names:
                    result.add_warning(
                        path,
                        f"Container transition uses unknown outcome '{transition.source_outcome}'",
                    )
                if transition.target not in nested_parent_targets:
                    result.add_error(
                        path,
                        f"Container transition target '{transition.target}' does not exist",
                    )
            continue

        if owner_name in state_names:
            continue

        if owner_name in outcome_names:
            for transition in transitions:
                if transition.target not in nested_parent_targets:
                    result.add_error(
                        path,
                        f"Final outcome transition target '{transition.target}' does not exist",
                    )
            continue

        result.add_error(
            path,
            f"Transitions defined for unknown owner '{owner_name}'",
        )

    _validate_reachability(state_machine, result, path)


def _validate_container_state(
    container: ContainerState,
    result: ValidationResult,
    path: str,
    parent_targets: Union[Set[str], None],
) -> None:
    """Validate a container state (Concurrence or OrthogonalState) recursively."""

    is_orthogonal = isinstance(container, OrthogonalState)
    kind = "Orthogonal state" if is_orthogonal else "Concurrence"
    child_term = "region" if is_orthogonal else "child state"
    if not container.states:
        result.add_warning(path, f"{kind} has no child states")

    state_names = set(container.states)
    outcome_names = {outcome.name for outcome in container.outcomes}
    _validate_conflicting_container_names(
        path=path,
        result=result,
        state_names=state_names,
        outcome_names=outcome_names,
    )
    _validate_outcome_names_without_whitespace(container, result, path)
    nested_parent_targets = state_names | outcome_names | (parent_targets or set())

    if not outcome_names:
        result.add_error(path, f"{kind} requires at least one outcome")

    if container.default_outcome and container.default_outcome not in outcome_names:
        result.add_error(
            path,
            f"Default outcome '{container.default_outcome}' does not exist",
        )

    for state_name, child_state in container.states.items():
        child_path = _validate_child_state_binding(
            container_path=path,
            result=result,
            state_name=state_name,
            child_state=child_state,
        )

        _validate_state(child_state, result, child_path, nested_parent_targets)

        # YASMIN containers that run children in parallel take no blackboard
        # remappings (and regions no parameter remappings); the factories drop them.
        if any(source or target for source, target in child_state.remappings.items()):
            result.add_error(
                child_path,
                f"Blackboard remappings on a {child_term} of a {kind.lower()} "
                "are ignored by YASMIN",
            )
        if is_orthogonal:
            if child_state.parameter_mappings:
                result.add_error(
                    child_path,
                    "Parameter remappings on a region are ignored by YASMIN",
                )
        else:
            _validate_parameter_mappings(container, child_state, child_path, result)

    for outcome_name, mapping in container.outcome_map.items():
        if outcome_name not in outcome_names:
            result.add_error(
                path,
                f"Outcome map references unknown outcome '{outcome_name}'",
            )

        for state_name, state_outcomes in mapping.items():
            if state_name not in state_names:
                result.add_error(
                    path,
                    f"Outcome map references unknown state '{state_name}'",
                )
                continue

            rule_outcomes = iter_outcome_rule_values(state_outcomes)
            if len(rule_outcomes) > 1:
                result.add_error(
                    f"{path}/{state_name}",
                    f"Outcome map rule '{outcome_name}' lists several outcomes "
                    f"({', '.join(rule_outcomes)}); YASMIN keeps only one per state",
                )

            child_state = container.states[state_name]
            child_outcomes = _known_child_outcomes(child_state)
            for state_outcome in rule_outcomes:
                if child_outcomes and state_outcome not in child_outcomes:
                    result.add_warning(
                        f"{path}/{state_name}",
                        f"Outcome map references unknown outcome '{state_outcome}'",
                    )
