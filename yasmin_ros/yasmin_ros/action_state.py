# Copyright (C) 2023 Miguel Ángel González Santamarta
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

from dataclasses import dataclass, field
from threading import RLock, Event
from typing import Set, Callable, Type, Any

from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import CallbackGroup
from action_msgs.msg import GoalStatus

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL, TIMEOUT
from yasmin_ros.ros_clients_cache import ROSClientsCache
from yasmin_ros.ros_state_utils import (
    resolve_node, wait_with_retry, wait_for_server_with_retry, setup_outcomes,
)


@dataclass
class _ActionExecution:
    done: Event = field(default_factory=Event)
    result: Any = None
    status: int = GoalStatus.STATUS_UNKNOWN
    goal_handle: Any = None
    cancel_requested: bool = False


class ActionState(State):
    """
    A state class for handling ROS 2 action client operations.

    This class encapsulates the behavior of a ROS 2 action client within a YASMIN
    state. It allows the creation and management of goals, feedback, and results
    associated with an action server.
    """

    def __init__(
        self,
        action_type: Type,
        action_name: str,
        create_goal_handler: Callable,
        outcomes: Set[str] = set(),
        result_handler: Callable = None,
        feedback_handler: Callable = None,
        callback_group: CallbackGroup = None,
        node: Node = None,
        wait_timeout: float = None,
        response_timeout: float = None,
        maximum_retry: int = 3,
        abort_handler: Callable = None,
    ) -> None:
        """
        Construct an ActionState with a specific action name and goal handler.

        This constructor initializes the action state with a specified action name,
        goal handler, and optional timeout.

        Args:
            action_type (Type): The type of the action to be executed.
            action_name (str): The name of the action to communicate with.
            create_goal_handler (Callable[[Blackboard], Any]): A function that creates a goal for the action.
            outcomes (Set[str], optional): A set of possible outcomes for this action state.
            result_handler (Callable[[Blackboard, Any], str], optional): A function to handle the result of the action.
            feedback_handler (Callable[[Blackboard, Any], None], optional): A function to handle feedback from the action.
            callback_group (CallbackGroup, optional): The callback group for the action client.
            node (Node, optional): The ROS 2 node to use. If None, uses the default YasminNode.
            wait_timeout (float, optional): The maximum time to wait for the action server. Default is None (wait indefinitely).
            response_timeout (float, optional): The maximum time to wait for the action response. Default is None (wait indefinitely).
            maximum_retry (int, optional): Maximum retries of the action if it returns timeout. Default is 3.
            abort_handler (Callable[[Blackboard, Any], str], optional): A function mapping the result
                of a goal aborted by the server to an outcome. Servers such as Nav2 report the failure
                reason in this result (e.g. ``error_code``). Without it, or for a rejected goal, the
                state returns ABORT.

        Raises:
            ValueError: If create_goal_handler is None.
        """

        self._goal_handle_lock = RLock()
        self._execution = None

        ## Handler function for creating goals.
        self._create_goal_handler: Callable[[Blackboard], Any] = create_goal_handler
        ## Handler function for processing results.
        self._result_handler: Callable[[Blackboard, Any], str] = result_handler
        ## Handler function for processing feedback.
        self._feedback_handler: Callable[[Blackboard, Any], None] = feedback_handler
        ## Handler function for processing aborted results.
        self._abort_handler: Callable[[Blackboard, Any], str] = abort_handler

        ## Maximum time to wait for the action server.
        self._wait_timeout: float = wait_timeout
        ## Timeout for the action response.
        self._response_timeout: float = response_timeout

        ## Maximum number of retries.
        self._maximum_retry: int = maximum_retry

        # Set outcomes
        outcomes = setup_outcomes(
            outcomes,
            {SUCCEED, ABORT, CANCEL},
            add_timeout=(
                self._wait_timeout is not None or self._response_timeout is not None
            ),
        )

        self._node: Node = resolve_node(node)

        ## Name of the action to communicate with.
        self._action_name: str = action_name

        ## Action type for caching
        self._action_type: Type = action_type

        ## Shared pointer to the action client (reused from cache if available).
        self._action_client: ActionClient = ROSClientsCache.get_or_create_action_client(
            self._node,
            action_type,
            action_name,
            callback_group,
        )

        if not self._create_goal_handler:
            raise ValueError("create_goal_handler is needed")

        super().__init__(outcomes)

    def _cancel_goal(self, execution=None) -> None:
        with self._goal_handle_lock:
            execution = execution or self._execution
            if execution is None:
                return
            execution.cancel_requested = True
            goal_handle = execution.goal_handle
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as error:
                yasmin.YASMIN_LOG_WARN(f"Failed to cancel action '{self._action_name}': {error}")

    def cancel_state(self) -> None:
        """Wake the local wait immediately and request remote cancellation."""
        with self._goal_handle_lock:
            super().cancel_state()
            execution = self._execution
            if execution is not None:
                execution.cancel_requested = True
                execution.done.set()
        self._cancel_goal(execution)

    def execute(self, blackboard: Blackboard) -> str:
        """
        Execute the action and return the outcome.

        This function creates a goal using the provided goal handler, sends the
        goal to the action server, and waits for the result or feedback.

        Args:
            blackboard (Blackboard): A shared pointer to the blackboard used for communication.

        Returns:
            str: A string representing the outcome of the action execution.
                Possible outcomes include SUCCEED, ABORT, CANCEL, or TIMEOUT.
        """
        goal = self._create_goal_handler(blackboard)

        yasmin.YASMIN_LOG_INFO(f"Waiting for action '{self._action_name}'")

        outcome = wait_for_server_with_retry(
            self._action_client.wait_for_server,
            self._wait_timeout,
            self._maximum_retry,
            f"Timeout reached, action '{self._action_name}' is not available",
            cancel_check=lambda: self.is_canceled() or not self._node.context.ok(),
        )
        if outcome is not None:
            return outcome

        with self._goal_handle_lock:
            if self.is_canceled():
                return CANCEL
            execution = _ActionExecution()
            self._execution = execution

        yasmin.YASMIN_LOG_INFO(f"Sending goal to action '{self._action_name}'")

        def feedback_handler(feedback):
            if (self._execution is execution and not execution.cancel_requested
                    and self._feedback_handler is not None):
                self._feedback_handler(blackboard, feedback.feedback)

        send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=feedback_handler
        )
        send_goal_future.add_done_callback(
            lambda future: self._goal_response_callback(future, execution)
        )

        outcome = wait_with_retry(
            lambda: execution.done.wait(self._response_timeout),
            self._maximum_retry,
            f"Timeout reached while waiting for response from "
            f"action '{self._action_name}'",
            cancel_check=self.is_canceled,
        )
        if outcome is not None:
            if outcome == TIMEOUT:
                self._cancel_goal(execution)
            return outcome

        if self.is_canceled():
            return CANCEL

        status = execution.status

        if status == GoalStatus.STATUS_CANCELED:
            return CANCEL

        elif status == GoalStatus.STATUS_ABORTED:
            if self._abort_handler is not None and execution.result is not None:
                return self._abort_handler(blackboard, execution.result)
            return ABORT

        elif status == GoalStatus.STATUS_SUCCEEDED:
            if self._result_handler is not None:
                return self._result_handler(blackboard, execution.result)

            return SUCCEED

        return ABORT

    def _goal_response_callback(self, future: Future, execution: _ActionExecution) -> None:
        try:
            goal_handle = future.result()
            with self._goal_handle_lock:
                execution.goal_handle = goal_handle
                stale = self._execution is not execution
                cancel = execution.cancel_requested or stale or self.is_canceled()
            if goal_handle is None or not goal_handle.accepted:
                execution.status = GoalStatus.STATUS_ABORTED
                execution.done.set()
                return
            if cancel:
                self._cancel_goal(execution)
            if stale:
                return
            goal_handle.get_result_async().add_done_callback(
                lambda result: self._get_result_callback(result, execution)
            )
        except Exception as error:
            yasmin.YASMIN_LOG_ERROR(
                f"Failed to handle goal response for action '{self._action_name}': {error}"
            )
            execution.status = GoalStatus.STATUS_UNKNOWN
            execution.done.set()

    def _get_result_callback(self, future: Future, execution: _ActionExecution) -> None:
        # Each invocation owns its event and result. Late callbacks can only
        # finish their own invocation, including after a timeout and reuse.
        try:
            result = future.result()
            execution.result = result.result
            execution.status = result.status
        except Exception as error:
            yasmin.YASMIN_LOG_ERROR(
                f"Failed to handle result for action '{self._action_name}': {error}"
            )
            execution.result = None
            execution.status = GoalStatus.STATUS_UNKNOWN
        finally:
            execution.done.set()
