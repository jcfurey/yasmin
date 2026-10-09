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
from threading import Event, Lock
from typing import Set, Callable, Type, Any

from rclpy.node import Node
from rclpy.task import Future
from rclpy.client import Client
from rclpy.callback_groups import CallbackGroup

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_ros.ros_clients_cache import ROSClientsCache
from yasmin_ros.ros_state_utils import (
    resolve_node,
    wait_with_retry,
    wait_for_server_with_retry,
    setup_outcomes,
)


@dataclass
class _ServiceCall:
    done: Event = field(default_factory=Event)
    response: Any = None


class ServiceState(State):
    """
    A state class that interacts with a ROS 2 service.

    This class manages communication with a specified ROS 2 service,
    allowing it to send requests and handle responses. It extends
    the base State class.
    """

    def __init__(
        self,
        srv_type: Type,
        srv_name: str,
        create_request_handler: Callable,
        outcomes: Set[str] = set(),
        response_handler: Callable = None,
        callback_group: CallbackGroup = None,
        node: Node = None,
        wait_timeout: float = None,
        response_timeout: float = None,
        maximum_retry: int = 3,
    ) -> None:
        """
        Initializes the ServiceState with the provided parameters.

        Args:
            srv_type (Type): The type of the service.
            srv_name (str): The name of the service to call.
            create_request_handler (Callable[[Blackboard], Any]): Function to create a service request.
            outcomes (Set[str], optional): A set of possible outcomes for this state.
            response_handler (Callable[[Blackboard, Any], str], optional): Function to handle the service response.
            callback_group (CallbackGroup, optional): The callback group for the service client.
            node (Node, optional): A ROS 2 node instance; if None, a default instance is used.
            wait_timeout (float, optional): Maximum time to wait for the service to become available. Default is None (wait indefinitely).
            response_timeout (float, optional): Maximum time to wait for the service response. Default is None (wait indefinitely).
            maximum_retry (int, optional): Maximum retries of the service if it returns timeout. Default is 3.

        Raises:
            ValueError: If the create_request_handler is not provided.
        """

        ## Function to create service requests.
        self._create_request_handler: Callable[[Blackboard], Any] = create_request_handler
        ## Function to handle service responses.
        self._response_handler: Callable[[Blackboard, Any], str] = response_handler

        ## Maximum wait time for service availability.
        self._wait_timeout: float = wait_timeout
        ## Timeout for the service response.
        self._response_timeout: float = response_timeout

        # Set outcomes
        outcomes = setup_outcomes(
            outcomes,
            {SUCCEED, ABORT, CANCEL},
            add_timeout=(
                self._wait_timeout is not None or self._response_timeout is not None
            ),
        )

        self._node = resolve_node(node)

        ## Name of the service.
        self._srv_name: str = srv_name

        ## Shared pointer to the service client.
        self._service_client: Client = ROSClientsCache.get_or_create_service_client(
            self._node,
            srv_type,
            srv_name,
            callback_group=callback_group,
        )

        ## The response received from the service.
        self._response: Any = None

        if not self._create_request_handler:
            raise ValueError("create_request_handler is needed")

        ## Maximum number of retries.
        self._maximum_retry: int = maximum_retry

        ## Current invocation; each one owns its event and response.
        self._call: _ServiceCall = None
        self._call_lock: Lock = Lock()

        super().__init__(outcomes)

    def execute(self, blackboard: Blackboard) -> str:
        """
        Execute the service call and handle the response.

        This method creates a request based on the blackboard data, waits for the
        service to become available, sends the request asynchronously, and waits for
        the response using a threading Event.

        Args:
            blackboard (Blackboard): A shared pointer to the blackboard containing data for
                request creation.

        Returns:
            str: The outcome of the service call, which can be SUCCEED, ABORT, or TIMEOUT.
        """
        request = self._create_request_handler(blackboard)

        yasmin.YASMIN_LOG_INFO(f"Waiting for service '{self._srv_name}'")

        outcome = wait_for_server_with_retry(
            lambda timeout: self._service_client.wait_for_service(timeout_sec=timeout),
            self._wait_timeout,
            self._maximum_retry,
            f"Timeout reached, service '{self._srv_name}' is not available",
            cancel_check=lambda: self.is_canceled() or not self._node.context.ok(),
        )
        if outcome is not None:
            return outcome

        # Checked under the lock so a concurrent cancel either stops the
        # request here or wakes this invocation's wait.
        with self._call_lock:
            if self.is_canceled():
                return CANCEL
            call = _ServiceCall()
            self._call = call

        try:
            yasmin.YASMIN_LOG_INFO(f"Sending request to service '{self._srv_name}'")

            future = self._service_client.call_async(request)
            future.add_done_callback(
                lambda done_future: self._response_callback(done_future, call)
            )

            outcome = wait_with_retry(
                lambda: call.done.wait(self._response_timeout),
                self._maximum_retry,
                f"Timeout reached while waiting for response from "
                f"service '{self._srv_name}'",
                cancel_check=self.is_canceled,
            )
            if outcome is not None:
                self._service_client.remove_pending_request(future)
                return outcome

            if self.is_canceled():
                self._service_client.remove_pending_request(future)
                return CANCEL

        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"Service call failed: {e}")
            return ABORT

        if call.response is None:
            return ABORT

        if self._response_handler:
            outcome = self._response_handler(blackboard, call.response)
            return outcome

        return SUCCEED

    def cancel_state(self) -> None:
        with self._call_lock:
            super().cancel_state()
            call = self._call
        if call is not None:
            call.done.set()

    def _response_callback(self, future: Future, call: _ServiceCall) -> None:
        """
        Store the service response for the invocation that sent the request.

        Late responses to an earlier invocation cannot complete a later one.

        Args:
            future (Future): The future object containing the service response.
            call (_ServiceCall): The invocation that sent the request.
        """
        try:
            call.response = future.result()
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"Service call failed: {e}")
            call.response = None
        call.done.set()
