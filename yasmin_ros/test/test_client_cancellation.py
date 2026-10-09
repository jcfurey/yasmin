# Copyright (C) 2026
# Licensed under the Apache License, Version 2.0.

from concurrent.futures import Future
from queue import Queue
from threading import Thread
from time import sleep
from types import SimpleNamespace
from unittest.mock import Mock, patch

from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci
from example_interfaces.srv import AddTwoInts
from std_msgs.msg import String

from yasmin_ros import ActionState, ServiceState
from yasmin_ros.basic_outcomes import ABORT, CANCEL, SUCCEED, TIMEOUT
from yasmin_ros.ros_clients_cache import ROSClientsCache


class ActionClientStub:
    def __init__(self):
        self.sent = Queue()

    def wait_for_server(self, timeout):
        return True

    def send_goal_async(self, goal, feedback_callback):
        future = Future()
        self.sent.put(future)
        return future


def start(state):
    results = Queue()

    def run():
        try:
            results.put(state())
        except BaseException as error:
            results.put(error)

    worker = Thread(target=run, daemon=True)
    worker.start()
    return worker, results


def action_state(client, **kwargs):
    with patch.object(
        ROSClientsCache, "get_or_create_action_client", return_value=client
    ):
        return ActionState(
            Fibonacci,
            "action",
            lambda _: Fibonacci.Goal(),
            node=SimpleNamespace(context=SimpleNamespace(ok=lambda: True)),
            **kwargs
        )


def goal_handle():
    result = Future()
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = result
    handle.cancel_goal_async.return_value = Future()  # No cancel acknowledgement.
    return handle, result


def test_cancel_wakes_action_without_goal_response():
    client = ActionClientStub()
    state = action_state(client)
    worker, results = start(state)
    pending = client.sent.get(timeout=1)
    state.cancel_state()
    worker.join(timeout=1)
    assert not worker.is_alive()
    assert results.get(timeout=1) == CANCEL
    handle, _ = goal_handle()
    pending.set_result(handle)
    handle.cancel_goal_async.assert_called_once()


def test_cancel_wakes_action_without_cancel_acknowledgement():
    client = ActionClientStub()
    state = action_state(client)
    worker, results = start(state)
    pending = client.sent.get(timeout=1)
    handle, _ = goal_handle()
    pending.set_result(handle)
    state.cancel_state()
    worker.join(timeout=1)
    assert not worker.is_alive()
    assert results.get(timeout=1) == CANCEL
    handle.cancel_goal_async.assert_called_once()


def test_old_result_cannot_complete_reused_action():
    client = ActionClientStub()
    state = action_state(client, response_timeout=0.2, maximum_retry=0)
    first_worker, first_results = start(state)
    first_goal = client.sent.get(timeout=1)
    first_handle, first_result = goal_handle()
    first_goal.set_result(first_handle)
    first_worker.join(timeout=1)
    assert first_results.get(timeout=1) == TIMEOUT
    first_handle.cancel_goal_async.assert_called_once()

    worker, results = start(state)
    pending = client.sent.get(timeout=1)
    try:
        first_result.set_result(
            SimpleNamespace(result=Fibonacci.Result(), status=GoalStatus.STATUS_SUCCEEDED)
        )
        assert results.empty()
        handle, result = goal_handle()
        pending.set_result(handle)
        result.set_result(
            SimpleNamespace(result=Fibonacci.Result(), status=GoalStatus.STATUS_ABORTED)
        )
        worker.join(timeout=1)
        assert results.get(timeout=1) == ABORT
    finally:
        state.cancel_state()
        worker.join(timeout=1)


def test_old_goal_response_is_canceled_after_reuse():
    client = ActionClientStub()
    state = action_state(client, response_timeout=0.02, maximum_retry=0)
    assert state() == TIMEOUT
    first_goal = client.sent.get(timeout=1)
    worker, results = start(state)
    pending = client.sent.get(timeout=1)
    old_handle, _ = goal_handle()
    first_goal.set_result(old_handle)
    old_handle.cancel_goal_async.assert_called_once()
    state.cancel_state()
    worker.join(timeout=1)
    assert results.get(timeout=1) == CANCEL
    handle, _ = goal_handle()
    pending.set_result(handle)
    handle.cancel_goal_async.assert_called_once()


def test_old_response_cannot_complete_reused_service():
    sent = Queue()
    client = Mock()
    client.wait_for_service.return_value = True

    def call_async(request):
        future = Future()
        sent.put(future)
        return future

    client.call_async.side_effect = call_async
    with patch.object(
        ROSClientsCache, "get_or_create_service_client", return_value=client
    ):
        state = ServiceState(
            AddTwoInts,
            "service",
            lambda _: AddTwoInts.Request(),
            outcomes={"old", "new"},
            response_handler=lambda _, response: "old" if response.sum == 1 else "new",
            node=SimpleNamespace(context=SimpleNamespace(ok=lambda: True)),
            response_timeout=0.2,
            maximum_retry=0,
        )

    assert state() == TIMEOUT
    old_request = sent.get(timeout=1)
    client.remove_pending_request.assert_called_once_with(old_request)

    worker, results = start(state)
    pending = sent.get(timeout=1)
    try:
        old_request.set_result(AddTwoInts.Response(sum=1))
        sleep(0.05)
        assert results.empty()
        pending.set_result(AddTwoInts.Response(sum=2))
        worker.join(timeout=1)
        assert results.get(timeout=1) == "new"
    finally:
        state.cancel_state()
        worker.join(timeout=1)


def test_goal_response_timeout_cancels_late_goal():
    client = ActionClientStub()
    state = action_state(client, goal_response_timeout=0.1)
    worker, results = start(state)
    pending = client.sent.get(timeout=1)
    worker.join(timeout=1)
    assert not worker.is_alive()
    assert results.get(timeout=1) == TIMEOUT
    handle, _ = goal_handle()
    pending.set_result(handle)
    handle.cancel_goal_async.assert_called_once()


def test_goal_response_timeout_does_not_limit_accepted_goal():
    client = ActionClientStub()
    state = action_state(client, goal_response_timeout=0.1)
    worker, results = start(state)
    pending = client.sent.get(timeout=1)
    try:
        handle, result = goal_handle()
        pending.set_result(handle)
        sleep(0.3)
        assert results.empty()
        result.set_result(
            SimpleNamespace(result=Fibonacci.Result(), status=GoalStatus.STATUS_SUCCEEDED)
        )
        worker.join(timeout=1)
        assert results.get(timeout=1) == SUCCEED
        handle.cancel_goal_async.assert_not_called()
    finally:
        state.cancel_state()
        worker.join(timeout=1)


def test_action_and_service_discovery_observe_cancellation():
    def unavailable(timeout):
        sleep(timeout)
        return False

    client = ActionClientStub()
    client.wait_for_server = unavailable
    action = action_state(client)
    service_client = Mock()
    service_client.wait_for_service.side_effect = lambda timeout_sec: unavailable(
        timeout_sec
    )
    with patch.object(
        ROSClientsCache, "get_or_create_service_client", return_value=service_client
    ):
        service = ServiceState(
            AddTwoInts,
            "service",
            lambda _: AddTwoInts.Request(),
            node=SimpleNamespace(context=SimpleNamespace(ok=lambda: True)),
        )
    for state in (action, service):
        worker, results = start(state)
        sleep(0.02)
        state.cancel_state()
        worker.join(timeout=1)
        assert not worker.is_alive()
        assert results.get(timeout=1) == CANCEL


def test_cache_separates_node_instances_with_identical_names():
    class NodeStub:
        def get_name(self):
            return "same_name"

        def create_client(self, *args, **kwargs):
            return object()

        def create_publisher(self, *args, **kwargs):
            return object()

    ROSClientsCache.clear_all()
    first, second = NodeStub(), NodeStub()
    try:
        with patch(
            "yasmin_ros.ros_clients_cache.ActionClient",
            side_effect=lambda *a, **kw: object(),
        ):
            for factory, message_type in (
                (ROSClientsCache.get_or_create_action_client, Fibonacci),
                (ROSClientsCache.get_or_create_service_client, AddTwoInts),
                (ROSClientsCache.get_or_create_publisher, String),
            ):
                endpoint = factory(first, message_type, "relative")
                assert endpoint is factory(first, message_type, "relative")
                assert endpoint is not factory(second, message_type, "relative")
        ROSClientsCache.clear_for_node(first)
        assert ROSClientsCache.get_cache_stats()["total"] == 3
    finally:
        ROSClientsCache.clear_all()
