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

import atexit
import uuid
from threading import Thread, RLock, current_thread
from typing import Optional

import yasmin_ros

import rclpy
from rclpy.node import Node

# Check if EventsExecutor is available
try:
    from rclpy.executors import EventsExecutor as Executor
except ImportError:
    from rclpy.executors import MultiThreadedExecutor as Executor


class YasminNode(Node):
    """
    A ROS 2 node for managing and handling YASMIN-based applications.

    YasminNode is a singleton class derived from Node and integrates
    custom functionalities for executing specific tasks in a ROS 2 environment.
    """

    ## The single instance of YasminNode.
    _instance: "YasminNode" = None
    ## Lock to control access to the instance.
    _lock: RLock = RLock()

    @staticmethod
    def get_instance(node_name: Optional[str] = None) -> "YasminNode":
        """
        Provides access to the singleton instance of YasminNode.

        This method ensures there is only one instance of YasminNode running.
        An instance whose context has been shut down is replaced, so states
        created after re-initializing rclpy do not use a dead node.

        Args:
            node_name (str, optional): Name used if this call creates the
                node. Defaults to a unique random name. A ``__node`` remapping
                passed on the command line takes precedence.

        Returns:
            YasminNode: A reference to the YasminNode instance.

        Raises:
            RuntimeError: Raised if the creation of the instance fails.
        """
        with YasminNode._lock:
            if not rclpy.ok():
                rclpy.init()

            instance = YasminNode._instance
            if instance is not None and not instance.context.ok():
                YasminNode._release_locked()

            if YasminNode._instance is None:
                YasminNode(node_name)

            return YasminNode._instance

    @staticmethod
    def destroy_instance() -> None:
        """
        Destroy the singleton instance if it exists.

        Cleanup also runs after the context has been shut down.
        """
        with YasminNode._lock:
            YasminNode._release_locked()

    @staticmethod
    def _release_locked() -> None:
        instance = YasminNode._instance
        if instance is None:
            return
        YasminNode._instance = None

        from yasmin_ros.ros_clients_cache import ROSClientsCache

        ROSClientsCache.clear_for_node(instance)
        if yasmin_ros.logger_node is instance:
            yasmin_ros.logger_node = None
        instance.shutdown()

    def __init__(self, node_name: Optional[str] = None) -> None:
        """
        Initializes the node and starts an Executor for its callbacks.

        Args:
            node_name (str, optional): Node name. Defaults to a unique
                random name.

        Raises:
            RuntimeError: Raised when an attempt is made to create
            more than one instance of YasminNode.
        """
        with YasminNode._lock:
            if YasminNode._instance is not None:
                raise RuntimeError("This class is a Singleton")

            if not node_name:
                node_name = f"yasmin_{str(uuid.uuid4()).replace('-', '')[:16]}_node"
            super().__init__(node_name)

            ## Executor for managing node operations.
            self._executor = Executor()
            self._executor.add_node(self)

            ## Thread to execute the spinning of the node.
            self._spin_thread: Thread = Thread(target=self._executor.spin, daemon=True)
            self._spin_thread.start()

            self._destroyed = False
            YasminNode._instance = self

    def shutdown(self) -> None:
        """
        Stop the executor thread and destroy the node.

        Safe to call more than once and after the context has been shut down.
        """
        executor, self._executor = self._executor, None
        if executor is not None:
            executor.remove_node(self)
            executor.shutdown()

        spin_thread, self._spin_thread = self._spin_thread, None
        if spin_thread is not None and spin_thread is not current_thread():
            spin_thread.join()

        if not self._destroyed:
            self._destroyed = True
            self.destroy_node()


atexit.register(YasminNode.destroy_instance)
