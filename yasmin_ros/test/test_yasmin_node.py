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

import unittest

import rclpy
from rclpy.parameter import Parameter

from yasmin import Blackboard
from yasmin_ros import TfBufferState
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin_ros.yasmin_node import YasminNode


class TestYasminNodeLifecycle(unittest.TestCase):
    def setUp(self):
        if not rclpy.ok():
            rclpy.init()

    def tearDown(self):
        YasminNode.destroy_instance()
        if rclpy.ok():
            rclpy.shutdown()

    def test_destroy_after_context_shutdown_still_cleans_up(self):
        node = YasminNode.get_instance()
        spin_thread = node._spin_thread
        rclpy.shutdown()

        YasminNode.destroy_instance()

        spin_thread.join(timeout=2.0)
        self.assertFalse(spin_thread.is_alive())
        self.assertIsNone(node._executor)
        self.assertTrue(node._destroyed)
        self.assertIsNone(YasminNode._instance)

    def test_reinitialized_context_gets_a_new_node(self):
        old_node = YasminNode.get_instance()
        rclpy.shutdown()
        rclpy.init()

        new_node = YasminNode.get_instance()

        self.assertIsNot(new_node, old_node)
        self.assertTrue(new_node.context.ok())
        self.assertTrue(old_node._destroyed)

    def test_named_instance(self):
        node = YasminNode.get_instance("custom_yasmin_node")
        self.assertEqual(node.get_name(), "custom_yasmin_node")
        # The name only applies when the instance is created.
        self.assertIs(YasminNode.get_instance("other_name"), node)
        self.assertIs(YasminNode.get_instance(), node)

    def test_shutdown_is_idempotent(self):
        node = YasminNode.get_instance()
        YasminNode.destroy_instance()
        node.shutdown()
        YasminNode.destroy_instance()


class TestTfBufferStateClock(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node(
            "tf_clock_test",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )

    def tearDown(self):
        self.node.destroy_node()
        YasminNode.destroy_instance()
        rclpy.shutdown()

    def test_buffer_uses_node_ros_clock(self):
        state = TfBufferState(node=self.node)
        state.configure()
        blackboard = Blackboard()

        self.assertEqual(SUCCEED, state(blackboard))

        buffer = blackboard["tf_buffer"]
        if not hasattr(buffer, "clock"):
            self.skipTest("this tf2_ros Buffer has no clock (Humble)")
        self.assertIs(buffer.clock, self.node.get_clock())

    def test_replacing_buffer_releases_previous_endpoints(self):
        state = TfBufferState(node=self.node)
        state.configure()
        blackboard = Blackboard()
        self.assertEqual(SUCCEED, state(blackboard))
        first_buffer = blackboard["tf_buffer"]
        subscriptions = len(list(self.node.subscriptions))
        services = len(list(self.node.services))

        # Reuse while the cache time is unchanged.
        self.assertEqual(SUCCEED, state(blackboard))
        self.assertIs(blackboard["tf_buffer"], first_buffer)

        state.set_parameter("cache_time_sec", 5.0)
        state.configure()
        self.assertEqual(SUCCEED, state(blackboard))

        self.assertIsNot(blackboard["tf_buffer"], first_buffer)
        self.assertEqual(len(list(self.node.subscriptions)), subscriptions)
        self.assertEqual(len(list(self.node.services)), services)


if __name__ == "__main__":
    unittest.main()
