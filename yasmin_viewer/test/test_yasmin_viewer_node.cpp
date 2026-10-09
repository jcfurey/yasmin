// Copyright (C) 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "yasmin_msgs/msg/state_machine.hpp"
#include "yasmin_viewer/yasmin_viewer_node.hpp"

using namespace std::chrono_literals;
using yasmin_msgs::msg::StateMachine;

namespace {

StateMachine machine(const std::string &name, const std::string &child) {
  StateMachine msg;
  msg.states.resize(2);
  msg.states[0].id = 0;
  msg.states[0].parent = -1;
  msg.states[0].name = name;
  msg.states[0].is_fsm = true;
  msg.states[1].id = 1;
  msg.states[1].parent = 0;
  msg.states[1].name = child;
  return msg;
}

} // namespace

TEST(TestYasminViewerNode, KeepsSameNamedMachinesFromNamespacedPublishers) {
  rclcpp::init(0, nullptr);
  {
    auto viewer = std::make_shared<yasmin_viewer::YasminViewerNode>(
        rclcpp::NodeOptions()
            .arguments({"--ros-args", "-r", "__ns:=/robot1"})
            .parameter_overrides({{"host", "127.0.0.1"}, {"port", 0}}));
    auto node = std::make_shared<rclcpp::Node>("publishers", "/robot1");
    auto other_ns = std::make_shared<rclcpp::Node>("publishers", "/robot2");
    // Relative topics resolve in each node's namespace.
    auto first = node->create_publisher<StateMachine>("fsm_viewer", 10);
    auto second = node->create_publisher<StateMachine>("fsm_viewer", 10);
    auto elsewhere = other_ns->create_publisher<StateMachine>("fsm_viewer", 10);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(viewer);
    std::atomic_bool running{true};
    std::thread spin([&]() {
      while (running.load()) {
        executor.spin_once(10ms);
      }
    });

    std::string json;
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      first->publish(machine("SAME", "first_child"));
      second->publish(machine("SAME", "second_child"));
      elsewhere->publish(machine("OTHER_ROBOT", "child"));
      json = viewer->get_fsms_json();
      if (json.find("first_child") != std::string::npos &&
          json.find("second_child") != std::string::npos) {
        break;
      }
      std::this_thread::sleep_for(50ms);
    }
    running.store(false);
    spin.join();

    EXPECT_NE(json.find("\"SAME\""), std::string::npos) << json;
    EXPECT_NE(json.find("\"SAME (2)\""), std::string::npos) << json;
    EXPECT_NE(json.find("first_child"), std::string::npos) << json;
    EXPECT_NE(json.find("second_child"), std::string::npos) << json;
    EXPECT_EQ(json.find("OTHER_ROBOT"), std::string::npos) << json;
  }
  rclcpp::shutdown();
}
