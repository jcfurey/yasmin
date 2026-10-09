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
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "yasmin_ros/yasmin_node.hpp"

using yasmin_ros::YasminNode;

class TestYasminNode : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override {
    YasminNode::destroy_instance();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(TestYasminNode, NamedInstance) {
  auto node = YasminNode::get_instance("custom_yasmin_node");
  EXPECT_STREQ(node->get_name(), "custom_yasmin_node");
  // The name only applies when the instance is created.
  EXPECT_EQ(YasminNode::get_instance("other_name"), node);
  EXPECT_EQ(YasminNode::get_instance(), node);
}

TEST_F(TestYasminNode, NodeOptionsAreApplied) {
  auto node = YasminNode::get_instance(
      "options_node",
      rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));
  EXPECT_TRUE(node->get_parameter("use_sim_time").as_bool());
  EXPECT_EQ(node->get_clock()->get_clock_type(), RCL_ROS_TIME);
}

TEST_F(TestYasminNode, CustomContextIsSpun) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto node = YasminNode::get_instance("custom_context",
                                       rclcpp::NodeOptions().context(context));
  EXPECT_EQ(node->get_node_base_interface()->get_context(), context);

  std::atomic_bool fired{false};
  auto timer = node->create_wall_timer(std::chrono::milliseconds(10),
                                       [&fired]() { fired.store(true); });
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!fired.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_TRUE(fired.load());
  timer.reset();
  node.reset();
  YasminNode::destroy_instance();
  context->shutdown("test done");
}

TEST_F(TestYasminNode, RejectsUninitializedCustomContext) {
  auto context = std::make_shared<rclcpp::Context>();
  EXPECT_THROW(YasminNode::get_instance("uninitialized",
                                        rclcpp::NodeOptions().context(context)),
               std::invalid_argument);
}

namespace {
std::string self_path;
} // namespace

// Runs in a child process started with --ros-args by the test below.
TEST(TestYasminNodeAutoInit, ChildUsesProcessArguments) {
  if (std::getenv("YASMIN_NODE_TEST_CHILD") == nullptr) {
    GTEST_SKIP() << "Runs only as the child of AutoInitUsesProcessArguments";
  }
  ASSERT_FALSE(rclcpp::ok());
  auto node = YasminNode::get_instance();
  EXPECT_STREQ(node->get_namespace(), "/cmdline_ns");
  EXPECT_TRUE(node->get_parameter("use_sim_time").as_bool());
  node.reset();
  YasminNode::destroy_instance();
  rclcpp::shutdown();
}

TEST(TestYasminNodeAutoInit, AutoInitUsesProcessArguments) {
  const std::string command =
      "YASMIN_NODE_TEST_CHILD=1 '" + self_path +
      "' --gtest_filter=TestYasminNodeAutoInit.ChildUsesProcessArguments "
      "--ros-args -r __ns:=/cmdline_ns -p use_sim_time:=true";
  EXPECT_EQ(std::system(command.c_str()), 0);
}

// Runs in a child process started with --ros-args by the test below.
TEST(TestYasminNodeAutoInit, ChildCompanionTakesSuffixedName) {
  if (std::getenv("YASMIN_NODE_TEST_CHILD") == nullptr) {
    GTEST_SKIP() << "Runs only as the child of CompanionSkipsNodeNameRemap";
  }
  YasminNode::configure_as_companion("_cpp");
  auto node = YasminNode::get_instance();
  EXPECT_STREQ(node->get_name(), "fsm_cpp");
  EXPECT_STREQ(node->get_namespace(), "/cmdline_ns");
  EXPECT_TRUE(node->get_parameter("use_sim_time").as_bool());
  node.reset();
  YasminNode::destroy_instance();
  rclcpp::shutdown();
}

TEST(TestYasminNodeAutoInit, CompanionSkipsNodeNameRemap) {
  const std::string command =
      "YASMIN_NODE_TEST_CHILD=1 '" + self_path +
      "' --gtest_filter=TestYasminNodeAutoInit.ChildCompanionTakesSuffixedName "
      "--ros-args -r __ns:=/cmdline_ns -r __node:=fsm -p use_sim_time:=true";
  EXPECT_EQ(std::system(command.c_str()), 0);
}

TEST_F(TestYasminNode, CompanionConfigurationIgnoredOnceNodeExists) {
  auto node = YasminNode::get_instance("main_node");
  YasminNode::configure_as_companion("_cpp");
  EXPECT_EQ(YasminNode::get_instance(), node);
  node.reset();
  YasminNode::destroy_instance();
  // No companion configuration was stored: a new default node is unnamed.
  EXPECT_EQ(
      std::string(YasminNode::get_instance()->get_name()).rfind("yasmin_", 0),
      0U);
}

TEST_F(TestYasminNode, ShutdownContextGetsNewNode) {
  std::weak_ptr<YasminNode> old_node = YasminNode::get_instance();
  rclcpp::shutdown();

  // Releases the stale node before re-initializing the default context.
  auto new_node = YasminNode::get_instance();
  EXPECT_TRUE(old_node.expired());
  EXPECT_TRUE(rclcpp::ok(new_node->get_node_base_interface()->get_context()));
}

int main(int argc, char **argv) {
  self_path = argv[0];
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
