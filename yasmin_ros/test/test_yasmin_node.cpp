// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#include <memory>
#include <stdexcept>

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

TEST_F(TestYasminNode, RejectsForeignContext) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  EXPECT_THROW(YasminNode::get_instance(
                   "foreign", rclcpp::NodeOptions().context(context)),
               std::invalid_argument);
  context->shutdown("test done");
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
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
