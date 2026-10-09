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
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "test_utils.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/common/ros_cloud_bridge.hpp"
#include "yasmin_pcl/io/point_cloud2_monitor_state.hpp"
#include "yasmin_pcl/io/point_cloud2_publisher_state.hpp"
#include "yasmin_ros/ros_clients_cache.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using namespace std::chrono_literals;
using sensor_msgs::msg::PointCloud2;
using yasmin_pcl::common::RosPointCloud2Ptr;
using yasmin_pcl::io::PointCloud2MonitorState;
using yasmin_pcl::io::PointCloud2PublisherState;

namespace {

RosPointCloud2Ptr cloud_with_x(float x) {
  return yasmin_pcl::test::create_ros_cloud_ptr({{x, 0.0F, 0.0F}});
}

float first_x(const PointCloud2 &cloud) {
  float x;
  std::memcpy(&x, cloud.data.data(), sizeof(x));
  return x;
}

bool wait_until(const std::function<bool()> &condition,
                std::chrono::milliseconds timeout = 5s) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return condition();
}

} // namespace

class TestPointCloud2TopicStates : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() {
    yasmin_ros::ROSClientsCache::clear_all();
    yasmin_ros::YasminNode::destroy_instance();
    rclcpp::shutdown();
  }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("cloud_topic_test");
    executor_.add_node(node_);
    spin_thread_ = std::thread([this]() {
      while (running_.load()) {
        executor_.spin_once(10ms);
      }
    });
  }

  void TearDown() override {
    running_.store(false);
    spin_thread_.join();
    executor_.remove_node(node_);
  }

  // Run the monitor while publishing until it returns.
  std::string run_monitor(PointCloud2MonitorState &state,
                          yasmin::Blackboard::SharedPtr blackboard,
                          const rclcpp::Publisher<PointCloud2>::SharedPtr &pub,
                          const RosPointCloud2Ptr &cloud) {
    auto result = std::async(std::launch::async,
                             [&]() { return state(blackboard); });
    while (result.wait_for(50ms) != std::future_status::ready) {
      pub->publish(*cloud);
    }
    return result.get();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::atomic_bool running_{true};
  std::thread spin_thread_;
};

TEST_F(TestPointCloud2TopicStates, MonitorReceivesBestEffortCloud) {
  auto pub = node_->create_publisher<PointCloud2>("cloud_in",
                                                  rclcpp::SensorDataQoS());
  PointCloud2MonitorState state;
  state.set_parameter<std::string>("topic", "cloud_in");
  // Python-style parameter types are accepted.
  state.set_parameter<std::int64_t>("queue_size", 1);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  ASSERT_EQ(run_monitor(state, blackboard, pub, cloud_with_x(4.0F)),
            "succeeded");
  const auto cloud = blackboard->get<RosPointCloud2Ptr>("output_cloud");
  ASSERT_NE(cloud, nullptr);
  EXPECT_FLOAT_EQ(first_x(*cloud), 4.0F);
}

TEST_F(TestPointCloud2TopicStates, MonitorTimesOut) {
  PointCloud2MonitorState state;
  state.set_parameter<std::string>("topic", "cloud_silent");
  state.set_parameter<double>("timeout_sec", 0.2);
  state.configure();
  auto blackboard = yasmin::Blackboard::make_shared();
  EXPECT_EQ(state(blackboard), "timeout");
}

TEST_F(TestPointCloud2TopicStates, CancelWakesWaitingMonitor) {
  PointCloud2MonitorState state;
  state.set_parameter<std::string>("topic", "cloud_never");
  state.configure();
  auto blackboard = yasmin::Blackboard::make_shared();
  auto result = std::async(std::launch::async,
                           [&]() { return state(blackboard); });
  ASSERT_TRUE(wait_until([&]() { return state.is_running(); }));
  state.cancel_state();
  ASSERT_EQ(result.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(result.get(), "canceled");
}

TEST_F(TestPointCloud2TopicStates, WaitForNewMessageDiscardsQueuedCloud) {
  auto pub = node_->create_publisher<PointCloud2>(
      "cloud_fresh", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  PointCloud2MonitorState state;
  state.set_parameter<std::string>("topic", "cloud_fresh");
  state.set_parameter<std::string>("qos", "reliable");
  state.configure();
  auto blackboard = yasmin::Blackboard::make_shared();
  ASSERT_EQ(run_monitor(state, blackboard, pub, cloud_with_x(1.0F)),
            "succeeded");

  // Queue a cloud while the state is inactive, then ask for a newer one.
  pub->publish(*cloud_with_x(2.0F));
  std::this_thread::sleep_for(200ms);
  state.set_parameter<bool>("wait_for_new_message", true);
  state.configure();
  ASSERT_EQ(run_monitor(state, blackboard, pub, cloud_with_x(3.0F)),
            "succeeded");
  EXPECT_FLOAT_EQ(
      first_x(*blackboard->get<RosPointCloud2Ptr>("output_cloud")), 3.0F);
}

TEST_F(TestPointCloud2TopicStates, PublisherReachesReliableAndBestEffort) {
  std::atomic_int reliable{0};
  std::atomic_int best_effort{0};
  auto reliable_sub = node_->create_subscription<PointCloud2>(
      "cloud_out", rclcpp::QoS(10).reliable(),
      [&](PointCloud2::SharedPtr msg) {
        if (first_x(*msg) == 7.0F) {
          ++reliable;
        }
      });
  auto best_effort_sub = node_->create_subscription<PointCloud2>(
      "cloud_out", rclcpp::SensorDataQoS(), [&](PointCloud2::SharedPtr msg) {
        if (first_x(*msg) == 7.0F) {
          ++best_effort;
        }
      });

  PointCloud2PublisherState state;
  state.set_parameter<std::string>("topic", "cloud_out");
  state.configure();
  auto blackboard = yasmin::Blackboard::make_shared();
  // Serialized bytes, as a Python state would store them.
  blackboard->set<yasmin_pcl::common::SerializedCloud>(
      "input_cloud", yasmin_ros::serialize_interface<PointCloud2>(
                         *cloud_with_x(7.0F)));
  EXPECT_TRUE(wait_until([&]() {
    EXPECT_EQ(state(blackboard), "succeeded");
    return reliable.load() > 0 && best_effort.load() > 0;
  }));
}

TEST_F(TestPointCloud2TopicStates, PublisherRejectsInvalidCloud) {
  PointCloud2PublisherState state;
  state.set_parameter<std::string>("topic", "cloud_invalid");
  state.configure();
  auto cloud = cloud_with_x(1.0F);
  cloud->data.clear();
  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<RosPointCloud2Ptr>("input_cloud", cloud);
  EXPECT_EQ(state(blackboard), "aborted");
}

TEST_F(TestPointCloud2TopicStates, ConfigureRejectsInvalidParameters) {
  PointCloud2MonitorState missing_topic;
  EXPECT_THROW(missing_topic.configure(), std::invalid_argument);

  PointCloud2MonitorState bad_qos;
  bad_qos.set_parameter<std::string>("topic", "cloud");
  bad_qos.set_parameter<std::string>("qos", "fast");
  EXPECT_THROW(bad_qos.configure(), std::invalid_argument);

  PointCloud2PublisherState bad_depth;
  bad_depth.set_parameter<std::string>("topic", "cloud");
  bad_depth.set_parameter<int>("queue_size", 0);
  EXPECT_THROW(bad_depth.configure(), std::invalid_argument);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
