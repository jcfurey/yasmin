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

#include "yasmin_pcl/io/point_cloud2_monitor_state.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/common/qos.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"

namespace yasmin_pcl::io {

using sensor_msgs::msg::PointCloud2;
namespace outcomes = yasmin_ros::basic_outcomes;

PointCloud2MonitorState::PointCloud2MonitorState()
    : PointCloud2MonitorState(nullptr) {}

PointCloud2MonitorState::PointCloud2MonitorState(
    const rclcpp::Node::SharedPtr &node)
    : yasmin::State({outcomes::SUCCEED, outcomes::TIMEOUT, outcomes::CANCEL}),
      node_(node), inbox_(std::make_shared<Inbox>()) {
  this->set_description(
      "Subscribes to a sensor_msgs/msg/PointCloud2 topic in C++ and stores "
      "the next message in 'output_cloud' without copying it.");
  this->set_outcome_description(outcomes::SUCCEED, "A cloud was received.");
  this->set_outcome_description(outcomes::TIMEOUT,
                                "No cloud arrived within 'timeout_sec'.");
  this->set_outcome_description(outcomes::CANCEL, "The state was canceled.");
  this->add_output_key("output_cloud",
                       "Received cloud stored as "
                       "std::shared_ptr<sensor_msgs::msg::PointCloud2>.");
  this->declare_parameter<std::string>(
      "topic", "Topic to subscribe to (relative names follow the namespace).",
      "");
  this->declare_parameter<std::string>(
      "qos", "'sensor_data' (best effort), 'reliable' or 'system_default'.",
      "sensor_data");
  this->declare_parameter<int>(
      "queue_size",
      "Messages kept while the state is inactive; 1 keeps the latest.", 1);
  this->declare_parameter<double>(
      "timeout_sec", "Seconds to wait for a message; <= 0 waits indefinitely.",
      -1.0);
  this->declare_parameter<bool>(
      "wait_for_new_message",
      "Discard queued messages on entry and wait for the next one.", false);
}

std::shared_ptr<PointCloud2MonitorState::Inbox>
PointCloud2MonitorState::current_inbox() {
  std::lock_guard<std::mutex> lock(this->inbox_mutex_);
  return this->inbox_;
}

void PointCloud2MonitorState::configure() {
  const auto topic = this->get_parameter<std::string>("topic");
  if (topic.empty()) {
    throw std::invalid_argument(
        "PointCloud2MonitorState parameter 'topic' must be set");
  }
  const auto qos_name = this->get_parameter<std::string>("qos");
  const int depth = this->get_parameter<int>("queue_size");
  this->timeout_sec_ = this->get_parameter<double>("timeout_sec");
  this->wait_for_new_message_ =
      this->get_parameter<bool>("wait_for_new_message");

  if (this->subscription_ && topic == this->subscribed_topic_ &&
      qos_name == this->subscribed_qos_ && depth == this->subscribed_depth_) {
    return;
  }
  const auto qos = common::make_qos(qos_name, depth);
  if (!this->node_) {
    this->node_ = yasmin_ros::YasminNode::get_instance();
  }

  // A new inbox per subscription: late messages of a replaced subscription
  // cannot reach it. The callback owns the inbox, not this state.
  auto inbox = std::make_shared<Inbox>();
  inbox->capacity = static_cast<std::size_t>(depth);
  this->subscription_ = this->node_->create_subscription<PointCloud2>(
      topic, qos, [inbox](std::shared_ptr<PointCloud2> msg) {
        {
          std::lock_guard<std::mutex> lock(inbox->mutex);
          inbox->messages.push_back(std::move(msg));
          while (inbox->messages.size() > inbox->capacity) {
            inbox->messages.pop_front();
          }
        }
        inbox->cond.notify_all();
      });
  {
    std::lock_guard<std::mutex> lock(this->inbox_mutex_);
    this->inbox_ = std::move(inbox);
  }
  this->subscribed_topic_ = topic;
  this->subscribed_qos_ = qos_name;
  this->subscribed_depth_ = depth;
  YASMIN_LOG_INFO("Subscribed to point clouds on '%s'",
                  this->subscription_->get_topic_name());
}

std::string
PointCloud2MonitorState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  if (!this->subscription_) {
    this->configure();
  }
  auto inbox = this->current_inbox();
  std::unique_lock<std::mutex> lock(inbox->mutex);
  if (this->wait_for_new_message_) {
    inbox->messages.clear();
  }
  const auto ready = [&]() {
    return !inbox->messages.empty() || this->is_canceled();
  };
  if (this->timeout_sec_ > 0) {
    if (!inbox->cond.wait_for(
            lock, std::chrono::duration<double>(this->timeout_sec_), ready)) {
      YASMIN_LOG_WARN("No point cloud received on '%s' within %.3f s",
                      this->subscribed_topic_.c_str(), this->timeout_sec_);
      return outcomes::TIMEOUT;
    }
  } else {
    inbox->cond.wait(lock, ready);
  }
  if (this->is_canceled()) {
    return outcomes::CANCEL;
  }
  auto cloud = std::move(inbox->messages.front());
  inbox->messages.pop_front();
  lock.unlock();

  blackboard->set<common::RosPointCloud2Ptr>("output_cloud", std::move(cloud));
  return outcomes::SUCCEED;
}

void PointCloud2MonitorState::cancel_state() {
  auto inbox = this->current_inbox();
  {
    std::lock_guard<std::mutex> lock(inbox->mutex);
    yasmin::State::cancel_state();
  }
  inbox->cond.notify_all();
}

} // namespace yasmin_pcl::io

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::io::PointCloud2MonitorState, yasmin::State)
