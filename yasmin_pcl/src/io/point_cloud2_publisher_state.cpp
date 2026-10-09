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

#include "yasmin_pcl/io/point_cloud2_publisher_state.hpp"

#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_validation.hpp"
#include "yasmin_pcl/common/qos.hpp"
#include "yasmin_pcl/common/ros_cloud_bridge.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/ros_clients_cache.hpp"
#include "yasmin_ros/yasmin_node.hpp"

namespace yasmin_pcl::io {

using sensor_msgs::msg::PointCloud2;
namespace outcomes = yasmin_ros::basic_outcomes;

PointCloud2PublisherState::PointCloud2PublisherState()
    : PointCloud2PublisherState(nullptr) {}

PointCloud2PublisherState::PointCloud2PublisherState(
    const rclcpp::Node::SharedPtr &node)
    : yasmin::State({outcomes::SUCCEED, outcomes::ABORT}), node_(node) {
  this->set_description(
      "Publishes the sensor_msgs/msg/PointCloud2 stored in 'input_cloud'.");
  this->set_outcome_description(outcomes::SUCCEED, "The cloud was published.");
  this->set_outcome_description(outcomes::ABORT,
                                "The input cloud was missing or invalid.");
  this->add_input_key(
      "input_cloud",
      "Cloud stored as std::shared_ptr<sensor_msgs::msg::PointCloud2>, a "
      "PointCloud2 by value, or serialized PointCloud2 bytes.");
  this->declare_parameter<std::string>(
      "topic", "Topic to publish to (relative names follow the namespace).",
      "");
  this->declare_parameter<std::string>(
      "qos",
      "'reliable' (also received by best-effort subscribers), 'sensor_data' "
      "or 'system_default'.",
      "reliable");
  this->declare_parameter<int>("queue_size", "Publisher history depth.", 1);
}

void PointCloud2PublisherState::configure() {
  const auto topic = this->get_parameter<std::string>("topic");
  if (topic.empty()) {
    throw std::invalid_argument(
        "PointCloud2PublisherState parameter 'topic' must be set");
  }
  const auto qos = common::make_qos(this->get_parameter<std::string>("qos"),
                                    this->get_parameter<int>("queue_size"));
  if (!this->node_) {
    this->node_ = yasmin_ros::YasminNode::get_instance();
  }
  this->publisher_ =
      yasmin_ros::ROSClientsCache::get_or_create_publisher<PointCloud2>(
          this->node_, topic, qos);
}

std::string
PointCloud2PublisherState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    if (!this->publisher_) {
      this->configure();
    }
    const auto cloud = common::get_ros_cloud(*blackboard, "input_cloud");
    if (!cloud) {
      YASMIN_LOG_WARN("Input ROS point cloud pointer is null");
      return outcomes::ABORT;
    }
    common::validate_cloud(*cloud);
    // The blackboard keeps its cloud; publishing an owned copy lets
    // intra-process subscribers take it without another copy.
    this->publisher_->publish(std::make_unique<PointCloud2>(*cloud));
    return outcomes::SUCCEED;
  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("Failed to publish point cloud: %s", e.what());
    return outcomes::ABORT;
  }
}

} // namespace yasmin_pcl::io

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::io::PointCloud2PublisherState,
                       yasmin::State)
