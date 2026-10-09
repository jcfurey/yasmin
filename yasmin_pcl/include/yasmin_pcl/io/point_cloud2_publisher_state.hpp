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

#ifndef YASMIN_PCL__IO__POINT_CLOUD2_PUBLISHER_STATE_HPP_
#define YASMIN_PCL__IO__POINT_CLOUD2_PUBLISHER_STATE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin/types.hpp"

namespace yasmin_pcl::io {

/**
 * @brief Publishes the PointCloud2 stored in `input_cloud`.
 *
 * The publisher is created by configure() from the `topic`, `qos` and
 * `queue_size` parameters. The input may be any representation accepted by
 * common::get_ros_cloud() and is validated before publication.
 */
class PointCloud2PublisherState : public yasmin::State {
public:
  YASMIN_PTR_ALIASES(PointCloud2PublisherState)

  /** @brief Construct the state using the default YasminNode. */
  PointCloud2PublisherState();

  /**
   * @brief Construct the state using an application-owned node.
   * @param node Node used for the publisher.
   */
  explicit PointCloud2PublisherState(const rclcpp::Node::SharedPtr &node);

  /** @brief Create or reuse the publisher from the parameters. */
  void configure() override;

  /**
   * @brief Publish the cloud stored in `input_cloud`.
   * @param blackboard The shared blackboard.
   * @return `succeeded` or `aborted`.
   */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

} // namespace yasmin_pcl::io

#endif // YASMIN_PCL__IO__POINT_CLOUD2_PUBLISHER_STATE_HPP_
