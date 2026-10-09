// Copyright (C) 2026 Maik Knof
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

#ifndef YASMIN_ROS__TF_BUFFER_STATE_HPP_
#define YASMIN_ROS__TF_BUFFER_STATE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#if __has_include(<tf2_ros/buffer.hpp>)
#include <tf2_ros/buffer.hpp>
#else
#include <tf2_ros/buffer.h>
#endif

#if __has_include(<tf2_ros/transform_listener.hpp>)
#include <tf2_ros/transform_listener.hpp>
#else
#include <tf2_ros/transform_listener.h>
#endif

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/yasmin_node.hpp"

namespace yasmin_ros {

/**
 * @brief State that creates a tf2 buffer and transform listener.
 *
 * The created objects are stored in the blackboard under the keys
 * "tf_buffer" and "tf_listener" so that following states can reuse the same
 * listener for transform lookups.
 */
class TfBufferState : public yasmin::State {
public:
  /**
   * @brief Shared pointer type for TfBufferState.
   */
  YASMIN_PTR_ALIASES(TfBufferState)

  /**
   * @brief Construct a new TfBufferState using the default YasminNode.
   */
  TfBufferState();

  /**
   * @brief Construct a new TfBufferState using an application-owned node.
   *
   * The listener does not spin its own thread, so the node must be spun by
   * an executor. The buffer uses the node's clock (and thus `use_sim_time`).
   *
   * @param node Node used for the clock and the tf subscriptions.
   */
  explicit TfBufferState(const rclcpp::Node::SharedPtr &node);

  /**
   * @brief Read the configured parameters.
   */
  void configure() override;

  /**
   * @brief Store the tf2 buffer/listener pair in the blackboard.
   *
   * The pair is created on first execution and reused afterwards, so the
   * transform history survives across executions. It is recreated when the
   * cache time changes.
   *
   * @param blackboard Blackboard used to share the created objects.
   * @return The execution outcome.
   */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  /// @brief Node used to create the tf2 listener subscriptions.
  rclcpp::Node::SharedPtr node_;
  /// @brief Buffer cache duration in seconds.
  double cache_time_sec_;
  /// @brief Buffer reused across executions.
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  /// @brief Listener feeding tf_buffer_.
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  /// @brief Cache time used to create tf_buffer_.
  double buffer_cache_time_sec_{0.0};
};

} // namespace yasmin_ros

#endif // YASMIN_ROS__TF_BUFFER_STATE_HPP_
