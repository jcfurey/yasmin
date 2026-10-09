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

#ifndef YASMIN_PCL__IO__POINT_CLOUD2_MONITOR_STATE_HPP_
#define YASMIN_PCL__IO__POINT_CLOUD2_MONITOR_STATE_HPP_

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin/types.hpp"

namespace yasmin_pcl::io {

/**
 * @brief Receives PointCloud2 messages natively and stores the next one.
 *
 * The subscription is created by configure() from the `topic`, `qos` and
 * `queue_size` parameters and stays active between executions. Execution
 * stores the oldest queued message in `output_cloud` as a shared pointer,
 * without copying it. With the default `queue_size` of 1 that is the latest
 * message; `wait_for_new_message` discards queued messages on entry.
 *
 * Messages are received and stored in C++, so a Python state machine can use
 * this state (through the factory) to keep a cloud pipeline native.
 */
class PointCloud2MonitorState : public yasmin::State {
public:
  YASMIN_PTR_ALIASES(PointCloud2MonitorState)

  /** @brief Construct the state using the default YasminNode. */
  PointCloud2MonitorState();

  /**
   * @brief Construct the state using an application-owned node.
   * @param node Node used for the subscription; it must be spun.
   */
  explicit PointCloud2MonitorState(const rclcpp::Node::SharedPtr &node);

  /** @brief Create or update the subscription from the parameters. */
  void configure() override;

  /**
   * @brief Wait for a message and store it in `output_cloud`.
   * @param blackboard The shared blackboard.
   * @return `succeeded`, `timeout` or `canceled`.
   */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

  /** @brief Cancel and wake a waiting execution. */
  void cancel_state() override;

private:
  /// @brief Messages shared with the subscription callback.
  struct Inbox {
    std::mutex mutex;
    std::condition_variable cond;
    std::deque<std::shared_ptr<sensor_msgs::msg::PointCloud2>> messages;
    std::size_t capacity{1};
  };

  std::shared_ptr<Inbox> current_inbox();

  rclcpp::Node::SharedPtr node_;
  std::mutex inbox_mutex_;
  std::shared_ptr<Inbox> inbox_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::string subscribed_topic_;
  std::string subscribed_qos_;
  int subscribed_depth_{0};
  double timeout_sec_{-1.0};
  bool wait_for_new_message_{false};
};

} // namespace yasmin_pcl::io

#endif // YASMIN_PCL__IO__POINT_CLOUD2_MONITOR_STATE_HPP_
