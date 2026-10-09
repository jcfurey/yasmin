// Copyright (C) 2024 Miguel Ángel González Santamarta
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

#ifndef YASMIN_ROS__YASMIN_NODE_HPP_
#define YASMIN_ROS__YASMIN_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

namespace yasmin_ros {

/**
 * @class YasminNode
 * @brief A ROS 2 node for managing and handling YASMIN-based applications.
 *
 * YasminNode is a singleton class derived from rclcpp::Node and integrates
 * custom functionalities for executing specific tasks in a ROS 2 environment.
 */
class YasminNode : public rclcpp::Node {
public:
  /** @brief Shared pointer type for YasminNode. */
  using SharedPtr = std::shared_ptr<YasminNode>;

protected:
  /**
   * @brief Default constructor. Initializes the node with a unique name.
   */
  explicit YasminNode();

  /**
   * @brief Initializes the node with the given name and options.
   *
   * @param node_name Node name; empty selects a unique random name.
   * @param options Node options, including the context the node belongs to.
   */
  YasminNode(const std::string &node_name, const rclcpp::NodeOptions &options);

public:
  /** @brief Deleted copy constructor (singleton). */
  YasminNode(YasminNode &other) = delete;

  /**
   * @brief Destructor. Cleans up resources.
   */
  ~YasminNode();

  /** @brief Deleted copy assignment (singleton). */
  void operator=(const YasminNode &) = delete;

  /**
   * @brief Provides access to the singleton instance of YasminNode.
   *
   * This method ensures there is only one instance of YasminNode running.
   * If rclcpp is not initialized, the default context is initialized with
   * the process arguments (as rclpy.init() uses sys.argv), so `--ros-args`
   * remappings and parameters apply to this node.
   *
   * @return A shared pointer to the singleton instance of YasminNode.
   */
  static YasminNode::SharedPtr get_instance();

  /**
   * @brief Provides access to the singleton, creating it with a given name.
   *
   * The name and options apply only if this call creates the node; an
   * existing instance is returned unchanged. A `__node` remapping passed on
   * the command line still takes precedence. An instance whose context has
   * been shut down is replaced, as with get_instance(). The default context
   * is initialized if needed; another context must already be initialized.
   *
   * @param node_name Node name; empty selects a unique random name.
   * @param options Node options, including the context.
   * @throws std::invalid_argument if a non-default context is not initialized.
   * @return A shared pointer to the singleton instance of YasminNode.
   */
  static YasminNode::SharedPtr
  get_instance(const std::string &node_name,
               const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the singleton instance if it exists.
   */
  static void destroy_instance();

private:
  /**
   * @brief Stop the executor thread and remove the node from the executor.
   */
  void stop_executor();

  /// @brief Executor spinning this node in spin_thread.
  std::unique_ptr<rclcpp::Executor> executor;
  /// @brief Thread for spinning the node.
  std::unique_ptr<std::thread> spin_thread;
  /// @brief Set when the executor's spin() has returned.
  std::atomic_bool spin_finished{false};
};

} // namespace yasmin_ros

#endif // YASMIN_ROS__YASMIN_NODE_HPP_
