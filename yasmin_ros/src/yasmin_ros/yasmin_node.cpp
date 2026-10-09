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

#include "yasmin_ros/yasmin_node.hpp"
#include "yasmin_ros/ros_clients_cache.hpp"
#include "yasmin_ros/ros_logs.hpp"

#include <chrono>
#include <fstream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#if __has_include("rclcpp/version.h")
#include "rclcpp/version.h"
#endif

using namespace yasmin_ros;

namespace {
YasminNode::SharedPtr &get_yasmin_node_instance() {
  static YasminNode::SharedPtr instance;
  return instance;
}

void release_instance(YasminNode::SharedPtr &instance) {
  if (instance == nullptr) {
    return;
  }
  reset_logger_node(instance.get());
  ROSClientsCache::clear_for_node(instance);
  instance.reset();
}

std::mutex &get_yasmin_node_instance_mutex() {
  static std::mutex mutex;
  return mutex;
}

// Library code has no argc/argv; Linux exposes them in /proc/self/cmdline.
std::vector<std::string> read_process_arguments() {
  std::vector<std::string> arguments;
  std::ifstream cmdline("/proc/self/cmdline", std::ios::binary);
  std::string argument;
  while (std::getline(cmdline, argument, '\0')) {
    arguments.push_back(argument);
  }
  return arguments;
}

// Like rclpy.init(), use the process arguments so --ros-args remappings and
// parameters (namespace, use_sim_time, ...) apply to nodes created here.
void init_default_context() {
  const auto arguments = read_process_arguments();
  std::vector<const char *> argv;
  argv.reserve(arguments.size());
  for (const auto &argument : arguments) {
    argv.push_back(argument.c_str());
  }
  try {
    rclcpp::init(static_cast<int>(argv.size()), argv.data());
  } catch (const std::exception &error) {
    RCLCPP_WARN(rclcpp::get_logger("yasmin_node"),
                "Ignoring process arguments for rclcpp::init: %s",
                error.what());
    rclcpp::init(0, nullptr);
  }
}

std::unique_ptr<rclcpp::Executor>
make_executor(const rclcpp::Context::SharedPtr &context) {
  if (context != rclcpp::contexts::get_global_default_context()) {
    // Its ExecutorOptions constructor is stable across distributions.
    rclcpp::ExecutorOptions options;
    options.context = context;
    return std::make_unique<rclcpp::executors::MultiThreadedExecutor>(options);
  }
#if __has_include("rclcpp/version.h")
#if RCLCPP_VERSION_GTE(29, 1, 1) // Jazzy, Kilted and Rolling
  return std::make_unique<rclcpp::experimental::executors::EventsExecutor>();
#else // Humble, Iron and Jazzy
  return std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
#endif
#else // Foxy and Galactic
  return std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
#endif
}
} // namespace

/**
 * @brief Generates a unique UUID as a string.
 *
 * This function uses random numbers to generate a 16-character hexadecimal
 * UUID.
 *
 * @return A string containing a 16-character hexadecimal UUID.
 */
inline std::string generateUUID() {
  static constexpr char hex_digits[] = "0123456789abcdef";
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 15);

  std::string result;
  result.reserve(16);
  for (int i = 0; i < 16; ++i) {
    result += hex_digits[dis(gen)];
  }
  return result;
}

YasminNode::SharedPtr YasminNode::get_instance() {
  return YasminNode::get_instance("", rclcpp::NodeOptions());
}

YasminNode::SharedPtr
YasminNode::get_instance(const std::string &node_name,
                         const rclcpp::NodeOptions &options) {
  std::lock_guard<std::mutex> lock(get_yasmin_node_instance_mutex());

  // After rclcpp::shutdown(), the node and its cached clients belong to the
  // old context. Release them first: rclcpp cannot re-initialize the default
  // context while nodes of the previous one are still alive.
  auto &instance = get_yasmin_node_instance();
  if (instance != nullptr &&
      !rclcpp::ok(instance->get_node_base_interface()->get_context())) {
    release_instance(instance);
  }

  const auto context = options.context();
  if (context == rclcpp::contexts::get_global_default_context()) {
    if (!rclcpp::ok()) {
      init_default_context();
    }
  } else if (!rclcpp::ok(context)) {
    throw std::invalid_argument(
        "YasminNode options select a context that is not initialized");
  }

  if (instance == nullptr) {
    instance = SharedPtr(new YasminNode(node_name, options));
  }

  return instance;
}

void YasminNode::destroy_instance() {
  std::lock_guard<std::mutex> lock(get_yasmin_node_instance_mutex());
  release_instance(get_yasmin_node_instance());
}

YasminNode::YasminNode() : YasminNode("", rclcpp::NodeOptions()) {}

YasminNode::YasminNode(const std::string &node_name,
                       const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name.empty() ? "yasmin_" + generateUUID() + "_node"
                                     : node_name,
                   options),
      executor(make_executor(options.context())) {
  // Add this node's base interface to the executor.
  this->executor->add_node(this->get_node_base_interface());

  // Initialize the spin thread to run the executor asynchronously.
  this->spin_thread = std::make_unique<std::thread>([this]() {
    try {
      this->executor->spin();
    } catch (const std::exception &error) {
      RCLCPP_ERROR(this->get_logger(), "YasminNode executor failed: %s",
                   error.what());
    }
    this->spin_finished.store(true);
  });
}

YasminNode::~YasminNode() {
  reset_logger_node(this);
  this->stop_executor();
}

void YasminNode::stop_executor() {
  if (this->spin_thread == nullptr) {
    return;
  }

  // Executor::cancel() is lost if spin() has not started yet: spin() then
  // marks itself as spinning and blocks. Repeat until spin() has returned.
  while (!this->spin_finished.load()) {
    this->executor->cancel();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  this->spin_thread->join();
  this->executor->remove_node(this->get_node_base_interface());

  this->spin_thread.reset();
}
