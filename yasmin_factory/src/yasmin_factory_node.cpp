// Copyright (C) 2025 Miguel Ángel González Santamarta
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

#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "yasmin/state_machine.hpp"
#include "yasmin/state_machine_cancel_exception.hpp"
#include "yasmin_factory/yasmin_factory.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

int main(int argc, char *argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Name the node before anything else creates it with a random name
  auto node = yasmin_ros::YasminNode::get_instance("yasmin_factory_node");

  // Set up ROS 2 loggers
  yasmin_ros::set_ros_loggers(node);
  YASMIN_LOG_INFO("yasmin_factory_node");

  // Get the state machine file parameter
  node->declare_parameter("state_machine_file", "");
  std::string sm_file = node->get_parameter("state_machine_file")
                            .get_parameter_value()
                            .get<std::string>();

  if (sm_file.empty()) {
    throw std::runtime_error("state_machine_file parameter must be non-empty");
  }

  // Get if enable viewer parameter
  node->declare_parameter("enable_viewer_pub", true);
  bool enable_viewer_pub = node->get_parameter("enable_viewer_pub")
                               .get_parameter_value()
                               .get<bool>();

  // Only build and validate the state machine (used by `ros2 yasmin validate`)
  node->declare_parameter("validate_only", false);
  node->declare_parameter("strict_validation", true);
  const bool validate_only = node->get_parameter("validate_only").as_bool();
  const bool strict_validation =
      node->get_parameter("strict_validation").as_bool();

  // 0: the state machine ended with an outcome; 1: it could not be created,
  // validated or run; 130: it was canceled (e.g. by Ctrl-C).
  int exit_code = 0;

  // Create the state machine, viewer publisher, and run everything inside a
  // scope so that every ROS- and Python-touching object is destroyed while
  // the rclcpp context and DDS participant are still valid.
  {
    // Create the factory in a scope
    yasmin_factory::YasminFactory factory;

    // Create the state machine from the XML file
    yasmin::StateMachine::SharedPtr sm;
    try {
      sm = factory.create_sm_from_file(sm_file);
      if (validate_only) {
        sm->validate(strict_validation);
      }
    } catch (const std::exception &e) {
      std::cerr << "Invalid state machine '" << sm_file << "': " << e.what()
                << std::endl;
      exit_code = 1;
    }

    std::unique_ptr<yasmin_viewer::YasminViewerPub> yasmin_pub_ptr;
    if (sm && !validate_only) {
      sm->set_sigint_handler(true);

      // Cancel when ROS shuts down (SIGTERM, a second SIGINT, or
      // rclcpp::shutdown()); pre-shutdown, so remote goals can still be
      // canceled.
      auto context = node->get_node_base_interface()->get_context();
      std::weak_ptr<yasmin::StateMachine> weak_sm = sm;
      auto shutdown_handle = context->add_pre_shutdown_callback([weak_sm]() {
        if (auto running_sm = weak_sm.lock()) {
          running_sm->cancel_state_machine();
        }
      });

      // Publisher for visualizing the state machine
      if (enable_viewer_pub) {
        yasmin_pub_ptr = std::make_unique<yasmin_viewer::YasminViewerPub>(sm);
      }

      // Execute the state machine
      try {
        std::string outcome = (*sm.get())();
        YASMIN_LOG_INFO(outcome.c_str());
      } catch (const yasmin::StateMachineCancelException &e) {
        YASMIN_LOG_WARN("State machine canceled");
        exit_code = 130;
      } catch (const std::exception &e) {
        YASMIN_LOG_ERROR("State machine execution failed: %s", e.what());
        exit_code = 1;
      }
      context->remove_pre_shutdown_callback(shutdown_handle);
    }

    // Tear down in strict order while the context is still alive: stop the
    // viewer (timer + publisher), then the SM (releases the Python states),
    // then the factory.
    yasmin_pub_ptr.reset();
    sm.reset();
  }

  // Destroy the Python-side YasminNode singleton as well: Python states
  // (e.g. GetParametersState) create it lazily, and it holds an rclpy node
  // with a parameter_event publisher plus a spin thread. Its only other
  // owner is an atexit handler, which would run after rclcpp::shutdown()
  // and try to finalize publishers on a dead DDS participant.
  try {
    pybind11::gil_scoped_acquire acquire;
#if PYBIND11_VERSION_MAJOR > 2 ||                                              \
    (PYBIND11_VERSION_MAJOR == 2 && PYBIND11_VERSION_MINOR >= 6)
    pybind11::module_::import("yasmin_ros.yasmin_node")
        .attr("YasminNode")
        .attr("destroy_instance")();
#else
    pybind11::module::import("yasmin_ros.yasmin_node")
        .attr("YasminNode")
        .attr("destroy_instance")();
#endif
  } catch (const pybind11::error_already_set &e) {
    RCLCPP_WARN(rclcpp::get_logger("yasmin_factory_node"),
                "Failed to destroy Python YasminNode: %s", e.what());
  }

  // Now nothing else holds the node, so this really destroys it and joins
  // the executor spin thread.
  yasmin_ros::YasminNode::destroy_instance();

  // Shutdown ROS 2
  rclcpp::shutdown();
  return exit_code;
}
