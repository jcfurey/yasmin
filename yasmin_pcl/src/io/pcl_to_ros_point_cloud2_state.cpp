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

#include "yasmin_pcl/io/pcl_to_ros_point_cloud2_state.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/common/cloud_validation.hpp"
#include "yasmin_pcl/common/ros_cloud_bridge.hpp"

namespace yasmin_pcl::io {

PclToRosPointCloud2State::PclToRosPointCloud2State()
    : yasmin::State({"succeeded", "aborted"}) {
  this->set_description(
      "Converts a pcl::PCLPointCloud2 cloud from the blackboard into a ROS "
      "sensor_msgs::msg::PointCloud2 message.");
  this->set_outcome_description("succeeded",
                                "The PCL point cloud was converted to ROS.");
  this->set_outcome_description("aborted",
                                "The input cloud was missing or invalid.");
  this->add_input_key("input_cloud",
                      "Input cloud stored as pcl::PCLPointCloud2::Ptr.");
  this->add_input_key(
      "input_header",
      "Optional exact ROS header (std_msgs::msg::Header) of the source cloud, "
      "e.g. RosToPclPointCloud2State 'output_header'. Restores the "
      "nanosecond stamp when the cloud still has that frame and time.");
  this->add_output_key("output_cloud",
                       "Converted cloud stored as "
                       "std::shared_ptr<sensor_msgs::msg::PointCloud2>, or as "
                       "serialized bytes with output_format 'serialized'.");
  this->declare_parameter<std::string>(
      "output_format",
      "'pointer' stores std::shared_ptr<PointCloud2> for C++ states; "
      "'serialized' stores serialized bytes for Python states.",
      "pointer");
}

void PclToRosPointCloud2State::configure() {
  const auto output_format = this->get_parameter<std::string>("output_format");
  if (output_format != "pointer" && output_format != "serialized") {
    throw std::invalid_argument("Unsupported output_format '" + output_format +
                                "'; expected 'pointer' or 'serialized'");
  }
  this->serialized_output_ = output_format == "serialized";
}

std::string
PclToRosPointCloud2State::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    const auto input_cloud =
        blackboard->get<common::PclPointCloud2Ptr>("input_cloud");

    if (!input_cloud) {
      YASMIN_LOG_WARN("Input PCL point cloud pointer is null");
      return "aborted";
    }
    common::validate_cloud(*input_cloud);

    auto output_cloud = common::make_ros_point_cloud2();
    pcl_conversions::fromPCL(*input_cloud, *output_cloud);

    if (blackboard->contains("input_header")) {
      const auto header =
          blackboard->get<std_msgs::msg::Header>("input_header");
      std::uint64_t pcl_stamp = 0;
      pcl_conversions::toPCL(header.stamp, pcl_stamp);
      // Only restore a header that still describes this cloud.
      if (header.frame_id == input_cloud->header.frame_id &&
          pcl_stamp == input_cloud->header.stamp) {
        output_cloud->header.stamp = header.stamp;
      } else {
        YASMIN_LOG_WARN("Ignoring 'input_header': its frame or time does not "
                        "match the cloud");
      }
    }

    if (this->serialized_output_) {
      blackboard->set<common::SerializedCloud>(
          "output_cloud",
          yasmin_ros::serialize_interface<sensor_msgs::msg::PointCloud2>(
              *output_cloud));
    } else {
      blackboard->set<common::RosPointCloud2Ptr>("output_cloud", output_cloud);
    }

    return "succeeded";
  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("Failed to convert PCL point cloud to ROS: %s", e.what());
    return "aborted";
  }
}

} // namespace yasmin_pcl::io

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::io::PclToRosPointCloud2State, yasmin::State)
