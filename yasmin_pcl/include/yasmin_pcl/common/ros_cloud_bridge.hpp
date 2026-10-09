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

#ifndef YASMIN_PCL__COMMON__ROS_CLOUD_BRIDGE_HPP_
#define YASMIN_PCL__COMMON__ROS_CLOUD_BRIDGE_HPP_

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "yasmin/blackboard.hpp"
#include "yasmin/demangle.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_ros/interface_serialization.hpp"

namespace yasmin_pcl::common {

/// @brief A CDR-serialized sensor_msgs/msg/PointCloud2 (Python bytes).
using SerializedCloud = std::vector<std::uint8_t>;

/**
 * @brief Read a ROS PointCloud2 stored in any supported representation.
 *
 * Accepted representations:
 * - std::shared_ptr<sensor_msgs::msg::PointCloud2>, as native C++ states
 *   store it (no copy);
 * - sensor_msgs::msg::PointCloud2 by value;
 * - std::vector<uint8_t> holding a serialized PointCloud2, which is how
 *   Python stores `rclpy.serialization.serialize_message(cloud)`.
 *
 * Python message objects cannot be read from C++ and are rejected with a
 * hint, as is any other type.
 *
 * @throws std::invalid_argument for unsupported types, or the serialization
 * error for malformed bytes.
 */
inline RosPointCloud2Ptr get_ros_cloud(const yasmin::Blackboard &blackboard,
                                       const std::string &key) {
  using sensor_msgs::msg::PointCloud2;
  const std::string type = blackboard.get_type(key);
  if (type == yasmin::demangle_type(typeid(RosPointCloud2Ptr).name())) {
    return blackboard.get<RosPointCloud2Ptr>(key);
  }
  if (type == yasmin::demangle_type(typeid(PointCloud2).name())) {
    return std::make_shared<PointCloud2>(blackboard.get<PointCloud2>(key));
  }
  if (type == yasmin::demangle_type(typeid(SerializedCloud).name())) {
    return std::make_shared<PointCloud2>(
        yasmin_ros::deserialize_interface<PointCloud2>(
            blackboard.get<SerializedCloud>(key)));
  }
  if (type == "pybind11::object") {
    throw std::invalid_argument(
        "Blackboard key '" + key +
        "' holds a Python object; C++ states read a PointCloud2 stored as "
        "rclpy.serialization.serialize_message(cloud)");
  }
  throw std::invalid_argument("Blackboard key '" + key + "' has type '" + type +
                              "', which is not a PointCloud2");
}

} // namespace yasmin_pcl::common

#endif // YASMIN_PCL__COMMON__ROS_CLOUD_BRIDGE_HPP_
