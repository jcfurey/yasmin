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

#ifndef YASMIN_PCL__COMMON__QOS_HPP_
#define YASMIN_PCL__COMMON__QOS_HPP_

#include <stdexcept>
#include <string>

#include <rclcpp/qos.hpp>

namespace yasmin_pcl::common {

/**
 * @brief Build a QoS profile from a parameter value.
 *
 * `sensor_data` is best effort (rclcpp::SensorDataQoS); `reliable` is
 * compatible with both reliable and best-effort subscribers;
 * `system_default` uses the middleware defaults.
 *
 * @param profile Profile name.
 * @param depth History depth, at least 1.
 */
inline rclcpp::QoS make_qos(const std::string &profile, int depth) {
  if (depth < 1) {
    throw std::invalid_argument("queue_size must be at least 1");
  }
  if (profile == "sensor_data") {
    return rclcpp::SensorDataQoS().keep_last(depth);
  }
  if (profile == "reliable") {
    return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable();
  }
  if (profile == "system_default") {
    return rclcpp::SystemDefaultsQoS().keep_last(depth);
  }
  throw std::invalid_argument("Unsupported qos '" + profile +
                              "'; expected 'sensor_data', 'reliable' or "
                              "'system_default'");
}

} // namespace yasmin_pcl::common

#endif // YASMIN_PCL__COMMON__QOS_HPP_
