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

#ifndef YASMIN_PCL__TEST__TEST_UTILS_HPP_
#define YASMIN_PCL__TEST__TEST_UTILS_HPP_

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "yasmin_pcl/common/cloud_types.hpp"

namespace yasmin_pcl::test {

/** @brief Create a PCL XYZ point cloud from a vector of points.
 *  @param points The input points.
 *  @return The created point cloud. */
inline pcl::PointCloud<pcl::PointXYZ>
create_xyz_cloud(const std::vector<pcl::PointXYZ> &points) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.height = 1U;
  cloud.is_dense = true;
  cloud.points.assign(points.begin(), points.end());
  return cloud;
}

/** @brief Create a PCL PointCloud2 shared pointer from points.
 *  @param points The input points.
 *  @return Shared pointer to a PCL PointCloud2. */
inline common::PclPointCloud2Ptr
create_pcl_cloud_ptr(const std::vector<pcl::PointXYZ> &points) {
  auto cloud = common::make_pcl_point_cloud2();
  pcl::PointCloud<pcl::PointXYZ> xyz_cloud = create_xyz_cloud(points);
  pcl::toPCLPointCloud2(xyz_cloud, *cloud);
  return cloud;
}

/** @brief Create a ROS PointCloud2 shared pointer from points.
 *  @param points The input points.
 *  @return Shared pointer to a ROS PointCloud2. */
inline common::RosPointCloud2Ptr
create_ros_cloud_ptr(const std::vector<pcl::PointXYZ> &points) {
  const auto pcl_cloud = create_pcl_cloud_ptr(points);
  auto ros_cloud = common::make_ros_point_cloud2();
  pcl_conversions::fromPCL(*pcl_cloud, *ros_cloud);
  return ros_cloud;
}

/** @brief Convert a PCLPointCloud2 to a PCL XYZ point cloud.
 *  @param cloud The input PCLPointCloud2.
 *  @return The converted XYZ point cloud. */
inline pcl::PointCloud<pcl::PointXYZ>
to_xyz_cloud(const pcl::PCLPointCloud2 &cloud) {
  pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
  pcl::fromPCLPointCloud2(cloud, xyz_cloud);
  return xyz_cloud;
}

/** @brief RAII temporary file that removes itself on destruction. */
class TempFile {
public:
  TempFile() = default;

  explicit TempFile(std::filesystem::path path) : path_(std::move(path)) {}

  TempFile(const TempFile &) = delete;
  TempFile &operator=(const TempFile &) = delete;

  TempFile(TempFile &&other) noexcept : path_(std::exchange(other.path_, {})) {}

  TempFile &operator=(TempFile &&other) noexcept {
    if (this != &other) {
      reset();
      path_ = std::exchange(other.path_, {});
    }
    return *this;
  }

  ~TempFile() { reset(); }

  const std::filesystem::path &path() const { return path_; }

  void reset() {
    if (!path_.empty()) {
      std::error_code ec;
      std::filesystem::remove(path_, ec);
      path_.clear();
    }
  }

private:
  std::filesystem::path path_;
};

/** @brief Create a unique temporary file path using mkstemps.
 *  @param stem The filename stem (prefix).
 *  @param suffix The file extension with leading dot, e.g. ".ply".
 *  @return A TempFile whose path() gives the created path. */
inline TempFile make_temp_file(const std::string &stem,
                               const std::string &suffix) {
  const auto dir = std::filesystem::temp_directory_path();
  std::string tmpl = (dir / (stem + "_XXXXXX" + suffix)).string();
  const int suffix_len = static_cast<int>(suffix.size());
  const int fd = mkstemps(tmpl.data(), suffix_len);
  if (fd < 0) {
    throw std::runtime_error("mkstemps failed for " + tmpl);
  }
  close(fd);
  return TempFile(std::filesystem::path(tmpl));
}

/** @brief Organized cloud with x, y, z FLOAT32 and a trailing UINT8 'ring'.
 *
 *  Point i has x = i, y = 0, z = 0 and ring = i. Rows can be padded.
 *  @param width Points per row.
 *  @param height Number of rows.
 *  @param row_padding Extra bytes at the end of each row.
 *  @return Shared pointer to the cloud. */
inline common::PclPointCloud2Ptr
create_ring_cloud(std::uint32_t width, std::uint32_t height,
                  std::uint32_t row_padding = 0) {
  auto cloud = common::make_pcl_point_cloud2();
  cloud->width = width;
  cloud->height = height;
  cloud->point_step = 13;
  cloud->row_step = width * cloud->point_step + row_padding;
  const char *names[] = {"x", "y", "z"};
  for (std::uint32_t i = 0; i < 3; ++i) {
    pcl::PCLPointField field;
    field.name = names[i];
    field.offset = 4 * i;
    field.datatype = pcl::PCLPointField::FLOAT32;
    field.count = 1;
    cloud->fields.push_back(field);
  }
  pcl::PCLPointField ring;
  ring.name = "ring";
  ring.offset = 12;
  ring.datatype = pcl::PCLPointField::UINT8;
  ring.count = 1;
  cloud->fields.push_back(ring);
  cloud->data.assign(std::size_t(cloud->row_step) * height, 0xEE);
  for (std::uint32_t row = 0; row < height; ++row) {
    for (std::uint32_t col = 0; col < width; ++col) {
      const std::uint32_t index = row * width + col;
      const float xyz[3] = {float(index), 0.0F, 0.0F};
      auto *point = cloud->data.data() + std::size_t(row) * cloud->row_step +
                    std::size_t(col) * cloud->point_step;
      std::memcpy(point, xyz, sizeof(xyz));
      point[12] = static_cast<std::uint8_t>(index);
    }
  }
  return cloud;
}

} // namespace yasmin_pcl::test

#endif // YASMIN_PCL__TEST__TEST_UTILS_HPP_
