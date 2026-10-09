// Copyright (C) 2026
// Licensed under the Apache License, Version 2.0.

#ifndef YASMIN_PCL__COMMON__CLOUD_VALIDATION_HPP_
#define YASMIN_PCL__COMMON__CLOUD_VALIDATION_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "yasmin_pcl/common/cloud_types.hpp"

namespace yasmin_pcl::common {

// ROS and PCL use the same PointField datatype IDs.
inline std::uint64_t field_size(std::uint8_t datatype) {
  switch (datatype) {
  case 1:
  case 2:
    return 1;
  case 3:
  case 4:
    return 2;
  case 5:
  case 6:
  case 7:
    return 4;
  case 8:
  case 9:
  case 10:
    return 8;
  default:
    throw std::invalid_argument("Unsupported point field datatype");
  }
}

template <class Cloud> void validate_cloud(const Cloud &cloud) {
  const auto packed_row = std::uint64_t(cloud.width) * cloud.point_step;
  const auto size = std::uint64_t(cloud.row_step) * cloud.height;
  if (packed_row > cloud.row_step || size != cloud.data.size() ||
      (cloud.width != 0 && cloud.height != 0 && cloud.point_step == 0)) {
    throw std::invalid_argument(
        "Point cloud dimensions/strides do not match data");
  }
  std::unordered_set<std::string> names;
  for (const auto &field : cloud.fields) {
    if (field.count == 0 ||
        std::uint64_t(field.offset) + field_size(field.datatype) * field.count >
            cloud.point_step ||
        (field.name != "_" && !names.insert(field.name).second)) {
      throw std::invalid_argument("Invalid point field '" + field.name + "'");
    }
  }
  if (cloud.width != 0 && cloud.height != 0 && cloud.fields.empty()) {
    throw std::invalid_argument("Nonempty point cloud has no fields");
  }
}

template <class Cloud>
void require_float_field(const Cloud &cloud, const std::string &name) {
  const auto field =
      std::find_if(cloud.fields.begin(), cloud.fields.end(),
                   [&](const auto &f) { return f.name == name; });
  if (field == cloud.fields.end() || field->datatype != 7 ||
      field->count != 1) {
    throw std::invalid_argument("PCL requires a scalar FLOAT32 field '" + name +
                                "'");
  }
}

// PCL's binary filters use flat point indices and native-endian float loads.
// Pack organized rows without changing the caller's cloud. Reject foreign
// byte order explicitly instead of interpreting those bytes as native floats.
inline PclPointCloud2Ptr prepare_filter_cloud(const PclPointCloud2Ptr &cloud) {
  if (!cloud) {
    throw std::invalid_argument("Input PCL point cloud pointer is null");
  }
  validate_cloud(*cloud);
  const std::uint16_t endian_probe = 1;
  const bool native_bigendian =
      *reinterpret_cast<const unsigned char *>(&endian_probe) == 0;
  if (bool(cloud->is_bigendian) != native_bigendian) {
    throw std::invalid_argument("PCL filters require native-endian point data");
  }
  if (std::uint64_t(cloud->width) * cloud->height >
      std::uint64_t(std::numeric_limits<int>::max())) {
    throw std::invalid_argument("Point cloud exceeds PCL's index range");
  }
  for (const auto *name : {"x", "y", "z"}) {
    require_float_field(*cloud, name);
  }
  const auto row_size = cloud->width * cloud->point_step;
  if (cloud->row_step == row_size) {
    return cloud;
  }
  auto packed = make_pcl_point_cloud2();
  *packed = *cloud;
  packed->row_step = row_size;
  packed->data.resize(std::size_t(row_size) * cloud->height);
  for (std::uint32_t row = 0; row < cloud->height; ++row) {
    if (row_size != 0) {
      std::memcpy(packed->data.data() + std::size_t(row) * row_size,
                  cloud->data.data() + std::size_t(row) * cloud->row_step,
                  row_size);
    }
  }
  return packed;
}

} // namespace yasmin_pcl::common

#endif // YASMIN_PCL__COMMON__CLOUD_VALIDATION_HPP_
