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

#include "yasmin_pcl/filters/voxel_grid_state.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_config.h>

#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/common/filter_state_utils.hpp"

namespace yasmin_pcl::filters {
namespace {

// Bounds as VoxelGrid computes them: PCL 1.15 bounds the selected points;
// older releases bound the whole cloud and lack the indices overloads.
void voxel_grid_bounds(const common::PclPointCloud2Ptr &cloud,
                       const pcl::IndicesPtr &indices, int x, int y, int z,
                       const std::string &field_name, float limit_min,
                       float limit_max, bool limit_negative,
                       Eigen::Vector4f &min_p, Eigen::Vector4f &max_p) {
#if PCL_VERSION_COMPARE(>=, 1, 15, 0)
  if (indices && field_name.empty()) {
    pcl::getMinMax3D(cloud, *indices, x, y, z, min_p, max_p);
    return;
  }
  if (indices) {
    pcl::getMinMax3D(cloud, *indices, x, y, z, field_name, limit_min, limit_max,
                     min_p, max_p, limit_negative);
    return;
  }
#else
  (void)indices;
#endif
  if (field_name.empty()) {
    pcl::getMinMax3D(cloud, x, y, z, min_p, max_p);
  } else {
    pcl::getMinMax3D(cloud, x, y, z, field_name, limit_min, limit_max, min_p,
                     max_p, limit_negative);
  }
}

// VoxelGrid only logs a warning and returns its input unfiltered when voxel
// indices would overflow, so mirror its check (same bounds, float inverse
// leaf size) and fail instead.
void require_voxel_index_range(const common::PclPointCloud2Ptr &cloud,
                               const pcl::IndicesPtr &indices,
                               const Eigen::Array3f &leaf,
                               const std::string &field_name, double limit_min,
                               double limit_max, bool limit_negative) {
  Eigen::Vector4f min_p, max_p;
  voxel_grid_bounds(
      cloud, indices, pcl::getFieldIndex(*cloud, "x"),
      pcl::getFieldIndex(*cloud, "y"), pcl::getFieldIndex(*cloud, "z"),
      field_name, static_cast<float>(limit_min), static_cast<float>(limit_max),
      limit_negative, min_p, max_p);
  double voxels = 1.0;
  for (int i = 0; i < 3; ++i) {
    if (!(min_p[i] <= max_p[i])) {
      return; // no finite point selected; PCL outputs an empty cloud
    }
    const float span = (max_p[i] - min_p[i]) * (1.0F / leaf[i]);
    voxels *= std::floor(static_cast<double>(span)) + 1.0;
  }
  if (voxels > static_cast<double>(std::numeric_limits<std::int32_t>::max())) {
    throw std::invalid_argument(
        "Voxel leaf size is too small for the cloud extent: voxel indices "
        "would overflow");
  }
}

} // namespace

VoxelGridState::VoxelGridState() : yasmin::State({"succeeded", "aborted"}) {
  this->leaf_size_x_ = 0.1F;
  this->leaf_size_y_ = 0.1F;
  this->leaf_size_z_ = 0.1F;
  this->downsample_all_data_ = true;
  this->minimum_points_number_per_voxel_ = 0;
  this->save_leaf_layout_ = false;
  this->filter_field_name_.clear();
  this->filter_limit_min_ = -std::numeric_limits<double>::infinity();
  this->filter_limit_max_ = std::numeric_limits<double>::infinity();
  this->filter_limit_negative_ = false;

  this->set_description(
      "Applies pcl::VoxelGrid to a pcl::PCLPointCloud2 stored in the "
      "blackboard.");
  this->set_outcome_description("succeeded",
                                "The cloud was downsampled successfully.");
  this->set_outcome_description(
      "aborted", "The input cloud was invalid or filtering failed.");

  this->declare_parameter<float>("leaf_size_x", "Voxel leaf size along x.",
                                 0.1F);
  this->declare_parameter<float>("leaf_size_y", "Voxel leaf size along y.",
                                 0.1F);
  this->declare_parameter<float>("leaf_size_z", "Voxel leaf size along z.",
                                 0.1F);
  this->declare_parameter<bool>(
      "downsample_all_data",
      "Downsample all fields instead of only the XYZ fields.", true);
  this->declare_parameter<int>(
      "minimum_points_number_per_voxel",
      "Minimum number of points required for a voxel to be kept.", 0);
  this->declare_parameter<bool>(
      "save_leaf_layout",
      "Store leaf layout information for later neighborhood queries.", false);
  this->declare_parameter<std::string>("filter_field_name",
                                       "Optional field name used together with "
                                       "filter limits before downsampling.",
                                       std::string(""));
  this->declare_parameter<double>("filter_limit_min",
                                  "Minimum field value used for pre-filtering.",
                                  -std::numeric_limits<double>::infinity());
  this->declare_parameter<double>("filter_limit_max",
                                  "Maximum field value used for pre-filtering.",
                                  std::numeric_limits<double>::infinity());
  this->declare_parameter<bool>(
      "filter_limit_negative",
      "Return points outside the specified field interval before downsampling.",
      false);

  this->add_input_key("input_cloud",
                      "Input cloud stored as pcl::PCLPointCloud2::Ptr.");
  this->add_input_key(
      "input_indices",
      "Optional subset of point indices stored as pcl::Indices.");
  this->add_output_key("output_cloud",
                       "Filtered cloud stored as pcl::PCLPointCloud2::Ptr.");
}

void VoxelGridState::configure() {
  this->leaf_size_x_ = this->get_parameter<float>("leaf_size_x");
  this->leaf_size_y_ = this->get_parameter<float>("leaf_size_y");
  this->leaf_size_z_ = this->get_parameter<float>("leaf_size_z");
  this->downsample_all_data_ = this->get_parameter<bool>("downsample_all_data");
  this->minimum_points_number_per_voxel_ =
      this->get_parameter<int>("minimum_points_number_per_voxel");
  this->save_leaf_layout_ = this->get_parameter<bool>("save_leaf_layout");
  this->filter_field_name_ =
      this->get_parameter<std::string>("filter_field_name");
  this->filter_limit_min_ = this->get_parameter<double>("filter_limit_min");
  this->filter_limit_max_ = this->get_parameter<double>("filter_limit_max");
  this->filter_limit_negative_ =
      this->get_parameter<bool>("filter_limit_negative");
}

std::string VoxelGridState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    const auto input_cloud = common::prepare_filter_cloud(
        blackboard->get<common::PclPointCloud2Ptr>("input_cloud"));

    if (!input_cloud) {
      YASMIN_LOG_WARN("Input PCL point cloud pointer is null");
      return "aborted";
    }

    if (this->minimum_points_number_per_voxel_ < 0) {
      YASMIN_LOG_WARN(
          "Parameter 'minimum_points_number_per_voxel' must not be negative");
      return "aborted";
    }

    for (const float leaf :
         {this->leaf_size_x_, this->leaf_size_y_, this->leaf_size_z_}) {
      if (!std::isfinite(leaf) || leaf <= 0 || !std::isfinite(1.0F / leaf)) {
        throw std::invalid_argument(
            "Voxel leaf sizes must be finite and positive");
      }
    }
    if (!this->filter_field_name_.empty()) {
      common::require_float_field(*input_cloud, this->filter_field_name_);
      if (std::isnan(this->filter_limit_min_) ||
          std::isnan(this->filter_limit_max_) ||
          this->filter_limit_min_ > this->filter_limit_max_) {
        throw std::invalid_argument("Invalid VoxelGrid limits");
      }
    }

    pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
    filter.setInputCloud(input_cloud);
    filter.setLeafSize(this->leaf_size_x_, this->leaf_size_y_,
                       this->leaf_size_z_);
    filter.setDownsampleAllData(this->downsample_all_data_);
    filter.setMinimumPointsNumberPerVoxel(
        static_cast<unsigned int>(this->minimum_points_number_per_voxel_));
    filter.setSaveLeafLayout(this->save_leaf_layout_);

    if (!this->filter_field_name_.empty()) {
      filter.setFilterFieldName(this->filter_field_name_);
      filter.setFilterLimits(this->filter_limit_min_, this->filter_limit_max_);
      filter.setFilterLimitsNegative(this->filter_limit_negative_);
    }

    common::set_optional_input_indices(filter, blackboard);
    require_voxel_index_range(
        input_cloud, filter.getIndices(),
        Eigen::Array3f(this->leaf_size_x_, this->leaf_size_y_,
                       this->leaf_size_z_),
        this->filter_field_name_, this->filter_limit_min_,
        this->filter_limit_max_, this->filter_limit_negative_);

    auto output_cloud = common::make_pcl_point_cloud2();
    filter.filter(*output_cloud);
    blackboard->set<common::PclPointCloud2Ptr>("output_cloud", output_cloud);

    return "succeeded";
  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("VoxelGrid filtering failed: %s", e.what());
    return "aborted";
  }
}

} // namespace yasmin_pcl::filters

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::filters::VoxelGridState, yasmin::State)
