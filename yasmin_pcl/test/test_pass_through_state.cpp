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

#include <cstring>
#include <gtest/gtest.h>

#include "test_utils.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/filters/pass_through_state.hpp"

TEST(PassThroughState, FiltersCloudAndStoresRemovedIndices) {
  yasmin_pcl::filters::PassThroughState state;
  state.set_parameter<std::string>("filter_field_name", "z");
  state.set_parameter<double>("filter_limit_min", 0.5);
  state.set_parameter<double>("filter_limit_max", 1.5);
  state.set_parameter<bool>("extract_removed_indices", true);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud",
      yasmin_pcl::test::create_pcl_cloud_ptr(
          {{0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 2.0F}}));

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(*output_cloud);
  ASSERT_EQ(xyz_cloud.points.size(), 1U);
  EXPECT_FLOAT_EQ(xyz_cloud.points[0].z, 1.0F);

  const auto removed_indices =
      blackboard->get<yasmin_pcl::common::Indices>("removed_indices");
  EXPECT_EQ(removed_indices.size(), 2U);
}

TEST(PassThroughState, RejectsMissingFieldAndMistypedIndices) {
  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud", yasmin_pcl::test::create_pcl_cloud_ptr({{1, 2, 3}}));
  yasmin_pcl::filters::PassThroughState state;
  state.set_parameter<std::string>("filter_field_name", "missing");
  state.configure();
  EXPECT_EQ(state(blackboard), "aborted");
  state.set_parameter<std::string>("filter_field_name", "z");
  state.configure();
  blackboard->set<std::string>("input_indices", "wrong type");
  EXPECT_EQ(state(blackboard), "aborted");
}

TEST(PassThroughState, PacksPaddedRowsAndPreservesInput) {
  auto cloud = yasmin_pcl::test::create_pcl_cloud_ptr({{1, 2, 1}, {4, 5, 3}});
  const auto original = cloud->data;
  cloud->width = 1;
  cloud->height = 2;
  cloud->row_step = cloud->point_step + 8;
  cloud->data.assign(cloud->row_step * cloud->height, 0xff);
  for (std::size_t row = 0; row < 2; ++row) {
    std::memcpy(cloud->data.data() + row * cloud->row_step,
                original.data() + row * cloud->point_step, cloud->point_step);
  }
  const auto padded = cloud->data;
  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>("input_cloud", cloud);
  yasmin_pcl::filters::PassThroughState state;
  state.set_parameter<std::string>("filter_field_name", "z");
  state.set_parameter<double>("filter_limit_min", 2);
  state.set_parameter<double>("filter_limit_max", 4);
  state.configure();
  EXPECT_EQ(state(blackboard), "succeeded");
  auto output =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  const auto xyz = yasmin_pcl::test::to_xyz_cloud(*output);
  ASSERT_EQ(xyz.size(), 1U);
  EXPECT_FLOAT_EQ(xyz[0].x, 4);
  EXPECT_FLOAT_EQ(xyz[0].z, 3);
  EXPECT_EQ(cloud->data, padded);
}

TEST(PassThroughState, RejectsForeignEndianAndTruncatedClouds) {
  for (bool foreign_endian : {false, true}) {
    auto cloud = yasmin_pcl::test::create_pcl_cloud_ptr({{1, 2, 3}});
    if (foreign_endian) {
      cloud->is_bigendian = !cloud->is_bigendian;
    } else {
      cloud->data.clear();
    }
    auto blackboard = yasmin::Blackboard::make_shared();
    blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>("input_cloud",
                                                           cloud);
    yasmin_pcl::filters::PassThroughState state;
    EXPECT_EQ(state(blackboard), "aborted");
  }
}

TEST(PassThroughState, ReturnsOutsideIntervalWhenNegativeEnabled) {
  yasmin_pcl::filters::PassThroughState state;
  state.set_parameter<std::string>("filter_field_name", "z");
  state.set_parameter<double>("filter_limit_min", 0.5);
  state.set_parameter<double>("filter_limit_max", 1.5);
  state.set_parameter<bool>("filter_limit_negative", true);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud",
      yasmin_pcl::test::create_pcl_cloud_ptr(
          {{0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 2.0F}}));

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(*output_cloud);
  ASSERT_EQ(xyz_cloud.points.size(), 2U);
  EXPECT_FLOAT_EQ(xyz_cloud.points[0].z, 0.0F);
  EXPECT_FLOAT_EQ(xyz_cloud.points[1].z, 2.0F);
}

TEST(PassThroughState, FiltersOnlyProvidedInputIndices) {
  yasmin_pcl::filters::PassThroughState state;
  state.set_parameter<std::string>("filter_field_name", "z");
  state.set_parameter<double>("filter_limit_min", 1.5);
  state.set_parameter<double>("filter_limit_max", 3.0);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud",
      yasmin_pcl::test::create_pcl_cloud_ptr(
          {{0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 2.0F}}));
  blackboard->set<yasmin_pcl::common::Indices>("input_indices", {1, 2});

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(*output_cloud);
  ASSERT_EQ(xyz_cloud.points.size(), 1U);
  EXPECT_FLOAT_EQ(xyz_cloud.points[0].z, 2.0F);
}
