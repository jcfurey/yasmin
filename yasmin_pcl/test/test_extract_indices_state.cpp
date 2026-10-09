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

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>

#include "test_utils.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/filters/extract_indices_state.hpp"

TEST(ExtractIndicesState, ExtractsRequestedIndices) {
  yasmin_pcl::filters::ExtractIndicesState state;
  state.set_parameter<bool>("extract_removed_indices", true);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud",
      yasmin_pcl::test::create_pcl_cloud_ptr(
          {{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {2.0F, 0.0F, 0.0F}}));
  blackboard->set<yasmin_pcl::common::Indices>("input_indices", {0, 2});

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(*output_cloud);
  ASSERT_EQ(xyz_cloud.points.size(), 2U);
  EXPECT_FLOAT_EQ(xyz_cloud.points[0].x, 0.0F);
  EXPECT_FLOAT_EQ(xyz_cloud.points[1].x, 2.0F);

  const auto output_indices =
      blackboard->get<yasmin_pcl::common::Indices>("output_indices");
  EXPECT_EQ(output_indices.size(), 2U);

  const auto removed_indices =
      blackboard->get<yasmin_pcl::common::Indices>("removed_indices");
  EXPECT_EQ(removed_indices.size(), 1U);
}

TEST(ExtractIndicesState, ReturnsComplementWhenNegativeEnabled) {
  yasmin_pcl::filters::ExtractIndicesState state;
  state.set_parameter<bool>("negative", true);
  state.set_parameter<bool>("extract_removed_indices", true);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud",
      yasmin_pcl::test::create_pcl_cloud_ptr(
          {{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {2.0F, 0.0F, 0.0F}}));
  blackboard->set<yasmin_pcl::common::Indices>("input_indices", {0, 2});

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(*output_cloud);
  ASSERT_EQ(xyz_cloud.points.size(), 1U);
  EXPECT_FLOAT_EQ(xyz_cloud.points[0].x, 1.0F);

  const auto output_indices =
      blackboard->get<yasmin_pcl::common::Indices>("output_indices");
  ASSERT_EQ(output_indices.size(), 1U);
  EXPECT_EQ(output_indices[0], 1);

  const auto removed_indices =
      blackboard->get<yasmin_pcl::common::Indices>("removed_indices");
  ASSERT_EQ(removed_indices.size(), 2U);
  EXPECT_EQ(removed_indices[0], 0);
  EXPECT_EQ(removed_indices[1], 2);
}

TEST(ExtractIndicesState, AbortsWhenInputIndicesAreMissing) {
  yasmin_pcl::filters::ExtractIndicesState state;
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud", yasmin_pcl::test::create_pcl_cloud_ptr(
                         {{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}}));

  EXPECT_EQ(state(blackboard), "aborted");
}

TEST(ExtractIndicesState, KeepOrganizedOnlyMasksRemovedFloatFields) {
  // PCL writes a float at every field offset of removed points, overrunning
  // the 1-byte 'ring' into the next point and past the buffer at the end.
  const auto input = yasmin_pcl::test::create_ring_cloud(4, 2);
  yasmin_pcl::filters::ExtractIndicesState state;
  state.set_parameter<bool>("negative", true);
  state.set_parameter<bool>("keep_organized", true);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>("input_cloud", input);
  blackboard->set<yasmin_pcl::common::Indices>("input_indices", {2, 7});
  ASSERT_EQ(state(blackboard), "succeeded");

  const auto output =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_EQ(output->data.size(), input->data.size());
  EXPECT_FALSE(output->is_dense);
  for (int point = 0; point < 8; ++point) {
    const auto *in = input->data.data() + point * 13;
    const auto *out = output->data.data() + point * 13;
    float x;
    std::memcpy(&x, out, sizeof(x));
    EXPECT_EQ(out[12], in[12]) << "ring of point " << point;
    if (point == 2 || point == 7) {
      EXPECT_TRUE(std::isnan(x)) << point;
    } else {
      EXPECT_EQ(std::memcmp(in, out, 13), 0) << "kept point " << point;
    }
  }
}
