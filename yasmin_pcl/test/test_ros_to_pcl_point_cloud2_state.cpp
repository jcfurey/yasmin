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
#include "yasmin_pcl/io/pcl_to_ros_point_cloud2_state.hpp"
#include "yasmin_pcl/io/ros_to_pcl_point_cloud2_state.hpp"

TEST(RosToPclPointCloud2State, ConvertsRosCloudToPclCloud) {
  yasmin_pcl::io::RosToPclPointCloud2State state;
  auto blackboard = yasmin::Blackboard::make_shared();

  blackboard->set<yasmin_pcl::common::RosPointCloud2Ptr>(
      "input_cloud", yasmin_pcl::test::create_ros_cloud_ptr(
                         {{0.0F, 0.0F, 0.0F}, {1.0F, 2.0F, 3.0F}}));

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(*output_cloud);
  ASSERT_EQ(xyz_cloud.points.size(), 2U);
  EXPECT_FLOAT_EQ(xyz_cloud.points[1].x, 1.0F);
  EXPECT_FLOAT_EQ(xyz_cloud.points[1].y, 2.0F);
  EXPECT_FLOAT_EQ(xyz_cloud.points[1].z, 3.0F);
}

TEST(RosToPclPointCloud2State, AbortsWhenInputCloudIsNull) {
  yasmin_pcl::io::RosToPclPointCloud2State state;
  auto blackboard = yasmin::Blackboard::make_shared();

  blackboard->set<yasmin_pcl::common::RosPointCloud2Ptr>("input_cloud",
                                                         nullptr);

  EXPECT_EQ(state(blackboard), "aborted");
}

TEST(RosToPclPointCloud2State, RejectsInvalidLayouts) {
  for (int mutation = 0; mutation < 5; ++mutation) {
    auto cloud = yasmin_pcl::test::create_ros_cloud_ptr({{1, 2, 3}});
    switch (mutation) {
    case 0:
      cloud->data.clear();
      break;
    case 1:
      cloud->row_step = cloud->point_step - 1;
      break;
    case 2:
      cloud->fields[0].offset = cloud->point_step;
      break;
    case 3:
      cloud->fields[0].count = 0;
      break;
    case 4:
      cloud->fields[0].datatype = 255;
      break;
    }
    auto blackboard = yasmin::Blackboard::make_shared();
    blackboard->set<yasmin_pcl::common::RosPointCloud2Ptr>("input_cloud",
                                                           cloud);
    yasmin_pcl::io::RosToPclPointCloud2State state;
    EXPECT_EQ(state(blackboard), "aborted") << mutation;
    EXPECT_FALSE(blackboard->contains("output_cloud"));
  }
}

TEST(RosToPclPointCloud2State, RoundTripPreservesFieldsPaddingAndFrame) {
  auto cloud = yasmin_pcl::test::create_ros_cloud_ptr({{1, 2, 3}, {4, 5, 6}});
  sensor_msgs::msg::PointField intensity;
  intensity.name = "intensity";
  intensity.offset = 12;
  intensity.count = 1;
  intensity.datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud->fields.push_back(intensity);
  const float value = 42.5F;
  std::memcpy(cloud->data.data() + intensity.offset, &value, sizeof(value));
  const auto packed = cloud->data;
  cloud->width = 1;
  cloud->height = 2;
  cloud->row_step = cloud->point_step + 8;
  cloud->data.assign(cloud->row_step * cloud->height, 0xee);
  for (std::size_t row = 0; row < 2; ++row) {
    std::memcpy(cloud->data.data() + row * cloud->row_step,
                packed.data() + row * cloud->point_step, cloud->point_step);
  }
  cloud->header.frame_id = "lidar";
  cloud->header.stamp.sec = 10;
  cloud->header.stamp.nanosec = 123456000; // PCL stores microseconds.
  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::RosPointCloud2Ptr>("input_cloud", cloud);
  yasmin_pcl::io::RosToPclPointCloud2State to_pcl;
  ASSERT_EQ(to_pcl(blackboard), "succeeded");
  blackboard->copy_value_from(*blackboard, "output_cloud", "input_cloud");
  yasmin_pcl::io::PclToRosPointCloud2State to_ros;
  ASSERT_EQ(to_ros(blackboard), "succeeded");
  const auto output =
      blackboard->get<yasmin_pcl::common::RosPointCloud2Ptr>("output_cloud");
  EXPECT_EQ(output->data, cloud->data);
  EXPECT_EQ(output->fields, cloud->fields);
  EXPECT_EQ(output->width, cloud->width);
  EXPECT_EQ(output->height, cloud->height);
  EXPECT_EQ(output->row_step, cloud->row_step);
  EXPECT_EQ(output->is_bigendian, cloud->is_bigendian);
  EXPECT_EQ(output->header, cloud->header);
}
