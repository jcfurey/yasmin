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
#include <stdexcept>
#include <string>
#include <gtest/gtest.h>

#include "test_utils.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/common/ros_cloud_bridge.hpp"
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

namespace {

std::size_t point_count(const yasmin_pcl::common::PclPointCloud2Ptr &cloud) {
  return std::size_t(cloud->width) * cloud->height;
}

} // namespace

TEST(RosToPclPointCloud2State, AcceptsSerializedAndByValueClouds) {
  const auto cloud = yasmin_pcl::test::create_ros_cloud_ptr({{1, 2, 3}, {4, 5, 6}});
  for (int representation = 0; representation < 2; ++representation) {
    auto blackboard = yasmin::Blackboard::make_shared();
    if (representation == 0) {
      // How Python stores rclpy.serialization.serialize_message(cloud).
      blackboard->set<yasmin_pcl::common::SerializedCloud>(
          "input_cloud",
          yasmin_ros::serialize_interface<sensor_msgs::msg::PointCloud2>(*cloud));
    } else {
      blackboard->set<sensor_msgs::msg::PointCloud2>("input_cloud", *cloud);
    }
    yasmin_pcl::io::RosToPclPointCloud2State state;
    ASSERT_EQ(state(blackboard), "succeeded") << representation;
    EXPECT_EQ(point_count(blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>(
                  "output_cloud")),
              2U);
  }
}

TEST(RosToPclPointCloud2State, RejectsUnsupportedAndMalformedInputs) {
  auto bytes = yasmin_ros::serialize_interface<sensor_msgs::msg::PointCloud2>(
      *yasmin_pcl::test::create_ros_cloud_ptr({{1, 2, 3}}));
  bytes.resize(bytes.size() / 2);
  for (int mutation = 0; mutation < 2; ++mutation) {
    auto blackboard = yasmin::Blackboard::make_shared();
    if (mutation == 0) {
      blackboard->set<std::string>("input_cloud", "not a cloud");
    } else {
      blackboard->set<yasmin_pcl::common::SerializedCloud>("input_cloud", bytes);
    }
    yasmin_pcl::io::RosToPclPointCloud2State state;
    EXPECT_EQ(state(blackboard), "aborted") << mutation;
    EXPECT_FALSE(blackboard->contains("output_cloud"));
  }
}

TEST(PclToRosPointCloud2State, RestoresExactStampFromMatchingHeader) {
  auto cloud = yasmin_pcl::test::create_ros_cloud_ptr({{1, 2, 3}});
  cloud->header.frame_id = "lidar";
  cloud->header.stamp.sec = 1;
  cloud->header.stamp.nanosec = 123456789;
  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::RosPointCloud2Ptr>("input_cloud", cloud);
  yasmin_pcl::io::RosToPclPointCloud2State to_pcl;
  ASSERT_EQ(to_pcl(blackboard), "succeeded");

  // Pipeline wiring: PCL cloud and exact header feed the reverse conversion.
  auto back = yasmin::Blackboard::make_shared();
  back->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud",
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud"));
  yasmin_pcl::io::PclToRosPointCloud2State to_ros;
  ASSERT_EQ(to_ros(back), "succeeded");
  EXPECT_NE(back->get<yasmin_pcl::common::RosPointCloud2Ptr>("output_cloud")
                ->header.stamp.nanosec,
            123456789U);

  back->set<std_msgs::msg::Header>(
      "input_header", blackboard->get<std_msgs::msg::Header>("output_header"));
  ASSERT_EQ(to_ros(back), "succeeded");
  const auto restored =
      back->get<yasmin_pcl::common::RosPointCloud2Ptr>("output_cloud");
  EXPECT_EQ(restored->header.stamp.sec, 1);
  EXPECT_EQ(restored->header.stamp.nanosec, 123456789U);
  EXPECT_EQ(restored->header.frame_id, "lidar");

  // A header that no longer describes the cloud is not applied.
  auto other = blackboard->get<std_msgs::msg::Header>("output_header");
  other.frame_id = "map";
  back->set<std_msgs::msg::Header>("input_header", other);
  ASSERT_EQ(to_ros(back), "succeeded");
  EXPECT_NE(back->get<yasmin_pcl::common::RosPointCloud2Ptr>("output_cloud")
                ->header.stamp.nanosec,
            123456789U);
}

TEST(PclToRosPointCloud2State, SerializedOutputRoundTripsForPython) {
  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud", yasmin_pcl::test::create_pcl_cloud_ptr({{1, 2, 3}}));
  yasmin_pcl::io::PclToRosPointCloud2State state;
  state.set_parameter<std::string>("output_format", "serialized");
  state.configure();
  ASSERT_EQ(state(blackboard), "succeeded");

  const auto cloud =
      yasmin_ros::deserialize_interface<sensor_msgs::msg::PointCloud2>(
          blackboard->get<yasmin_pcl::common::SerializedCloud>("output_cloud"));
  EXPECT_EQ(cloud.width * cloud.height, 1U);

  state.set_parameter<std::string>("output_format", "python");
  EXPECT_THROW(state.configure(), std::invalid_argument);
}
