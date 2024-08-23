// Copyright 2024 Husarion sp. z o.o.
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
#include <chrono>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "panther_utils/tf2_utils.hpp"

class TestTransformPose : public ::testing::Test
{
public:
  TestTransformPose();
  ~TestTransformPose();

  void SetBaseLinkToOdomTransform(const builtin_interfaces::msg::Time & stamp);

protected:
  rclcpp::Clock::SharedPtr clock_;
  tf2_ros::Buffer::SharedPtr tf2_buffer_;
};

void TestTransformPose::SetBaseLinkToOdomTransform(const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 0.3;
  transform.transform.translation.y = 0.2;
  transform.transform.translation.z = 0.1;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  tf2_buffer_->setTransform(transform, "unittest", false);
}

TestTransformPose::TestTransformPose()
{
  rclcpp::init(0, nullptr);

  clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);

  // Silence error about dedicated thread's being necessary
  tf2_buffer_->setUsingDedicatedThread(true);
}

TestTransformPose::~TestTransformPose() { rclcpp::shutdown(); }

TEST_F(TestTransformPose, NoTf)
{
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_THROW(
    { panther_utils::tf2_utils::TransformPose(tf2_buffer_, pose, ""); }, std::runtime_error);
  EXPECT_THROW(
    { panther_utils::tf2_utils::TransformPose(tf2_buffer_, pose, "base_link"); },
    std::runtime_error);
  pose.header.frame_id = "odom";
  EXPECT_THROW(
    { panther_utils::tf2_utils::TransformPose(tf2_buffer_, pose, "base_link"); },
    std::runtime_error);
}

TEST_F(TestTransformPose, FrameToFrameTransform)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom";
  pose.pose.position.x = 0.1;
  pose.header.stamp = clock_->now();

  ASSERT_NO_THROW(
    { pose = panther_utils::tf2_utils::TransformPose(tf2_buffer_, pose, "odom", 10.0); };);
  EXPECT_NEAR(pose.pose.position.x, 0.1, 0.01);
  EXPECT_NEAR(pose.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(pose.pose.position.z, 0.0, 0.01);
  EXPECT_EQ(pose.header.frame_id, "odom");
  EXPECT_NEAR(tf2::getYaw(pose.pose.orientation), 0.0, 0.01);
}

TEST_F(TestTransformPose, FrameToOtherFrameTransform)
{
  geometry_msgs::msg::PoseStamped pose;
  const auto stamp = clock_->now();
  pose.header.frame_id = "odom";
  pose.pose.position.x = 0.1;
  pose.header.stamp = stamp;
  SetBaseLinkToOdomTransform(stamp);

  ASSERT_NO_THROW(
    { pose = panther_utils::tf2_utils::TransformPose(tf2_buffer_, pose, "base_link", 10.0); };);
  EXPECT_NEAR(pose.pose.position.x, -0.2, 0.01);
  EXPECT_NEAR(pose.pose.position.y, -0.2, 0.01);
  EXPECT_NEAR(pose.pose.position.z, -0.1, 0.01);
  EXPECT_EQ(pose.header.frame_id, "base_link");
  EXPECT_NEAR(tf2::getYaw(pose.pose.orientation), 0.0, 0.01);
}

TEST_F(TestTransformPose, TestTimeout)
{
  const auto stamp = clock_->now();
  SetBaseLinkToOdomTransform(stamp);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom";
  pose.header.stamp = stamp;
  pose.header.stamp.sec += 5;

  EXPECT_THROW(
    { panther_utils::tf2_utils::TransformPose(tf2_buffer_, pose, "base_link", 1.0); },
    std::runtime_error);
}

TEST(OffsetPose, CheckDefaultPose)
{
  const auto translation = tf2::Vector3(0.1, 0.2, 0.3);
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 1.57, 3.14);

  tf2::Transform transform;
  transform.setOrigin(translation);
  transform.setRotation(rotation);

  geometry_msgs::msg::PoseStamped pose;
  auto offset_pose = panther_utils::tf2_utils::OffsetPose(pose, transform);

  EXPECT_NEAR(offset_pose.pose.position.x, translation.getX(), 0.01);
  EXPECT_NEAR(offset_pose.pose.position.y, translation.getY(), 0.01);
  EXPECT_NEAR(offset_pose.pose.position.z, translation.getZ(), 0.01);

  tf2::Quaternion offset_rotation;
  tf2::fromMsg(offset_pose.pose.orientation, offset_rotation);
  tf2::Matrix3x3 matrix(offset_rotation);
  double roll, pitch, yaw;
  matrix.getRPY(roll, pitch, yaw);
  EXPECT_NEAR(roll, 0.0, 0.01);
  EXPECT_NEAR(pitch, 1.57, 0.01);
  EXPECT_NEAR(yaw, 3.14, 0.01);
}

TEST(OffsetPose, CheckDefinedPose)
{
  const auto translation = tf2::Vector3(0.7, 0.2, 0.3);
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, M_PI / 2.0, M_PI);

  tf2::Transform transform;
  transform.setOrigin(translation);
  transform.setRotation(rotation);

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0.1;
  pose.pose.position.y = 0.9;
  pose.pose.position.z = 0.2;
  tf2::Quaternion pose_rotation;
  pose_rotation.setRPY(M_PI, M_PI / 2.0, 0.0);
  pose.pose.orientation = tf2::toMsg(pose_rotation);

  auto offset_pose = panther_utils::tf2_utils::OffsetPose(pose, transform);

  EXPECT_NEAR(offset_pose.pose.position.x, pose.pose.position.x - translation.getZ(), 0.01);
  EXPECT_NEAR(offset_pose.pose.position.y, pose.pose.position.y - translation.getY(), 0.01);
  EXPECT_NEAR(offset_pose.pose.position.z, pose.pose.position.z - translation.getX(), 0.01);

  tf2::Quaternion offset_rotation;
  tf2::fromMsg(offset_pose.pose.orientation, offset_rotation);
  tf2::Matrix3x3 matrix(offset_rotation);
  double roll, pitch, yaw;
  matrix.getRPY(roll, pitch, yaw);
  EXPECT_NEAR(roll, 0.0, 0.01);
  EXPECT_NEAR(pitch, 0.0, 0.01);
  EXPECT_NEAR(yaw, 0.0, 0.01);
}
