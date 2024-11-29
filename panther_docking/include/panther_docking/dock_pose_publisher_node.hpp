// Copyright (c) 2024 Husarion Sp. z o.o.
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

#ifndef PANTHER_DOCKING_DOCK_POSE_PUBLISHER_NODE_HPP_
#define PANTHER_DOCKING_DOCK_POSE_PUBLISHER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace panther_docking
{
constexpr double kMinimalDetectionDistance = 1.0;

class DockPosePublisherNode : public rclcpp::Node
{
public:
  DockPosePublisherNode(const std::string & name);

private:
  void publishPose();

  double timeout_;
  std::string target_frame_;
  std::vector<std::string> source_frames_;
  std::string base_frame_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace panther_docking

#endif  // PANTHER_DOCKING_DOCK_POSE_PUBLISHER_NODE_HPP_
