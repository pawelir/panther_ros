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

#include "panther_docking/panther_charging_dock.hpp"

#include <stdexcept>

#include <nav2_util/node_utils.hpp>

#include "panther_utils/common_utilities.hpp"
#include "panther_utils/tf2_utils.hpp"

namespace panther_docking
{

void PantherChargingDock::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name,
  std::shared_ptr<tf2_ros::Buffer> tf)
{
  name_ = name;

  if (!tf) {
    throw std::runtime_error("PantherChargingDock requires a TF buffer");
  }

  tf2_buffer_ = tf;

  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  declareParameters(node);
  getParameters(node);

  if (!use_wibotic_info_) {
    RCLCPP_INFO(logger_, "Wibotic info is disabled.");
  }

  pose_filter_ = std::make_unique<opennav_docking::PoseFilter>(
    pose_filter_coef_, external_detection_timeout_);
}

void PantherChargingDock::cleanup()
{
  dock_pose_sub_.reset();
  staging_pose_pub_.reset();
}

void PantherChargingDock::activate()
{
  auto node = node_.lock();
  dock_pose_sub_ = node->create_subscription<PoseStampedMsg>(
    "docking/dock_pose", 1,
    std::bind(&PantherChargingDock::setDockPose, this, std::placeholders::_1));
  staging_pose_pub_ = node->create_publisher<PoseStampedMsg>("docking/staging_pose", 1);

  dock_pose_publisher_change_state_client_ =
    node->create_client<lifecycle_msgs::srv::ChangeState>("dock_pose_publisher/change_state");

  if (use_wibotic_info_) {
    wibotic_info_sub_ = node->create_subscription<WiboticInfoMsg>(
      "wibotic_info", 1,
      std::bind(&PantherChargingDock::setWiboticInfo, this, std::placeholders::_1));
  }

  setDockPosePublisherState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
}

void PantherChargingDock::deactivate()
{
  dock_pose_sub_.reset();
  staging_pose_pub_.reset();
  dock_pose_publisher_change_state_client_.reset();
}

void PantherChargingDock::declareParameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  nav2_util::declare_parameter_if_not_declared(
    node, "base_frame", rclcpp::ParameterValue("base_link"));

  nav2_util::declare_parameter_if_not_declared(node, "fixed_frame", rclcpp::ParameterValue("odom"));

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".external_detection_timeout", rclcpp::ParameterValue(0.0));

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".docking_distance_threshold", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".docking_yaw_threshold", rclcpp::ParameterValue(0.3));

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".staging_x_offset", rclcpp::ParameterValue(-0.7));

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".filter_coef", rclcpp::ParameterValue(0.1));

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_wibotic_info", rclcpp::ParameterValue(true));

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".wibotic_info_timeout", rclcpp::ParameterValue(1.5));
}

void PantherChargingDock::getParameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  node->get_parameter("base_frame", base_frame_name_);
  node->get_parameter("fixed_frame", fixed_frame_name_);

  node->get_parameter(name_ + ".external_detection_timeout", external_detection_timeout_);
  node->get_parameter(name_ + ".docking_distance_threshold", docking_distance_threshold_);
  node->get_parameter(name_ + ".docking_yaw_threshold", docking_yaw_threshold_);
  node->get_parameter(name_ + ".staging_x_offset", staging_x_offset_);

  node->get_parameter(name_ + ".filter_coef", pose_filter_coef_);

  node->get_parameter(name_ + ".use_wibotic_info", use_wibotic_info_);
  node->get_parameter(name_ + ".wibotic_info_timeout", wibotic_info_timeout_);
}

// When there is no pose actual position of robot is a staging pose
PantherChargingDock::PoseStampedMsg PantherChargingDock::getStagingPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  RCLCPP_DEBUG_STREAM(logger_, "Getting staging pose in frame: " << frame);

  // No global pose provided, use the detected dock pose
  if (pose != geometry_msgs::msg::Pose()) {
    dock_pose_.pose = pose;
    dock_frame_ = frame;
  }

  updateAndPublishStagingPose(frame);

  return staging_pose_;
}

bool PantherChargingDock::getRefinedPose(PoseStampedMsg & pose)
{
  RCLCPP_DEBUG(logger_, "Getting refined pose");
  setDockPosePublisherState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::Time request_detection_time;

  if (dock_pose_.header.frame_id.empty()) {
    return false;
  }

  {
    auto node = node_.lock();
    request_detection_time = node->now();
  }

  auto timeout = rclcpp::Duration::from_seconds(external_detection_timeout_);
  auto duration = rclcpp::Time(request_detection_time) - rclcpp::Time(dock_pose_.header.stamp);
  if (duration > timeout) {
    RCLCPP_WARN_STREAM(
      logger_, "Detection timeout exceeded. Duration since last detection: "
                 << duration.seconds() << " seconds (timeout threshold: " << timeout.seconds()
                 << " seconds). "
                 << "No detection received or lost detection for external detection.");
    return false;
  }

  pose = dock_pose_;
  updateAndPublishStagingPose(fixed_frame_name_);

  return true;
}

bool PantherChargingDock::isDocked()
{
  RCLCPP_DEBUG(logger_, "Checking if docked");
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = base_frame_name_;

  robot_pose = panther_utils::tf2_utils::TransformPose(tf2_buffer_, robot_pose, fixed_frame_name_);

  return panther_utils::tf2_utils::ArePosesNear(
    robot_pose, dock_pose_, docking_distance_threshold_, docking_yaw_threshold_);
}

bool PantherChargingDock::isCharging()
{
  RCLCPP_DEBUG(logger_, "Checking if charging");
  try {
    if (!use_wibotic_info_) {
      if (isDocked()) {
        setDockPosePublisherState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        return true;
      }
      return false;
    }

    if (!wibotic_info_) {
      setDockPosePublisherState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      throw opennav_docking_core::FailedToCharge("No Wibotic info received.");
    }

    rclcpp::Time requested_wibotic_info_time;
    {
      auto node = node_.lock();
      requested_wibotic_info_time = node->now();
    }

    const auto duration = requested_wibotic_info_time - wibotic_info_->header.stamp;
    if (duration > rclcpp::Duration::from_seconds(wibotic_info_timeout_)) {
      RCLCPP_WARN_STREAM(
        logger_, "Wibotic info is outdated. Time difference is: "
                   << duration.seconds() << "s but timeout is " << wibotic_info_timeout_ << "s.");
      return false;
    }

    if (wibotic_info_->i_charger > kWiboticChargingCurrentThreshold) {
      setDockPosePublisherState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      return true;
    }
  } catch (const opennav_docking_core::FailedToDetectDock & e) {
    RCLCPP_ERROR_STREAM(logger_, "An occurred error while checking if charging: " << e.what());
    setDockPosePublisherState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  }

  return false;
}

bool PantherChargingDock::disableCharging() { return true; }

bool PantherChargingDock::hasStoppedCharging() { return !isCharging(); }

void PantherChargingDock::setDockPose(const PoseStampedMsg::SharedPtr pose)
{
  auto filtered_pose = pose_filter_->update(*pose);
  dock_pose_ = filtered_pose;
}

void PantherChargingDock::updateAndPublishStagingPose(const std::string & frame)
{
  const double yaw = tf2::getYaw(dock_pose_.pose.orientation);
  RCLCPP_DEBUG_STREAM(
    logger_, "Dock pose x: " << dock_pose_.pose.position.x << " y: " << dock_pose_.pose.position.y
                             << " yaw: " << yaw);

  staging_pose_ = dock_pose_;
  staging_pose_.header.frame_id = frame;
  staging_pose_.header.stamp = node_.lock()->now();
  staging_pose_.pose.position.x += std::cos(yaw) * staging_x_offset_;
  staging_pose_.pose.position.y += std::sin(yaw) * staging_x_offset_;
  staging_pose_.pose.position.z = 0.0;

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  staging_pose_.pose.orientation = tf2::toMsg(orientation);

  staging_pose_pub_->publish(staging_pose_);
}

void PantherChargingDock::setWiboticInfo(const WiboticInfoMsg::SharedPtr msg)
{
  wibotic_info_ = std::make_shared<WiboticInfoMsg>(*msg);
}

void PantherChargingDock::setDockPosePublisherState(std::uint8_t state)
{
  if (dock_pose_publisher_state_ == state) {
    return;
  }

  RCLCPP_DEBUG_STREAM(logger_, "Setting dock pose publisher state to: " << static_cast<int>(state));
  dock_pose_publisher_state_ = state;

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = state;
  dock_pose_publisher_change_state_client_->async_send_request(request);
}

}  // namespace panther_docking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(panther_docking::PantherChargingDock, opennav_docking_core::ChargingDock)
