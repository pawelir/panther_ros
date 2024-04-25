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

#ifndef PANTHER_MANAGER_SAFETY_MANAGER_NODE_HPP_
#define PANTHER_MANAGER_SAFETY_MANAGER_NODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "panther_msgs/msg/driver_state.hpp"
#include "panther_msgs/msg/io_state.hpp"
#include "panther_msgs/msg/system_status.hpp"

#include "panther_utils/moving_average.hpp"

namespace panther_manager
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using BoolMsg = std_msgs::msg::Bool;
using DriverStateMsg = panther_msgs::msg::DriverState;
using IOStateMsg = panther_msgs::msg::IOState;
using SystemStatusMsg = panther_msgs::msg::SystemStatus;

class SafetyManagerNode : public rclcpp::Node
{
public:
  SafetyManagerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SafetyManagerNode() {}

  void Initialize();

protected:
  void DeclareParameters();
  void RegisterBehaviorTree();
  void CreateSafetyTree();
  void CreateShutdownTree();
  bool SystemReady();

private:
  void BatteryCB(const BatteryStateMsg::SharedPtr battery);
  void DriverStateCB(const DriverStateMsg::SharedPtr driver_state);
  void EStopCB(const BoolMsg::SharedPtr e_stop);
  void IOStateCB(const IOStateMsg::SharedPtr io_state);
  void SystemStatusCB(const SystemStatusMsg::SharedPtr system_status);
  void SafetyTreeTimerCB();

  void ShutdownRobot(const std::string & reason);

  static constexpr float kCriticalBatteryTemp = 55.0;
  static constexpr float kFatalBatteryTemp = 62.0;

  float update_charging_anim_step_;

  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_sub_;
  rclcpp::Subscription<DriverStateMsg>::SharedPtr driver_state_sub_;
  rclcpp::Subscription<BoolMsg>::SharedPtr e_stop_sub_;
  rclcpp::Subscription<IOStateMsg>::SharedPtr io_state_sub_;
  rclcpp::Subscription<SystemStatusMsg>::SharedPtr system_status_sub_;
  rclcpp::TimerBase::SharedPtr safety_tree_timer_;

  BT::BehaviorTreeFactory factory_;
  BT::NodeConfig safety_config_;
  BT::NodeConfig shutdown_config_;
  BT::Tree safety_tree_;
  BT::Tree shutdown_tree_;
  std::unique_ptr<BT::Groot2Publisher> safety_bt_publisher_;
  std::unique_ptr<BT::Groot2Publisher> shutdown_bt_publisher_;

  std::unique_ptr<panther_utils::MovingAverage<double>> battery_temp_ma_;
  std::unique_ptr<panther_utils::MovingAverage<double>> cpu_temp_ma_;
  std::unique_ptr<panther_utils::MovingAverage<double>> front_driver_temp_ma_;
  std::unique_ptr<panther_utils::MovingAverage<double>> rear_driver_temp_ma_;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_SAFETY_MANAGER_NODE_HPP_
