// Copyright 2023 Husarion sp. z o.o.
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

#ifndef PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_HPP_
#define PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <panther_hardware_interfaces/gpio_driver.hpp>
#include <panther_hardware_interfaces/motors_controller.hpp>
#include <panther_hardware_interfaces/panther_system_ros_interface.hpp>
#include <panther_hardware_interfaces/roboteq_error_filter.hpp>

namespace panther_hardware_interfaces
{
using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

/**
 * @brief Class that implements SystemInterface from ros2_control for Panther
 */
class PantherSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PantherSystem)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<StateInterface> export_state_interfaces() override;
  std::vector<CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  void CheckJointSize();
  void SortJointNames();
  void CheckJointNames();
  void SetInitialValues();
  void CheckInterfaces();
  void ReadDrivetrainSettings();
  void ReadCanOpenSettings();
  void ReadInitializationActivationAttempts();
  void ReadParametersAndCreateRoboteqErrorFilter();

  void UpdateHwStates();
  void UpdateDriverState();
  void UpdateSystemFeedback();

  static constexpr size_t kJointsSize = 4;

  // Currently only velocity command mode is supported - although Roboteq driver support position
  // and torque mode, in 2.1 firmware both modes aren't really stable and safe.
  // In torque mode sometimes after killing the software motor moves and it generally isn't well
  // tuned. Position mode also isn't really stable (reacts abruptly to spikes). If updating the
  // firmware to 2.1a will solve these issues, it may be worth to enable other modes.
  double hw_commands_velocities_[kJointsSize];

  double hw_states_positions_[kJointsSize];
  double hw_states_velocities_[kJointsSize];
  double hw_states_efforts_[kJointsSize];

  // Define expected joint order, so that it doesn't matter what order is defined in the URDF.
  // It is expected that the joint name should contain these specifiers.
  std::string joint_order_[kJointsSize] = {"fl", "fr", "rl", "rr"};
  std::string joints_names_sorted_[kJointsSize];

  std::unique_ptr<GPIOController> gpio_controller_;
  std::shared_ptr<MotorsController> motors_controller_;

  DrivetrainSettings drivetrain_settings_;
  CanOpenSettings canopen_settings_;

  std::shared_ptr<RoboteqErrorFilter> roboteq_error_filter_;

  PantherSystemRosInterface panther_system_ros_interface_;

  // Sometimes SDO errors can happen during initialization and activation of Roboteq drivers,
  // in these cases it is better to retry
  // Example errors:
  // SDO abort code 05040000 received on upload request of object 1000 (Device type) to
  // node 02: SDO protocol timed out
  // SDO abort code 05040000 received on upload request of sub-object 1018:01 (Vendor-ID) to
  // node 02: SDO protocol timed out
  unsigned max_roboteq_initialization_attempts_ = 2;
  unsigned max_roboteq_activation_attempts_ = 2;

  // SDO error can happen also during setting safety stop (it may be not necessary to use attempts
  // once we have GPIO controller)
  unsigned max_safety_stop_attempts_ = 20;

  rclcpp::Logger logger_{rclcpp::get_logger("PantherSystem")};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // TODO: refactor
  std::size_t read_sdo_errors_filter_id_;
  std::size_t write_sdo_errors_filter_id_;
  std::size_t read_pdo_errors_filter_id_;
  std::size_t roboteq_driver_errors_filter_id_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_HPP_
