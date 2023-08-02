#ifndef PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_HPP_
#define PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_HPP_

#include <panther_hardware_interfaces/visibility_control.hpp>

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
#include <panther_hardware_interfaces/panther_wheels_controller.hpp>

namespace panther_hardware_interfaces
{
using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

class PantherSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PantherSystem)

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  std::vector<StateInterface> export_state_interfaces() override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  std::vector<CommandInterface> export_command_interfaces() override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  PANTHER_HARDWARE_INTERFACES_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // TODO: naming
  static constexpr size_t JOINTS_SIZE_ = 4;

  // consider adding position and torque mode after updating roboteq firmware to 2.1a
  // In 2.1 both position and torque mode aren't really stable and safe
  // in torque mode sometimes after killing software motor moves and it generally isn't well tuned
  // position mode also isn't really stable (reacts abruptly to spikes, which we hope will be fixed
  // in the new firmware)

  double hw_commands_velocities_[JOINTS_SIZE_];

  double hw_states_positions_[JOINTS_SIZE_];
  double hw_states_velocities_[JOINTS_SIZE_];
  double hw_states_efforts_[JOINTS_SIZE_];

  // Define expected joint order, so that it doesn't mattter order defined in the panther_macro
  // it is expected that joint name should contain these specifiers
  std::string joint_order_[JOINTS_SIZE_] = {"fl", "fr", "rl", "rr"};
  std::string joints_names_sorted_[JOINTS_SIZE_];

  std::unique_ptr<GPIOController> gpio_controller_;
  std::unique_ptr<PantherWheelsController> roboteq_controller_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_HPP_