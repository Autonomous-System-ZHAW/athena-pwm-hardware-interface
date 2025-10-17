#ifndef PWM_HARDWARE_INTERFACE__PWM_SYSTEM_HPP_
#define PWM_HARDWARE_INTERFACE__PWM_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "pwm_hardware_interface/visibility_control.h"
#include <pwm_hardware_interface/pca9685_comm.h>

namespace pwm_hardware_interface
{
class PwmSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PwmSystemHardware)

  PWM_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  PWM_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  PWM_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  PWM_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  PWM_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  PWM_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  PWM_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_commands_;
  PCA9685::PCA9685 pca;
  static constexpr int THROTTLE = 0;
  static constexpr int STEERING = 1;
  double command_to_duty_cycle(double command, double min_input, double max_input, double min_duty_cycle, double max_duty_cycle, double initial_offset, double lower_threshold, double upper_threshold);
  double throttle_command_to_duty_cycle(double command, double min_input, double max_input, double min_duty_cycle, double max_duty_cycle, double lower_threshold, double upper_threshold);
};

}  // namespace pwm_hardware_interface

#endif  // PWM_HARDWARE_INTERFACE__PWM_SYSTEM_HPP_
