#include "pwm_hardware_interface/pwm_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"



namespace pwm_hardware_interface {

hardware_interface::CallbackReturn PwmSystemHardware::on_init(const hardware_interface::HardwareInfo & info) {

    // pca.set_pwm_freq(50.0);

    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.joints.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger("PwmSystemHardware"),
        "%zu joints found. 2 expected.", 
        info_.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {

      RCLCPP_INFO(rclcpp::get_logger("PwmSystemHardware"), "Joint: %s", joint.name.c_str());
      // Pca9685System has one command interface on each output
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("PwmSystemHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // First joint is throttle joint with a velocity interface
    hardware_interface::ComponentInfo &joint_throttle = info_.joints[THROTTLE];
    if (joint_throttle.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
          rclcpp::get_logger("PwmSystemHardware"),
          "Joint '%s' have %s command interfaces found. '%s' expected.", joint_throttle.name.c_str(),
          joint_throttle.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Second joint is steering joint with a position interface
    hardware_interface::ComponentInfo &joint_steer = info_.joints[STEERING];
    if (joint_steer.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
          rclcpp::get_logger("PwmSystemHardware"),
          "Joint '%s' have %s command interfaces found. '%s' expected.", joint_steer.name.c_str(),
          joint_steer.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> PwmSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  RCLCPP_INFO(rclcpp::get_logger("PwmSystemHardware"), "Joint 0: %s", info_.joints[THROTTLE].name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("PwmSystemHardware"), "Joint 1: %s", info_.joints[STEERING].name.c_str());

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[THROTTLE].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[THROTTLE]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[STEERING].name, hardware_interface::HW_IF_POSITION, &hw_commands_[STEERING]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PwmSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[THROTTLE].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[THROTTLE]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[STEERING].name, hardware_interface::HW_IF_POSITION, &hw_commands_[STEERING]));

  return command_interfaces;
}

hardware_interface::CallbackReturn PwmSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    if (std::isnan(hw_commands_[i]))
    {
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("PwmSystemHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    hw_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("PwmSystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PwmSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  return hardware_interface::return_type::OK;
}

double PwmSystemHardware::command_to_duty_cycle(double command, double min_input, double max_input, double min_duty_cycle, double max_duty_cycle, double initial_offset, double lower_threshold, double upper_threshold)
{
  double clamped_command = std::clamp(command, min_input, max_input);

  if (clamped_command > 0)
  {
    min_input = 0;
    min_duty_cycle = 1.5 + initial_offset + upper_threshold;
    double slope = (max_duty_cycle - min_duty_cycle) / (max_input - min_input);
    return min_duty_cycle + slope * clamped_command;
  }
  if (clamped_command < 0)
  {
    max_input = 0;
    max_duty_cycle = 1.5 + initial_offset - lower_threshold;
    double slope = (max_duty_cycle - min_duty_cycle) / (max_input - min_input);
    return max_duty_cycle + slope * clamped_command;
  }

  return 1.5 + initial_offset;
}

double PwmSystemHardware::throttle_command_to_duty_cycle(double command, double min_input, double max_input, double min_duty_cycle, double max_duty_cycle, double lower_threshold, double upper_threshold)
{
  double clamped_command = std::clamp(command, min_input, max_input);

  if (clamped_command > 0)
  {
    clamped_command = clamped_command - 61.54;
    clamped_command = std::max(0.0, clamped_command);
    min_input = 0;
    min_duty_cycle = 1.5 + upper_threshold;
    double slope = (max_duty_cycle - min_duty_cycle) / (max_input - min_input - 61.54);
    return min_duty_cycle + slope * clamped_command; 
  }
  if (clamped_command < 0)
  {
    return min_duty_cycle;
  }
  return 1.5;
}

hardware_interface::return_type PwmSystemHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double duty_cycle = throttle_command_to_duty_cycle(hw_commands_[THROTTLE], -540, 190, 1.44, 1.665, 0.058, 0.0917);

  // RCLCPP_INFO(
  //     rclcpp::get_logger("PwmSystemHardware"),
  //     "Joint '%d' has command '%f', duty_cycle: '%f'.", THROTTLE, hw_commands_[THROTTLE], duty_cycle);

  pca.set_pwm_ms(THROTTLE, duty_cycle);

  duty_cycle = command_to_duty_cycle(hw_commands_[STEERING], -0.38397, 0.37961, 1.226, 1.675, -0.05, 0, 0);

  // RCLCPP_INFO(
  //     rclcpp::get_logger("PwmSystemHardware"),
  //     "Joint '%d' has command '%f', duty_cycle: '%f'.", STEERING, hw_commands_[STEERING], duty_cycle);

  pca.set_pwm_ms(STEERING, duty_cycle);

  return hardware_interface::return_type::OK;
}

}  // namespace pwm_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pwm_hardware_interface::PwmSystemHardware, hardware_interface::SystemInterface)
