#ifndef DELTA_STEPPER_HW_HPP
#define DELTA_STEPPER_HW_HPP

#include "hardware_interface/system_interface.hpp"
#include "delta_stepper_hw/ArduinoStepperDriver.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <memory>
#include <string>
#include <vector>

namespace delta_stepper_hw
{

class DeltaStepperHardware : public hardware_interface::SystemInterface
{
public:

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type
  read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type
  write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

private:

  std::shared_ptr<ArduinoStepperDriver> stepper_driver_;

  std::string serial_device_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<double> hw_velocities_;

};

}

#endif