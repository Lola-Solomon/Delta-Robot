#include "delta_stepper_hw/delta_stepper_hw.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace delta_stepper_hw {

hardware_interface::CallbackReturn DeltaStepperHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    serial_device_ = "/dev/ttyACM0";

    stepper_driver_ = std::make_shared<ArduinoStepperDriver>(serial_device_);

    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_states_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(), 0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DeltaStepperHardware::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    if (stepper_driver_->init() != 0)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DeltaStepperHardware::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    stepper_driver_->activate();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DeltaStepperHardware::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    stepper_driver_->deactivate();

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DeltaStepperHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (uint i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_states_[i]));

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &hw_velocities_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DeltaStepperHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (uint i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_positions_[i]));
    }

    return command_interfaces;
}

hardware_interface::return_type DeltaStepperHardware::read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
        hw_states_[i] = hw_positions_[i];
        hw_velocities_[i] = 0.0;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DeltaStepperHardware::write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    double rad_joint1 = hw_positions_[0];
    double rad_joint2 = hw_positions_[1];
    double rad_joint3 = hw_positions_[2];

    stepper_driver_->setTargetPositions(rad_joint1, rad_joint2, rad_joint3);

    return hardware_interface::return_type::OK;
}

} // namespace delta_stepper_hw


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    delta_stepper_hw::DeltaStepperHardware,
    hardware_interface::SystemInterface)