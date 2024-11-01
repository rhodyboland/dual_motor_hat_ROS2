// hardware/dfrobot_dc_motor_hardware.cpp

#include "dfrobot_dc_motor_hardware/dfrobot_dc_motor_hardware.hpp"

#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dfrobot_dc_motor_hardware {
hardware_interface::CallbackReturn DFRobotDCMotorHardware::on_init(
    const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    try {
        cfg_.left_wheel_name = info.hardware_parameters.at("left_wheel_name");
        cfg_.right_wheel_name = info.hardware_parameters.at("right_wheel_name");
        cfg_.bus_id = std::stoi(info.hardware_parameters.at("bus_id"));
        cfg_.i2c_address = static_cast<uint8_t>(std::stoi(info.hardware_parameters.at("i2c_address")));
        cfg_.ticks_per_revolution = std::stoi(info.hardware_parameters.at("ticks_per_revolution"));
        cfg_.max_rpm = std::stoi(info.hardware_parameters.at("max_rpm"));
    } catch (const std::out_of_range& e) {
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotorHardware"),
                     "Missing hardware parameter: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    } catch (const std::invalid_argument& e) {
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotorHardware"),
                     "Invalid hardware parameter value: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    wheel_l_.setup(cfg_.left_wheel_name);
    wheel_r_.setup(cfg_.right_wheel_name);

    max_speed_ = cfg_.max_rpm * 2 * M_PI / 60.0;  // Convert RPM to rad/s

    // Initialize dfrobot_ after parsing configuration
    dfrobot_ = std::make_unique<DFRobotDCMotor>(cfg_.bus_id, cfg_.i2c_address);

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DFRobotDCMotorHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DFRobotDCMotorHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

    return command_interfaces;
}

hardware_interface::CallbackReturn DFRobotDCMotorHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("DFRobotDCMotorHardware"), "Configuring ...please wait...");
    if (!dfrobot_->begin()) {
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotorHardware"), "Failed to initialize motor driver");
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("DFRobotDCMotorHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DFRobotDCMotorHardware::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("DFRobotDCMotorHardware"), "Cleaning up ...please wait...");
    RCLCPP_INFO(rclcpp::get_logger("DFRobotDCMotorHardware"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DFRobotDCMotorHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("DFRobotDCMotorHardware"), "Activating ...please wait...");
    // Any activation code if necessary
    RCLCPP_INFO(rclcpp::get_logger("DFRobotDCMotorHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DFRobotDCMotorHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("DFRobotDCMotorHardware"), "Deactivating ...please wait...");
    // Stop the motors
    dfrobot_->setRawMotorSpeed(0, 0);
    RCLCPP_INFO(rclcpp::get_logger("DFRobotDCMotorHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DFRobotDCMotorHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
    double delta_seconds = period.seconds();

    int16_t left_ticks = 0;
    int16_t right_ticks = 0;

    if (!dfrobot_->getEncoderTicks(left_ticks, right_ticks)) {
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotorHardware"), "Failed to read encoder ticks");
        return hardware_interface::return_type::ERROR;
    }

    // Convert ticks to radians
    double delta_pos_left = (left_ticks / static_cast<double>(cfg_.ticks_per_revolution)) * 2 * M_PI;
    double delta_pos_right = (right_ticks / static_cast<double>(cfg_.ticks_per_revolution)) * 2 * M_PI;

    // Update positions
    wheel_l_.pos += delta_pos_left;
    wheel_r_.pos += delta_pos_right;

    // Update velocities
    wheel_l_.vel = delta_pos_left / delta_seconds;
    wheel_r_.vel = delta_pos_right / delta_seconds;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DFRobotDCMotorHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    // Convert desired velocities to motor commands (-127 to 127)
    double motor_command_left = (wheel_l_.cmd / max_speed_) * 127.0;
    double motor_command_right = (wheel_r_.cmd / max_speed_) * 127.0;

    // Clamp to -127 to 127
    motor_command_left = std::max(std::min(motor_command_left, 127.0), -127.0);
    motor_command_right = std::max(std::min(motor_command_right, 127.0), -127.0);

    // Cast to int8_t
    int8_t left_command = static_cast<int8_t>(motor_command_left);
    int8_t right_command = static_cast<int8_t>(motor_command_right);

    // Send commands to motor driver
    if (!dfrobot_->setRawMotorSpeed(left_command, right_command)) {
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotorHardware"), "Failed to set motor speed");
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

}  // namespace dfrobot_dc_motor_hardware

// Plugin registration
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    dfrobot_dc_motor_hardware::DFRobotDCMotorHardware, hardware_interface::SystemInterface)
