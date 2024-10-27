// src/dfrobot_dc_motor_hardware.cpp

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
        cfg_.encoder_reduction_ratio = std::stoi(info.hardware_parameters.at("encoder_reduction_ratio"));
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

    wheel_l_.setup(cfg_.left_wheel_name, cfg_.encoder_reduction_ratio);
    wheel_r_.setup(cfg_.right_wheel_name, cfg_.encoder_reduction_ratio);

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
    dfrobot_->setEncoderEnable(DFRobotDCMotor::M1);
    dfrobot_->setEncoderEnable(DFRobotDCMotor::M2);
    dfrobot_->setEncoderReductionRatio(DFRobotDCMotor::M1, cfg_.encoder_reduction_ratio);
    dfrobot_->setEncoderReductionRatio(DFRobotDCMotor::M2, cfg_.encoder_reduction_ratio);
    RCLCPP_INFO(rclcpp::get_logger("DFRobotDCMotorHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DFRobotDCMotorHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("DFRobotDCMotorHardware"), "Deactivating ...please wait...");
    dfrobot_->motorStop(DFRobotDCMotor::M1);
    dfrobot_->motorStop(DFRobotDCMotor::M2);
    dfrobot_->setEncoderDisable(DFRobotDCMotor::M1);
    dfrobot_->setEncoderDisable(DFRobotDCMotor::M2);
    RCLCPP_INFO(rclcpp::get_logger("DFRobotDCMotorHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DFRobotDCMotorHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
    double delta_seconds = period.seconds();

    // Get encoder speeds in RPM
    int16_t rpm_l = -dfrobot_->getEncoderSpeed(DFRobotDCMotor::M1);
    int16_t rpm_r = dfrobot_->getEncoderSpeed(DFRobotDCMotor::M2);

    // Log raw encoder readings
    // RCLCPP_INFO(this->get_logger(), "Raw Encoder Readings - Left RPM: %d, Right RPM: %d", rpm_l, rpm_r);

    // Convert RPM to rad/s
    wheel_l_.vel = rpm_l * 2 * M_PI / 60.0;
    wheel_r_.vel = rpm_r * 2 * M_PI / 60.0;

    // Log converted velocities
    // RCLCPP_INFO(this->get_logger(), "Converted Velocities - Left rad/s: %.3f, Right rad/s: %.3f", wheel_l_.vel, wheel_r_.vel);

    // Integrate to get position
    wheel_l_.pos += wheel_l_.vel * delta_seconds;
    wheel_r_.pos += wheel_r_.vel * delta_seconds;

    // Log updated positions
    // RCLCPP_INFO(this->get_logger(), "Updated Positions - Left pos: %.3f, Right pos: %.3f", wheel_l_.pos, wheel_r_.pos);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DFRobotDCMotorHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    // Convert desired velocities to RPM
    double rpm_l = wheel_l_.cmd * 60.0 / (2 * M_PI);
    double rpm_r = wheel_r_.cmd * 60.0 / (2 * M_PI);

    // Clamp RPM to max RPM
    rpm_l = std::max(std::min(rpm_l, static_cast<double>(cfg_.max_rpm)), -static_cast<double>(cfg_.max_rpm));
    rpm_r = std::max(std::min(rpm_r, static_cast<double>(cfg_.max_rpm)), -static_cast<double>(cfg_.max_rpm));

    // Convert RPM to speed percentage
    float speed_percent_l = std::abs(rpm_l) / cfg_.max_rpm * 100.0f;
    float speed_percent_r = std::abs(rpm_r) / cfg_.max_rpm * 100.0f;

    // Determine orientation
    uint8_t orientation_l = rpm_l >= 0 ? DFRobotDCMotor::CCW : DFRobotDCMotor::CW;
    uint8_t orientation_r = rpm_r >= 0 ? DFRobotDCMotor::CCW : DFRobotDCMotor::CW;

    // Send commands to motor driver
    dfrobot_->motorMovement(DFRobotDCMotor::M1, orientation_l, speed_percent_l);
    dfrobot_->motorMovement(DFRobotDCMotor::M2, orientation_r, speed_percent_r);

    return hardware_interface::return_type::OK;
}

}  // namespace dfrobot_dc_motor_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    dfrobot_dc_motor_hardware::DFRobotDCMotorHardware, hardware_interface::SystemInterface)
