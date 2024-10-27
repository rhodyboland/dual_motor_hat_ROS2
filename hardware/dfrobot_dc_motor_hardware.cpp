// dfrobot_dc_motor_hardware.cpp

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

    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.bus_id = std::stoi(info_.hardware_parameters["bus_id"]);
    cfg_.i2c_address = std::stoi(info_.hardware_parameters["i2c_address"]);
    cfg_.encoder_reduction_ratio = std::stoi(info_.hardware_parameters["encoder_reduction_ratio"]);
    cfg_.max_rpm = std::stoi(info_.hardware_parameters["max_rpm"]);

    wheel_l_.setup(cfg_.left_wheel_name, cfg_.encoder_reduction_ratio);
    wheel_r_.setup(cfg_.right_wheel_name, cfg_.encoder_reduction_ratio);

    max_speed_ = cfg_.max_rpm * 2 * M_PI / 60.0;  // Convert RPM to rad/s

    // Initialize dfrobot_ after parsing configuration
    dfrobot_ = std::make_unique<DFRobotDCMotor>(cfg_.bus_id, cfg_.i2c_address);

    return hardware_interface::CallbackReturn::SUCCESS;
}

// Update other methods to use dfrobot_ pointer

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
    dfrobot_->setEncoderEnable(DFRobotDCMotor::ALL);
    dfrobot_->setEncoderReductionRatio(DFRobotDCMotor::M1, cfg_.encoder_reduction_ratio);
    dfrobot_->setEncoderReductionRatio(DFRobotDCMotor::M2, cfg_.encoder_reduction_ratio);
    RCLCPP_INFO(rclcpp::get_logger("DFRobotDCMotorHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DFRobotDCMotorHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("DFRobotDCMotorHardware"), "Deactivating ...please wait...");
    dfrobot_->motorStop(DFRobotDCMotor::ALL);
    dfrobot_->setEncoderDisable(DFRobotDCMotor::ALL);
    RCLCPP_INFO(rclcpp::get_logger("DFRobotDCMotorHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DFRobotDCMotorHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
    double delta_seconds = period.seconds();

    // Get encoder speeds in RPM
    int16_t rpm_l = dfrobot_->getEncoderSpeed(DFRobotDCMotor::M1);
    int16_t rpm_r = dfrobot_->getEncoderSpeed(DFRobotDCMotor::M2);

    // Convert RPM to rad/s
    wheel_l_.vel = rpm_l * 2 * M_PI / 60.0;
    wheel_r_.vel = rpm_r * 2 * M_PI / 60.0;

    // Integrate to get position
    wheel_l_.pos += wheel_l_.vel * delta_seconds;
    wheel_r_.pos += wheel_r_.vel * delta_seconds;

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
    uint8_t orientation_l = rpm_l >= 0 ? DFRobotDCMotor::CW : DFRobotDCMotor::CCW;
    uint8_t orientation_r = rpm_r >= 0 ? DFRobotDCMotor::CW : DFRobotDCMotor::CCW;

    // Send commands to motor driver
    dfrobot_->motorMovement(DFRobotDCMotor::M1, orientation_l, speed_percent_l);
    dfrobot_->motorMovement(DFRobotDCMotor::M2, orientation_r, speed_percent_r);

    return hardware_interface::return_type::OK;
}

}  // namespace dfrobot_dc_motor_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    dfrobot_dc_motor_hardware::DFRobotDCMotorHardware, hardware_interface::SystemInterface)
