#ifndef DFROBOT_DC_MOTOR_HARDWARE_HPP_
#define DFROBOT_DC_MOTOR_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "dfrobot_dc_motor.hpp"
#include "dfrobot_dc_motor_hardware/wheel.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

// Include the visibility control header
#include "dfrobot_dc_motor_hardware/visibility_control.h"

namespace dfrobot_dc_motor_hardware {
class DFROBOT_DC_MOTOR_HARDWARE_PUBLIC DFRobotDCMotorHardware : public hardware_interface::SystemInterface {
   public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DFRobotDCMotorHardware);

    // Apply the visibility macro to each method
    DFROBOT_DC_MOTOR_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    DFROBOT_DC_MOTOR_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    DFROBOT_DC_MOTOR_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    DFROBOT_DC_MOTOR_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    DFROBOT_DC_MOTOR_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    DFROBOT_DC_MOTOR_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    DFROBOT_DC_MOTOR_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    DFROBOT_DC_MOTOR_HARDWARE_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    DFROBOT_DC_MOTOR_HARDWARE_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

   private:
    struct Config {
        std::string left_wheel_name = "";
        std::string right_wheel_name = "";
        int bus_id = 1;
        uint8_t i2c_address = 0x10;
        int encoder_reduction_ratio = 210;
        int max_rpm = 75;
    };

    Config cfg_;
    std::unique_ptr<DFRobotDCMotor> dfrobot_;
    Wheel wheel_l_;
    Wheel wheel_r_;

    double max_speed_;  // Max wheel speed in rad/s
};

}  // namespace dfrobot_dc_motor_hardware

#endif  // DFROBOT_DC_MOTOR_HARDWARE_HPP_
