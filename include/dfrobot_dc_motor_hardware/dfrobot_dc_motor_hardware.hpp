#ifndef DFROBOT_DC_MOTOR_HARDWARE_HPP
#define DFROBOT_DC_MOTOR_HARDWARE_HPP

#include <tf2_ros/transform_broadcaster.h>

#include <atomic>  // Added for atomic flag
#include <cstdint>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>

#include "i2c_interface.hpp"

namespace dfrobot_dc_motor_hardware {

struct MotorState {
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
};

struct MotorCommand {
    double velocity = 0.0;
};

class DFRobotDCMotorHardware : public hardware_interface::SystemInterface {
   public:
    // Lifecycle methods with corrected return types
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces()
        override;
    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    // Destructor
    ~DFRobotDCMotorHardware();

   private:
    std::unique_ptr<I2CInterface> i2c_;
    uint8_t i2c_address_;
    std::string i2c_device_;

    // Define motor states and commands
    std::vector<MotorState> motor_states_;
    std::vector<MotorCommand> motor_commands_;

    // Define constants similar to the Python library
    static constexpr uint8_t REG_PID = 0x01;
    static constexpr uint8_t REG_PVD = 0x02;
    static constexpr uint8_t REG_CTRL_MODE = 0x03;
    static constexpr uint8_t REG_ENCODER1_EN = 0x04;
    static constexpr uint8_t REG_ENCODER1_SPEED = 0x05;
    static constexpr uint8_t REG_ENCODER1_REDUCTION_RATIO = 0x07;
    static constexpr uint8_t REG_ENCODER2_EN = 0x09;
    static constexpr uint8_t REG_ENCODER2_SPEED = 0x0A;
    static constexpr uint8_t REG_ENCODER2_REDUCTION_RATIO = 0x0C;
    static constexpr uint8_t REG_MOTOR_PWM = 0x0E;
    static constexpr uint8_t REG_MOTOR1_ORIENTATION = 0x0F;
    static constexpr uint8_t REG_MOTOR1_SPEED = 0x10;
    static constexpr uint8_t REG_MOTOR2_ORIENTATION = 0x12;
    static constexpr uint8_t REG_MOTOR2_SPEED = 0x13;

    static constexpr uint8_t DEF_PID = 0xDF;
    static constexpr uint8_t DEF_VID = 0x10;

    // Control modes
    static constexpr uint8_t CONTROL_MODE_DC_MOTOR = 0x00;
    static constexpr uint8_t CONTROL_MODE_STEPPER = 0x01;

    // Orientations
    static constexpr uint8_t ORIENTATION_CW = 0x01;
    static constexpr uint8_t ORIENTATION_CCW = 0x02;
    static constexpr uint8_t ORIENTATION_STOP = 0x05;

    // Motor IDs
    static constexpr size_t MOTOR_COUNT = 2;

    // Odometry Parameters
    double wheel_radius_ = 0.05;  // meters (adjust to your wheel radius)
    double wheel_base_ = 0.3;     // meters (distance between wheels)
    int encoder_resolution_ =
        1024;  // counts per wheel rotation (adjust accordingly)

    // Odometry Variables
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;

    int last_encoder_count_left_ = 0;
    int last_encoder_count_right_ = 0;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<rclcpp::Node> node_;

    // Thread for spinning node
    std::thread node_spin_thread_;
    std::atomic<bool> stop_thread_{false};  // Use atomic flag for thread safety
};
}  // namespace dfrobot_dc_motor_hardware

#endif  // DFROBOT_DC_MOTOR_HARDWARE_HPP
