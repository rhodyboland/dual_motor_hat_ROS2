#include "dfrobot_dc_motor_hardware/dfrobot_dc_motor_hardware.hpp"

#include <tf2/convert.h>  // Optional, if needed

#include <chrono>
#include <cmath>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // Corrected include

namespace dfrobot_dc_motor_hardware {

DFRobotDCMotorHardware::~DFRobotDCMotorHardware() {
    // Signal the spin thread to stop
    stop_thread_ = true;

    // Optionally, destroy the node to allow spin to exit
    node_.reset();

    // Join the spin thread if it's running
    if (node_spin_thread_.joinable()) {
        node_spin_thread_.join();
    }
}

hardware_interface::CallbackReturn DFRobotDCMotorHardware::on_init(
    const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Parse I2C parameters from YAML config
    if (info_.hardware_parameters.find("i2c_device") ==
            info_.hardware_parameters.end() ||
        info_.hardware_parameters.find("i2c_address") ==
            info_.hardware_parameters.end()) {
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotorHardware"),
                     "Missing i2c_device or i2c_address parameters");
        return hardware_interface::CallbackReturn::ERROR;
    }

    i2c_device_ = info_.hardware_parameters.at("i2c_device");
    std::string addr_str = info_.hardware_parameters.at("i2c_address");
    try {
        i2c_address_ = std::stoi(addr_str, nullptr, 0);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotorHardware"),
                     "Invalid i2c_address format: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize I2C interface
    try {
        i2c_ = std::make_unique<I2CInterface>(i2c_device_, i2c_address_);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotorHardware"),
                     "I2C Initialization failed: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize motors
    size_t motor_count = info_.joints.size();
    if (motor_count != MOTOR_COUNT) {
        RCLCPP_WARN(rclcpp::get_logger("DFRobotDCMotorHardware"),
                    "Expected %zu motors, but got %zu", MOTOR_COUNT,
                    motor_count);
    }
    motor_states_.resize(motor_count);
    motor_commands_.resize(motor_count);

    // Initialize ROS node
    node_ = std::make_shared<rclcpp::Node>("dfrobot_dc_motor_hardware_node");

    // Initialize odometry publisher
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    // Initialize last encoder counts
    last_encoder_count_left_ = 0;
    last_encoder_count_right_ = 0;

    // Start spinning the node in a separate thread with a spin loop that checks
    // the stop flag
    node_spin_thread_ = std::thread([this]() {
        while (rclcpp::ok() && !stop_thread_) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(10));  // Adjust as needed
        }
    });

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DFRobotDCMotorHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &motor_states_[i].velocity));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &motor_states_[i].position));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DFRobotDCMotorHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &motor_commands_[i].velocity));
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn DFRobotDCMotorHardware::on_activate(
    const rclcpp_lifecycle::State &) {
    // Set control mode to DC Motor
    try {
        i2c_->writeBytes(REG_CTRL_MODE, {CONTROL_MODE_DC_MOTOR});
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotorHardware"),
                     "Failed to set control mode: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Stop all motors on activation
    try {
        for (size_t i = 0; i < MOTOR_COUNT; ++i) {
            uint8_t reg = REG_MOTOR1_ORIENTATION + (i * 3);
            i2c_->writeBytes(reg, {ORIENTATION_STOP});
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotorHardware"),
                     "Failed to stop motors: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize odometry variables
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    last_encoder_count_left_ = 0;
    last_encoder_count_right_ = 0;

    // Start spinning the node in a separate thread
    stop_thread_ = false;
    node_spin_thread_ = std::thread([this]() {
        while (rclcpp::ok() && !stop_thread_) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(10));  // Adjust as needed
        }
    });

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DFRobotDCMotorHardware::on_deactivate(
    const rclcpp_lifecycle::State &) {
    // Optionally stop motors on deactivation
    try {
        for (size_t i = 0; i < MOTOR_COUNT; ++i) {
            uint8_t reg = REG_MOTOR1_ORIENTATION + (i * 3);
            i2c_->writeBytes(reg, {ORIENTATION_STOP});
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotorHardware"),
                     "Failed to stop motors on deactivate: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Signal the spin thread to stop
    stop_thread_ = true;

    // Optionally, destroy the node to allow spin to exit
    node_.reset();

    // Join the spin thread if it's running
    if (node_spin_thread_.joinable()) {
        node_spin_thread_.join();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DFRobotDCMotorHardware::read(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
    // Read encoder counts
    int encoder_count_left = 0;
    int encoder_count_right = 0;
    try {
        // Assuming motor1 is left and motor2 is right
        auto data_left = i2c_->readBytes(REG_ENCODER1_SPEED, 2);
        encoder_count_left =
            (static_cast<int16_t>(data_left[0] << 8)) | data_left[1];

        auto data_right = i2c_->readBytes(REG_ENCODER2_SPEED, 2);
        encoder_count_right =
            (static_cast<int16_t>(data_right[0] << 8)) | data_right[1];
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotorHardware"),
                     "Failed to read encoder counts: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }

    // Compute delta encoder counts
    int delta_left = encoder_count_left - last_encoder_count_left_;
    int delta_right = encoder_count_right - last_encoder_count_right_;

    last_encoder_count_left_ = encoder_count_left;
    last_encoder_count_right_ = encoder_count_right;

    // Convert encoder counts to distance
    double distance_per_count =
        (2.0 * M_PI * wheel_radius_) / encoder_resolution_;
    double delta_s_left = delta_left * distance_per_count;
    double delta_s_right = delta_right * distance_per_count;

    // Compute average and delta
    double delta_s = (delta_s_right + delta_s_left) / 2.0;
    double delta_theta = (delta_s_right - delta_s_left) / wheel_base_;

    // Update pose
    double delta_x = delta_s * std::cos(theta_ + delta_theta / 2.0);
    double delta_y = delta_s * std::sin(theta_ + delta_theta / 2.0);

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    // Normalize theta to [-pi, pi]
    while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    // Publish odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // Set position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    // Set orientation
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, theta_);
    odom.pose.pose.orientation = tf2::toMsg(odom_quat);

    // Set velocity
    odom.twist.twist.linear.x = delta_s / period.seconds();
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = delta_theta / period.seconds();

    odom_pub_->publish(odom);

    // Broadcast TF
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = time;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";

    odom_tf.transform.translation.x = x_;
    odom_tf.transform.translation.y = y_;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = tf2::toMsg(odom_quat);

    tf_broadcaster_->sendTransform(odom_tf);

    // Read encoder speeds and update motor_states_.velocity
    try {
        for (size_t i = 0; i < MOTOR_COUNT; ++i) {
            uint8_t reg = REG_ENCODER1_SPEED + (i * 5);
            auto data = i2c_->readBytes(reg, 2);
            int16_t speed = (static_cast<int16_t>(data[0] << 8)) | data[1];
            motor_states_[i].velocity =
                static_cast<double>(speed);  // Assuming RPM
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotorHardware"),
                     "Failed to read encoder speeds: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DFRobotDCMotorHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &) {
    // Write motor commands (velocity)
    try {
        for (size_t i = 0; i < MOTOR_COUNT; ++i) {
            double cmd_velocity = motor_commands_[i].velocity;

            // Clamp speed to [-100, 100]
            if (cmd_velocity > 100.0) cmd_velocity = 100.0;
            if (cmd_velocity < -100.0) cmd_velocity = -100.0;

            // Determine orientation and speed based on command
            uint8_t orientation =
                (cmd_velocity >= 0) ? ORIENTATION_CW : ORIENTATION_CCW;
            uint8_t speed =
                static_cast<uint8_t>(std::abs(cmd_velocity));  // Assuming 0-100

            uint8_t reg_orientation = REG_MOTOR1_ORIENTATION + (i * 3);
            uint8_t reg_speed = REG_MOTOR1_SPEED + (i * 3);

            i2c_->writeBytes(reg_orientation, {orientation});
            i2c_->writeBytes(
                reg_speed, {speed, 0});  // Assuming second byte is decimal part
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotorHardware"),
                     "Failed to write motor commands: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

}  // namespace dfrobot_dc_motor_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dfrobot_dc_motor_hardware::DFRobotDCMotorHardware,
                       hardware_interface::SystemInterface)
