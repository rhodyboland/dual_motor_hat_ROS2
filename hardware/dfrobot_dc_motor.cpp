// hardware/dfrobot_dc_motor.cpp

#include "dfrobot_dc_motor_hardware/dfrobot_dc_motor.hpp"

#include <cstring>
#include <rclcpp/rclcpp.hpp>

DFRobotDCMotor::DFRobotDCMotor(int bus_id, uint8_t addr) : bus_id_(bus_id), addr_(addr) {
    file_i2c_ = -1;
}

DFRobotDCMotor::~DFRobotDCMotor() {
    if (file_i2c_ > 0) {
        close(file_i2c_);
    }
}

bool DFRobotDCMotor::begin() {
    // Open I2C bus
    std::string bus_name = "/dev/i2c-" + std::to_string(bus_id_);

    if ((file_i2c_ = open(bus_name.c_str(), O_RDWR)) < 0) {
        // Failed to open the bus
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotor"), "Failed to open the I2C bus");
        return false;
    }

    // Set the I2C slave address
    if (ioctl(file_i2c_, I2C_SLAVE, addr_) < 0) {
        // Failed to acquire bus access and/or talk to slave
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotor"), "Failed to acquire bus access and/or talk to slave");
        return false;
    }

    // Read WHO_AM_I to confirm the device is connected
    std::vector<uint8_t> who_am_i(1);
    if (!readBytes(CMD_WHO_AM_I, 1, who_am_i)) {
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotor"), "Failed to read WHO_AM_I");
        return false;
    }

    if (who_am_i[0] != addr_) {
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotor"), "WHO_AM_I mismatch: expected 0x%02X, got 0x%02X", addr_, who_am_i[0]);
        return false;
    }

    // Additional initialization if needed

    return true;
}

bool DFRobotDCMotor::setRawMotorSpeed(int8_t left, int8_t right) {
    // Clamp values to -127 to 127, use -128 for standby (optional)
    left = std::max(std::min(left, static_cast<int8_t>(127)), static_cast<int8_t>(-127));
    right = std::max(std::min(right, static_cast<int8_t>(127)), static_cast<int8_t>(-127));

    std::vector<uint8_t> data = {
        static_cast<uint8_t>(left),
        static_cast<uint8_t>(right)};
    return writeBytes(CMD_RAW_MOTOR_SPEED, data);
}

bool DFRobotDCMotor::getEncoderTicks(int16_t &left, int16_t &right) {
    std::vector<uint8_t> buf(4);
    if (!readBytes(CMD_ENCODER_TICKS, 4, buf)) {
        RCLCPP_ERROR(rclcpp::get_logger("DFRobotDCMotor"), "Failed to read encoder ticks");
        return false;
    }

    // Unpack little-endian signed 16-bit integers
    left = static_cast<int16_t>(buf[0] | (buf[1] << 8));
    right = static_cast<int16_t>(buf[2] | (buf[3] << 8));

    return true;
}

bool DFRobotDCMotor::writeBytes(uint8_t reg, const std::vector<uint8_t> &buf) {
    std::vector<uint8_t> data;
    data.push_back(reg);
    data.insert(data.end(), buf.begin(), buf.end());
    ssize_t bytes_written = write(file_i2c_, data.data(), data.size());
    return (bytes_written == static_cast<ssize_t>(data.size()));
}

bool DFRobotDCMotor::readBytes(uint8_t reg, uint8_t len, std::vector<uint8_t> &buf) {
    if (write(file_i2c_, &reg, 1) != 1) {
        // Failed to write register address
        return false;
    }
    buf.resize(len);
    ssize_t bytes_read = read(file_i2c_, buf.data(), len);
    return (bytes_read == len);
}
