// dfrobot_dc_motor.cpp

#include "dfrobot_dc_motor_hardware/dfrobot_dc_motor.hpp"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

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
        std::cerr << "Failed to open the I2C bus\n";
        return false;
    }

    // Set the I2C slave address
    if (ioctl(file_i2c_, I2C_SLAVE, addr_) < 0) {
        // Failed to acquire bus access and/or talk to slave
        std::cerr << "Failed to acquire bus access and/or talk to slave\n";
        return false;
    }

    // Read PID and VID to confirm the device is connected
    std::vector<uint8_t> pid(1);
    if (!readBytes(_REG_PID, 1, pid)) {
        std::cerr << "Failed to read PID\n";
        return false;
    }

    std::vector<uint8_t> vid(1);
    if (!readBytes(_REG_PVD, 1, vid)) {
        std::cerr << "Failed to read VID\n";
        return false;
    }

    if (pid[0] != _REG_DEF_PID) {
        // Device not detected
        std::cerr << "Device not detected\n";
        return false;
    } else {
        // Set control mode, stop motors, disable encoders
        setControlMode(_control_mode_dc_motor);
        motorStop(ALL);
        setEncoderDisable(ALL);
        return true;
    }
}

void DFRobotDCMotor::setControlMode(uint8_t mode) {
    writeBytes(_REG_CTRL_MODE, {mode});
}

void DFRobotDCMotor::setEncoderEnable(uint8_t id) {
    uint8_t reg = _REG_ENCODER1_EN;
    if (id == M2) {
        reg = _REG_ENCODER2_EN;
    }
    writeBytes(reg, {0x01});
}

void DFRobotDCMotor::setEncoderDisable(uint8_t id) {
    uint8_t reg = _REG_ENCODER1_EN;
    if (id == M2) {
        reg = _REG_ENCODER2_EN;
    }
    writeBytes(reg, {0x00});
}

void DFRobotDCMotor::setEncoderReductionRatio(uint8_t id, uint16_t reduction_ratio) {
    uint8_t reg = _REG_ENCODER1_REDUCTION_RATIO;
    if (id == M2) {
        reg = _REG_ENCODER2_REDUCTION_RATIO;
    }
    uint8_t high_byte = (reduction_ratio >> 8) & 0xFF;
    uint8_t low_byte = reduction_ratio & 0xFF;
    writeBytes(reg, {high_byte, low_byte});
}

int16_t DFRobotDCMotor::getEncoderSpeed(uint8_t id) {
    uint8_t reg = _REG_ENCODER1_SPEED;
    if (id == M2) {
        reg = _REG_ENCODER2_SPEED;
    }
    std::vector<uint8_t> buf(2);
    if (!readBytes(reg, 2, buf)) {
        std::cerr << "Failed to read encoder speed\n";
        return 0;
    }
    int16_t speed = (buf[0] << 8) | buf[1];
    if (speed & 0x8000) {
        speed = -(0x10000 - speed);
    }
    return speed;
}

void DFRobotDCMotor::setMotorPWMFrequency(uint16_t frequency) {
    if (frequency < 100 || frequency > 12750) {
        std::cerr << "Frequency out of range\n";
        return;
    }
    frequency = frequency / 50;
    writeBytes(_REG_MOTOR_PWM, {static_cast<uint8_t>(frequency)});
    usleep(100000);  // Sleep for 100ms
}

void DFRobotDCMotor::motorMovement(uint8_t id, uint8_t orientation, float speed) {
    if (orientation != CW && orientation != CCW) {
        std::cerr << "Invalid orientation\n";
        return;
    }
    if (speed < 0.0 || speed > 100.0) {
        std::cerr << "Speed out of range\n";
        return;
    }
    uint8_t reg_orientation = _REG_MOTOR1_ORIENTATION;
    uint8_t reg_speed = _REG_MOTOR1_SPEED;
    if (id == M2) {
        reg_orientation = _REG_MOTOR2_ORIENTATION;
        reg_speed = _REG_MOTOR2_SPEED;
    }
    uint8_t speed_int = static_cast<uint8_t>(speed);
    uint8_t speed_dec = static_cast<uint8_t>((speed - speed_int) * 10);
    writeBytes(reg_orientation, {orientation});
    writeBytes(reg_speed, {speed_int, speed_dec});
}

void DFRobotDCMotor::motorStop(uint8_t id) {
    if (id == ALL) {
        motorStop(M1);
        motorStop(M2);
    } else {
        uint8_t reg_orientation = _REG_MOTOR1_ORIENTATION;
        if (id == M2) {
            reg_orientation = _REG_MOTOR2_ORIENTATION;
        }
        writeBytes(reg_orientation, {STOP});
    }
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
