// hardware/include/dfrobot_dc_motor_hardware/dfrobot_dc_motor.hpp

#ifndef DFROBOT_DC_MOTOR_HPP
#define DFROBOT_DC_MOTOR_HPP

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"

class DFRobotDCMotor {
   public:
    DFRobotDCMotor(int bus_id = 1, uint8_t addr = 0x57);
    ~DFRobotDCMotor();

    bool begin();
    void setAddress(uint8_t addr);

    bool setRawMotorSpeed(int8_t left, int8_t right);
    bool getEncoderTicks(int16_t &left, int16_t &right);

   private:
    int file_i2c_;
    int bus_id_;
    uint8_t addr_;

    bool writeBytes(uint8_t reg, const std::vector<uint8_t> &buf);
    bool readBytes(uint8_t reg, uint8_t len, std::vector<uint8_t> &buf);

    // Commands
    static const uint8_t CMD_FIRMWARE_VERSION = 0x08;
    static const uint8_t CMD_WHO_AM_I = 0x0F;
    static const uint8_t CMD_PWM_FREQUENCY = 0x10;
    static const uint8_t CMD_RAW_MOTOR_SPEED = 0x30;
    static const uint8_t CMD_ENCODER_TICKS = 0x32;
    // ... other command constants as needed
};

#endif  // DFROBOT_DC_MOTOR_HPP
