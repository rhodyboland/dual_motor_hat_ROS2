// dfrobot_dc_motor.hpp

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
    // DFRobotDCMotor();  // Add default constructor
    DFRobotDCMotor(int bus_id, uint8_t addr);
    ~DFRobotDCMotor();

    bool begin();

    void setAddress(uint8_t addr);

    void setEncoderEnable(uint8_t id);
    void setEncoderDisable(uint8_t id);

    void setEncoderReductionRatio(uint8_t id, uint16_t reduction_ratio);

    int16_t getEncoderSpeed(uint8_t id);

    void setMotorPWMFrequency(uint16_t frequency);

    void motorMovement(uint8_t id, uint8_t orientation, float speed);

    void motorStop(uint8_t id);

    static const uint8_t M1 = 0x01;
    static const uint8_t M2 = 0x02;
    static const uint8_t ALL = 0xFF;

    static const uint8_t CW = 0x01;   // clockwise
    static const uint8_t CCW = 0x02;  // counterclockwise
    static const uint8_t STOP = 0x05;

   private:
    int file_i2c_;
    int bus_id_;
    uint8_t addr_;

    bool writeBytes(uint8_t reg, const std::vector<uint8_t> &buf);
    bool readBytes(uint8_t reg, uint8_t len, std::vector<uint8_t> &buf);

    void setControlMode(uint8_t mode);

    // Register addresses and default values
    static const uint8_t _REG_SLAVE_ADDR = 0x00;
    static const uint8_t _REG_PID = 0x01;
    static const uint8_t _REG_PVD = 0x02;
    static const uint8_t _REG_CTRL_MODE = 0x03;
    static const uint8_t _REG_ENCODER1_EN = 0x04;
    static const uint8_t _REG_ENCODER1_SPEED = 0x05;
    static const uint8_t _REG_ENCODER1_REDUCTION_RATIO = 0x07;
    static const uint8_t _REG_ENCODER2_EN = 0x09;
    static const uint8_t _REG_ENCODER2_SPEED = 0x0A;
    static const uint8_t _REG_ENCODER2_REDUCTION_RATIO = 0x0C;
    static const uint8_t _REG_MOTOR_PWM = 0x0E;
    static const uint8_t _REG_MOTOR1_ORIENTATION = 0x0F;
    static const uint8_t _REG_MOTOR1_SPEED = 0x10;
    static const uint8_t _REG_MOTOR2_ORIENTATION = 0x12;
    static const uint8_t _REG_MOTOR2_SPEED = 0x13;

    static const uint8_t _REG_DEF_PID = 0xDF;
    static const uint8_t _REG_DEF_VID = 0x10;

    static const uint8_t _control_mode_dc_motor = 0x00;
    static const uint8_t _control_mode_stepper = 0x01;
};

#endif  // DFROBOT_DC_MOTOR_HPP
