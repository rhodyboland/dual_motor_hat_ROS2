// hardware/include/dfrobot_dc_motor_hardware/wheel.hpp

#ifndef DFROBOT_DC_MOTOR_WHEEL_HPP
#define DFROBOT_DC_MOTOR_WHEEL_HPP

#include <string>

class Wheel {
   public:
    std::string name = "";
    double cmd = 0.0;
    double pos = 0.0;
    double vel = 0.0;

    void setup(const std::string &wheel_name) {
        name = wheel_name;
    }
};

#endif  // DFROBOT_DC_MOTOR_WHEEL_HPP
