// wheel.hpp

#ifndef DFROBOT_DC_MOTOR_WHEEL_HPP
#define DFROBOT_DC_MOTOR_WHEEL_HPP

#include <string>

class Wheel {
   public:
    std::string name = "";
    double cmd = 0.0;
    double pos = 0.0;
    double vel = 0.0;
    int reduction_ratio = 0;

    void setup(const std::string &wheel_name, int reduction_ratio_) {
        name = wheel_name;
        reduction_ratio = reduction_ratio_;
    }
};

#endif  // DFROBOT_DC_MOTOR_WHEEL_HPP
