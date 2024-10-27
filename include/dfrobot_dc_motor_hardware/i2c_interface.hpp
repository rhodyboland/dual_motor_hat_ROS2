#ifndef DFROBOT_DC_MOTOR_I2C_INTERFACE_HPP
#define DFROBOT_DC_MOTOR_I2C_INTERFACE_HPP

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace dfrobot_dc_motor_hardware {

class I2CInterface {
   public:
    I2CInterface(const std::string &device, int address) : address_(address) {
        fd_ = open(device.c_str(), O_RDWR);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open the i2c bus: " +
                                     std::string(strerror(errno)));
        }
        if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
            close(fd_);
            throw std::runtime_error(
                "Failed to acquire bus access and/or talk to slave: " +
                std::string(strerror(errno)));
        }
    }

    ~I2CInterface() {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    void writeBytes(uint8_t reg, const std::vector<uint8_t> &data) {
        std::vector<uint8_t> buffer;
        buffer.push_back(reg);
        buffer.insert(buffer.end(), data.begin(), data.end());
        if (write(fd_, buffer.data(), buffer.size()) !=
            static_cast<ssize_t>(buffer.size())) {
            throw std::runtime_error("Failed to write to the i2c bus: " +
                                     std::string(strerror(errno)));
        }
    }

    std::vector<uint8_t> readBytes(uint8_t reg, size_t length) {
        if (write(fd_, &reg, 1) != 1) {
            throw std::runtime_error("Failed to write register address: " +
                                     std::string(strerror(errno)));
        }
        std::vector<uint8_t> data(length);
        if (read(fd_, data.data(), length) != static_cast<ssize_t>(length)) {
            throw std::runtime_error("Failed to read from the i2c bus: " +
                                     std::string(strerror(errno)));
        }
        return data;
    }

   private:
    int fd_ = -1;
    int address_;
};

}  // namespace dfrobot_dc_motor_hardware

#endif  // DFROBOT_DC_MOTOR_I2C_INTERFACE_HPP
