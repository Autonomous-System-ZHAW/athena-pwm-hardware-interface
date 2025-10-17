#include "pwm_hardware_interface/i2c_peripheral.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <thread>
#include <chrono>

extern "C" {
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
}
#include <system_error>

namespace PCA9685 {

I2CPeripheral::I2CPeripheral(const std::string& device, const uint8_t address) {
  open_bus(device);
  connect_to_peripheral(address);
}

I2CPeripheral::~I2CPeripheral() {
  if (bus_fd >= 0) {
    close(bus_fd);
  }
}

void I2CPeripheral::write_register_byte(const uint8_t register_address, const uint8_t value) {
  uint8_t wr_buf[2] = {register_address, value};
  if (write(bus_fd, wr_buf, 2) != 2) {
    const auto msg = "Could not write value (" + std::to_string(value) + ") to register " + std::to_string(register_address);
    throw std::system_error(errno, std::system_category(), msg);
  }
}

uint8_t I2CPeripheral::read_register_byte(const uint8_t register_address) {
  uint8_t value = 0;
  uint8_t wr_buf[1] = {register_address};
  uint8_t rd_buf[1];

  if (write(bus_fd, wr_buf, 1) != 1) {
    const auto msg = "Could not write register address " + std::to_string(register_address);
    throw std::system_error(errno, std::system_category(), msg);
  }
  std::this_thread::sleep_for(std::chrono::microseconds(50));
  if (read(bus_fd, rd_buf, 1) != 1) {
    const auto msg = "Could not read value from register address " + std::to_string(register_address);
    throw std::system_error(errno, std::system_category(), msg);
  } else {
    value = rd_buf[0];
  }
  return value;
}

void I2CPeripheral::open_bus(const std::string& device) {
  bus_fd = open(device.c_str(), O_RDWR);
  if(bus_fd < 0) {
    throw std::system_error(errno, std::system_category(), "Could not open i2c bus.");
  }
}

void I2CPeripheral::connect_to_peripheral(const uint8_t address) {
  if(ioctl(bus_fd, I2C_SLAVE, address) < 0) {
    throw std::system_error(errno, std::system_category(), "Could not set peripheral address.");
  }
}

}  // namespace PCA9685