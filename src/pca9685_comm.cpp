#include "pwm_hardware_interface/pca9685_comm.h"
#include <unistd.h>
#include <cmath>
#include <thread>
#include <chrono>
#include <algorithm>

#include "pwm_hardware_interface/constants.h"
#include "pwm_hardware_interface/i2c_peripheral.h"

namespace PCA9685 {

PCA9685::PCA9685(const std::string &device, int address) {
  i2c_dev = std::make_unique<I2CPeripheral>(device, address);

  set_pwm_freq(50.0);
  set_all_motors_to_middle_position();
  std::this_thread::sleep_for(std::chrono::microseconds(5000));

  // activate clock
  auto mode1_val = i2c_dev->read_register_byte(REG_MODE1);
  mode1_val &= ~SLEEP;
  i2c_dev->write_register_byte(REG_MODE1, mode1_val);
  std::this_thread::sleep_for(std::chrono::microseconds(5000));
}

PCA9685::~PCA9685() = default;

void PCA9685::set_pwm_freq(const double freq_hz) {
  frequency = freq_hz;

  auto prescaleval = 2.5e7; //    # 25MHz internal clock
  prescaleval /= 4096.0; //       # 12-bit resolution
  prescaleval /= freq_hz;
  prescaleval -= 1.0;

  auto prescale = static_cast<int>(std::round(prescaleval));

  const auto oldmode = i2c_dev->read_register_byte(REG_MODE1);

  auto newmode = (oldmode & 0x7F) | SLEEP; // set bits for sleep without restart

  i2c_dev->write_register_byte(REG_MODE1, newmode); // set to sleep without restart
  i2c_dev->write_register_byte(REG_PRESCALE, prescale); // reconfigure clock
  i2c_dev->write_register_byte(REG_MODE1, oldmode); // reset to old mode
  std::this_thread::sleep_for(std::chrono::microseconds(5000)); // wait
  i2c_dev->write_register_byte(REG_MODE1, oldmode | RESTART); // restart
}

void PCA9685::set_pwm(const int channel, const uint16_t off_value)
{
  uint8_t register_address = REG_THROTTLE_OFF_LOW;
  if (channel == 1)
  {
    register_address = REG_STEER_OFF_LOW;
  }
  i2c_dev->write_register_byte(register_address, off_value & 0xFF);
  i2c_dev->write_register_byte((register_address + 0x01), (off_value >> 8));
}

void PCA9685::set_pwm_ms(const int channel, double ms)
{
  ms = std::clamp(ms, 1.5 - deviation_min, 1.5 + deviation_max);
  auto period_ms = 1000.0 / frequency;
  auto bits_per_ms = 4096 / period_ms;
  auto bits = ms * bits_per_ms;
  set_pwm(channel, bits);
}

void PCA9685::set_all_motors_to_middle_position()
{
  set_pwm_ms(REG_THROTTLE_OFF_LOW, 1.5);
  set_pwm_ms(REG_STEER_OFF_LOW, 1.45);
}

}  // namespace PCA9685