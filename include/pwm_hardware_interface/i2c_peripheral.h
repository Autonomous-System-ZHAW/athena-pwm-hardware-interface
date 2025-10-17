#ifndef I2CPERIPHERAL_H
#define I2CPERIPHERAL_H

#include <cstdint>
#include <string>

namespace PCA9685 {

  class I2CPeripheral {
    public:
      I2CPeripheral(const std::string& device, const uint8_t address);
      ~I2CPeripheral();

      void write_register_byte(const uint8_t register_address, const uint8_t value);

      uint8_t read_register_byte(const uint8_t register_address);

    private:
      int bus_fd;

      void open_bus(const std::string& device);
      void connect_to_peripheral(const uint8_t address);

  };

}  // namespace PCA9685

#endif  // I2CPERIPHERAL_H