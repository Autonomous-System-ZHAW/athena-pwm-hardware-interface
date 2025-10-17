#ifndef PWM_HAT_CONSTANTS_H
#define PWM_HAT_CONSTANTS_H
#include <cstdint>

namespace PCA9685 {

// Registers/etc:
constexpr uint8_t REG_MODE1              = 0x00;
constexpr uint8_t REG_MODE2              = 0x01;
constexpr uint8_t REG_PRESCALE           = 0xFE;
constexpr uint8_t REG_THROTTLE_OFF_LOW = 0x08;
constexpr uint8_t REG_STEER_OFF_LOW = 0x0c;

// Bits:
constexpr uint8_t RESTART            = 0x80;
constexpr uint8_t SLEEP              = 0x10;

}  // namespace PCA9685

#endif //PWM_HAT_CONSTANTS_H