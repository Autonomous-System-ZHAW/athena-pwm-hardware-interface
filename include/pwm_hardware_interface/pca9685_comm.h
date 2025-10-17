#ifndef PWM_HAT_PCA9685_H
#define PWM_HAT_PCA9685_H

#include <string>
#include <memory>

namespace PCA9685 {

    class I2CPeripheral;

    class PCA9685 {
        public:
            explicit PCA9685(const std::string &device = "/dev/i2c-1", int address = 0x40);
            ~PCA9685();

            void set_pwm_freq(const double freq_hz);

            void set_pwm_ms(const int channel, double ms);

            void set_all_motors_to_middle_position();

        private:
            std::unique_ptr<I2CPeripheral> i2c_dev;

            void set_pwm(const int channel, const uint16_t off_value);

            // Default frequency pulled from PCA9685 datasheet.
            double frequency = 200.0;
            double deviation_max = 0.2;
            double deviation_min = 0.3;
    };

}  // namespace PCA9685

#endif //PWM_HAT_PCA9685_H