#pragma once

#include <string>
#include <cstdint>
#include <vector>
#include <memory>

#include <radxa/board_io.h>
#include <radxa/pwm_driver.h>
#include <radxa/i2c_driver.h>

namespace radxa
{
  class RadxaBoardIo : public radxa::BoardIo
  {
  private:
    static constexpr uint8_t imu = 0x69;
    static constexpr uint8_t imu_who_am_i_reg = 0x00;
    static uint8_t imu_who_am_i;
    static constexpr uint8_t pwr[2] = { 0x06, 0x01 };
    static constexpr uint8_t accel_reg = 0x2D;
    static constexpr uint8_t gyro_reg = 0x33;

    static constexpr double pwm_freq = 500.0;
    struct PwmConfig {
      uint8_t pwm_chip;
      uint8_t pwm_channel;
    };
    const std::vector<PwmConfig> pwm_configs;
    
    std::vector<std::unique_ptr<PwmDriver>> pwm_drivers_;
    radxa::I2cDriver i2c;
  
  public:
    RadxaBoardIo();
    ~RadxaBoardIo() override = default;

    bool getVoltage(float& voltage) override;
    bool readImu(ImuRaw& data) override;
    bool setMotorPwms(const float* pwms, int motor_number) override;
  
  };

}
