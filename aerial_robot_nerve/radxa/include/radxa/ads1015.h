#pragma once

#include <cstdint>

#include <radxa/i2c_driver.h>

namespace radxa
{

class Ads1015
{
public:
  struct Config
  {
    uint8_t address{0x48};
    uint8_t channel{0};
    float divider_ratio{11.0F};
    // Battery volts per raw count. A positive value overrides divider_ratio.
    float adc_scale{0.0F};
  };

  Ads1015(I2cInterface& i2c, Config config);

  bool probe();
  bool readVoltage(float& battery_voltage);
  void setAdcScale(float adc_scale);
  float adcScale() const;

  static int16_t decodeRaw(uint8_t msb, uint8_t lsb);
  static float rawToAdcVoltage(int16_t raw);

private:
  I2cInterface& i2c_;
  Config config_;
};

}  // namespace radxa
