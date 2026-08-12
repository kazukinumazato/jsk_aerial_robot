#include <radxa/ads1015.h>

#include <chrono>
#include <thread>

namespace radxa
{
namespace
{
constexpr uint8_t kConversionRegister = 0x00;
constexpr uint8_t kConfigRegister = 0x01;
constexpr float kFullScaleVoltage = 4.096F;
constexpr float kCountsPerFullScale = 2048.0F;
}  // namespace

Ads1015::Ads1015(I2cInterface& i2c, Config config)
  : i2c_(i2c), config_(config)
{
}

bool Ads1015::probe()
{
  uint8_t config_data[2]{};
  return i2c_.writeRead(config_.address, &kConfigRegister, 1, config_data,
                        sizeof(config_data));
}

int16_t Ads1015::decodeRaw(uint8_t msb, uint8_t lsb)
{
  const uint16_t word =
      (static_cast<uint16_t>(msb) << 8) | static_cast<uint16_t>(lsb);
  // ADS1015 conversion data is a left-aligned signed 12-bit value. An
  // arithmetic shift of the signed 16-bit word preserves the sign.
  return static_cast<int16_t>(word) >> 4;
}

float Ads1015::rawToAdcVoltage(int16_t raw)
{
  return static_cast<float>(raw) * kFullScaleVoltage / kCountsPerFullScale;
}

bool Ads1015::readVoltage(float& battery_voltage)
{
  if (config_.channel > 3) {
    return false;
  }

  constexpr uint16_t kStartSingle = 0x8000;
  constexpr uint16_t kPga4096 = 0x0200;
  constexpr uint16_t kSingleShot = 0x0100;
  constexpr uint16_t kDataRate1600 = 0x0080;
  constexpr uint16_t kComparatorDisabled = 0x0003;
  const uint16_t mux = static_cast<uint16_t>(0x04 + config_.channel) << 12;
  const uint16_t config = kStartSingle | mux | kPga4096 | kSingleShot |
                          kDataRate1600 | kComparatorDisabled;
  const uint8_t config_data[3] = {
      kConfigRegister,
      static_cast<uint8_t>((config >> 8) & 0xff),
      static_cast<uint8_t>(config & 0xff)};

  if (!i2c_.write(config_.address, config_data, sizeof(config_data))) {
    return false;
  }

  // 1600 SPS takes 0.625 ms. Leave margin for oscillator tolerance.
  std::this_thread::sleep_for(std::chrono::microseconds(900));

  uint8_t data[2]{};
  if (!i2c_.writeRead(config_.address, &kConversionRegister, 1, data,
                      sizeof(data))) {
    return false;
  }

  const int16_t raw = decodeRaw(data[0], data[1]);
  if (config_.adc_scale > 0.0F) {
    battery_voltage = static_cast<float>(raw) * config_.adc_scale;
  } else {
    battery_voltage = rawToAdcVoltage(raw) * config_.divider_ratio;
  }
  return true;
}

void Ads1015::setAdcScale(float adc_scale)
{
  config_.adc_scale = adc_scale;
}

float Ads1015::adcScale() const
{
  return config_.adc_scale;
}

}  // namespace radxa
