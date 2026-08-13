#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <radxa/ads1015.h>
#include <radxa/board_io.h>
#include <radxa/dshot_driver.h>
#include <radxa/i2c_driver.h>
#include <radxa/icm20948.h>
#include <radxa/pwm_driver.h>

namespace radxa
{

struct PwmConfig
{
  int chip{0};
  int channel{0};
};

struct RadxaBoardConfig
{
  std::string i2c_device{"/dev/i2c-7"};
  Icm20948::Config imu;
  Ads1015::Config adc;

  // One independent SPI controller/MOSI output is required per DShot channel.
  std::vector<std::string> dshot_spi_devices;
  uint32_t dshot_spi_speed_hz{1600000};
  bool require_all_dshot_channels{false};

  // Physical output ports 5..8. Defaults retain the mapping from the
  // original Cubie A7Z prototype; verify pwmchip numbering after overlays.
  std::array<PwmConfig, 4> pwm{{
      {10, 7}, {20, 1}, {20, 2}, {20, 3}}};
  double pwm_frequency_hz{500.0};
};

class RadxaBoardIo : public BoardIo
{
public:
  explicit RadxaBoardIo(RadxaBoardConfig config);
  ~RadxaBoardIo() override;

  bool init() override;
  bool readImu(ImuSample& sample) override;
  bool readBatteryVoltage(float& voltage) override;
  bool setMotorOutputs(const float* values, std::size_t count,
                       bool dshot_enabled) override;
  void stopOutputs() override;

  void setAdcScale(float adc_scale);
  std::size_t dshotChannelCount() const;

  static uint16_t normalizedToDshot(float value, bool enabled);

private:
  RadxaBoardConfig config_;
  I2cDriver i2c_;
  Icm20948 imu_;
  Ads1015 adc_;
  std::vector<std::unique_ptr<DshotDriver>> dshot_;
  std::array<std::unique_ptr<PwmDriver>, 4> pwm_;
  bool initialized_{false};
};

}  // namespace radxa
