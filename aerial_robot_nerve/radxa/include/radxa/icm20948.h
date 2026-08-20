#pragma once

#include <array>
#include <cstdint>

#include <radxa/board_io.h>
#include <radxa/i2c_driver.h>

namespace radxa
{

class Icm20948
{
public:
  struct Config
  {
    uint8_t address{0x69};
    bool enable_magnetometer{false};
    bool require_magnetometer{false};
    std::array<int, 3> axis_map{{0, 1, 2}};
    std::array<float, 3> axis_sign{{1.0F, 1.0F, 1.0F}};
    std::array<int, 3> mag_axis_map{{0, 1, 2}};
    std::array<float, 3> mag_axis_sign{{1.0F, 1.0F, -1.0F}};
    std::array<float, 3> accel_bias{{0.0F, 0.0F, 0.0F}};
    std::array<float, 3> gyro_bias{{0.0F, 0.0F, 0.0F}};
    std::array<float, 3> mag_bias{{0.0F, 0.0F, 0.0F}};
    std::array<float, 3> mag_scale{{1.0F, 1.0F, 1.0F}};
  };

  Icm20948(I2cInterface& i2c, Config config);

  bool init();
  bool read(ImuSample& sample);
  bool magnetometerAvailable() const;

  static void convertAccelGyro(const uint8_t* raw, ImuSample& sample);
  static void applyCalibration(const Config& config, ImuSample& sample);

private:
  bool selectBank(uint8_t bank);
  bool writeRegister(uint8_t bank, uint8_t reg, uint8_t value);
  bool readRegister(uint8_t bank, uint8_t reg, uint8_t& value);
  bool readRegisters(uint8_t bank, uint8_t reg, uint8_t* data,
                     std::size_t length);
  bool initMagnetometer();
  bool disableMagnetometerBypass();
  bool readMagnetometer(std::array<float, 3>& mag_tesla);

  I2cInterface& i2c_;
  Config config_;
  uint8_t selected_bank_{0xff};
  bool magnetometer_available_{false};
  std::array<float, 3> last_mag_tesla_{{0.0F, 0.0F, 0.0F}};
};

}  // namespace radxa
