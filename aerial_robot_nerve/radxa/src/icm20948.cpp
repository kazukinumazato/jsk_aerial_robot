#include <radxa/icm20948.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace radxa
{
namespace
{
constexpr uint8_t kBank0 = 0;
constexpr uint8_t kBank2 = 2;
constexpr uint8_t kBankSelect = 0x7f;
constexpr uint8_t kWhoAmI = 0x00;
constexpr uint8_t kExpectedWhoAmI = 0xea;
constexpr uint8_t kUserControl = 0x03;
constexpr uint8_t kPowerManagement1 = 0x06;
constexpr uint8_t kPowerManagement2 = 0x07;
constexpr uint8_t kInterruptPinConfig = 0x0f;
constexpr uint8_t kAccelXoutH = 0x2d;

constexpr uint8_t kGyroSampleRateDivider = 0x00;
constexpr uint8_t kGyroConfig1 = 0x01;
constexpr uint8_t kAccelSampleRateDivider1 = 0x10;
constexpr uint8_t kAccelSampleRateDivider2 = 0x11;
constexpr uint8_t kAccelConfig = 0x14;

constexpr uint8_t kAk09916Address = 0x0c;
constexpr uint8_t kAk09916WhoAmI = 0x01;
constexpr uint8_t kAk09916ExpectedWhoAmI = 0x09;
constexpr uint8_t kAk09916Status1 = 0x10;
constexpr uint8_t kAk09916Data = 0x11;
constexpr uint8_t kAk09916Control2 = 0x31;
constexpr uint8_t kAk09916Control3 = 0x32;

constexpr float kGravity = 9.80665F;
constexpr float kAccelCountsPerG = 4096.0F;       // +/-8 g
constexpr float kGyroCountsPerDps = 16.4F;        // +/-2000 deg/s
constexpr float kDegreesToRadians = 0.01745329251994329577F;
constexpr float kMagTeslaPerCount = 0.15e-6F;     // AK09916: 0.15 uT/LSB

int16_t readBigEndian(const uint8_t* data)
{
  return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) |
                              static_cast<uint16_t>(data[1]));
}

int16_t readLittleEndian(const uint8_t* data)
{
  return static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) |
                              static_cast<uint16_t>(data[0]));
}
}  // namespace

Icm20948::Icm20948(I2cInterface& i2c, Config config)
  : i2c_(i2c), config_(config)
{
}

bool Icm20948::selectBank(uint8_t bank)
{
  if (bank > 3) {
    return false;
  }
  if (selected_bank_ == bank) {
    return true;
  }
  const uint8_t data[2] = {kBankSelect, static_cast<uint8_t>(bank << 4)};
  if (!i2c_.write(config_.address, data, sizeof(data))) {
    selected_bank_ = 0xff;
    return false;
  }
  selected_bank_ = bank;
  return true;
}

bool Icm20948::writeRegister(uint8_t bank, uint8_t reg, uint8_t value)
{
  if (!selectBank(bank)) {
    return false;
  }
  const uint8_t data[2] = {reg, value};
  return i2c_.write(config_.address, data, sizeof(data));
}

bool Icm20948::readRegister(uint8_t bank, uint8_t reg, uint8_t& value)
{
  return readRegisters(bank, reg, &value, 1);
}

bool Icm20948::readRegisters(uint8_t bank, uint8_t reg, uint8_t* data,
                             std::size_t length)
{
  if (!selectBank(bank)) {
    return false;
  }
  return i2c_.writeRead(config_.address, &reg, 1, data, length);
}

bool Icm20948::init()
{
  uint8_t who_am_i = 0;
  if (!readRegister(kBank0, kWhoAmI, who_am_i) ||
      who_am_i != kExpectedWhoAmI) {
    std::cerr << "ICM-20948 WHO_AM_I mismatch: expected 0xea, got 0x"
              << std::hex << static_cast<int>(who_am_i) << std::dec << std::endl;
    return false;
  }

  if (!writeRegister(kBank0, kPowerManagement1, 0x80)) {
    return false;
  }
  selected_bank_ = 0xff;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Auto-select the best available clock and enable every accel/gyro axis.
  if (!writeRegister(kBank0, kPowerManagement1, 0x01) ||
      !writeRegister(kBank0, kPowerManagement2, 0x00)) {
    return false;
  }

  // Match spinal: gyro +/-2000 dps, accel +/-8 g, with DLPFs enabled.
  // Gyro DLPFCFG=3 and accel DLPFCFG=6; both sample-rate divisors are zero.
  if (!writeRegister(kBank2, kGyroSampleRateDivider, 0x00) ||
      !writeRegister(kBank2, kGyroConfig1, 0x1f) ||
      !writeRegister(kBank2, kAccelSampleRateDivider1, 0x00) ||
      !writeRegister(kBank2, kAccelSampleRateDivider2, 0x00) ||
      !writeRegister(kBank2, kAccelConfig, 0x35)) {
    return false;
  }
  // Do not feed the all-zero power-up sample into the attitude estimator.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  magnetometer_available_ = false;
  if (config_.enable_magnetometer || config_.require_magnetometer) {
    magnetometer_available_ = initMagnetometer();
    if (!magnetometer_available_) {
      std::cerr << "AK09916 magnetometer is unavailable; continuing with 6-axis IMU"
                << std::endl;
    }
  }
  return magnetometer_available_ || !config_.require_magnetometer;
}

bool Icm20948::initMagnetometer()
{
  // Disable the ICM's auxiliary master, then expose its internal AK09916 on
  // the primary I2C bus through bypass mode.
  if (!writeRegister(kBank0, kUserControl, 0x00)) {
    return false;
  }
  if (!writeRegister(kBank0, kInterruptPinConfig, 0x02)) {
    disableMagnetometerBypass();
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  uint8_t who_am_i = 0;
  if (!i2c_.writeRead(kAk09916Address, &kAk09916WhoAmI, 1, &who_am_i, 1) ||
      who_am_i != kAk09916ExpectedWhoAmI) {
    // Some ICM-20948 breakout boards do not expose the internal AK09916 in
    // bypass mode. Leaving BYPASS_EN set after this failure makes subsequent
    // accel/gyro reads NACK on the Cubie A7Z TWI controller.
    disableMagnetometerBypass();
    return false;
  }

  const uint8_t reset[2] = {kAk09916Control3, 0x01};
  if (!i2c_.write(kAk09916Address, reset, sizeof(reset))) {
    disableMagnetometerBypass();
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  // Continuous measurement mode 4: 100 Hz.
  const uint8_t mode[2] = {kAk09916Control2, 0x08};
  if (!i2c_.write(kAk09916Address, mode, sizeof(mode))) {
    disableMagnetometerBypass();
    return false;
  }
  return true;
}

bool Icm20948::disableMagnetometerBypass()
{
  if (!writeRegister(kBank0, kInterruptPinConfig, 0x00)) {
    std::cerr << "failed to disable ICM-20948 magnetometer bypass" << std::endl;
    return false;
  }
  return true;
}

void Icm20948::convertAccelGyro(const uint8_t* raw, ImuSample& sample)
{
  for (std::size_t i = 0; i < 3; ++i) {
    sample.acc_mps2[i] = static_cast<float>(readBigEndian(raw + i * 2)) *
                          kGravity / kAccelCountsPerG;
    sample.gyro_radps[i] =
        static_cast<float>(readBigEndian(raw + 6 + i * 2)) *
        kDegreesToRadians / kGyroCountsPerDps;
  }
  const int16_t raw_temperature = readBigEndian(raw + 12);
  sample.temperature_c = static_cast<float>(raw_temperature) / 333.87F + 21.0F;
}

bool Icm20948::readMagnetometer(std::array<float, 3>& mag_tesla)
{
  uint8_t status = 0;
  if (!i2c_.writeRead(kAk09916Address, &kAk09916Status1, 1, &status, 1)) {
    return false;
  }
  if ((status & 0x01) == 0) {
    mag_tesla = last_mag_tesla_;
    return true;
  }

  // Reading through ST2 is required to release the AK09916 data latch.
  uint8_t data[8]{};
  if (!i2c_.writeRead(kAk09916Address, &kAk09916Data, 1, data,
                      sizeof(data))) {
    return false;
  }
  if ((data[7] & 0x08) != 0) {
    return false;
  }
  for (std::size_t i = 0; i < 3; ++i) {
    last_mag_tesla_[i] =
        static_cast<float>(readLittleEndian(data + i * 2)) * kMagTeslaPerCount;
  }
  mag_tesla = last_mag_tesla_;
  return true;
}

void Icm20948::applyCalibration(ImuSample& sample) const
{
  const auto raw_acc = sample.acc_mps2;
  const auto raw_gyro = sample.gyro_radps;
  const auto raw_mag = sample.mag_tesla;
  for (std::size_t i = 0; i < 3; ++i) {
    const int source = std::clamp(config_.axis_map[i], 0, 2);
    const float sign = config_.axis_sign[i] < 0.0F ? -1.0F : 1.0F;
    sample.acc_mps2[i] = sign * raw_acc[source] - config_.accel_bias[i];
    sample.gyro_radps[i] = sign * raw_gyro[source] - config_.gyro_bias[i];
    sample.mag_tesla[i] =
        (sign * raw_mag[source] - config_.mag_bias[i]) * config_.mag_scale[i];
  }
}

bool Icm20948::read(ImuSample& sample)
{
  // Accel, gyro, and temperature are contiguous in user bank 0.
  uint8_t raw[14]{};
  if (!readRegisters(kBank0, kAccelXoutH, raw, sizeof(raw))) {
    return false;
  }
  convertAccelGyro(raw, sample);

  if (magnetometer_available_) {
    if (!readMagnetometer(sample.mag_tesla)) {
      // A temporary magnetometer error must not stop gyro/accel control.
      sample.mag_tesla = last_mag_tesla_;
    }
  }
  applyCalibration(sample);
  return true;
}

bool Icm20948::magnetometerAvailable() const
{
  return magnetometer_available_;
}

}  // namespace radxa
