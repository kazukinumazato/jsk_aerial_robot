#include <array>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <radxa/ads1015.h>
#include <radxa/dshot_driver.h>
#include <radxa/icm20948.h>
#include <radxa/radxa_board_io.h>

namespace
{
class MagnetometerUnavailableI2c : public radxa::I2cInterface
{
public:
  bool write(uint8_t address, const uint8_t* data, std::size_t length) override
  {
    if (address == 0x69 && length == 2) {
      writes.emplace_back(data[0], data[1]);
    }
    return true;
  }

  bool read(uint8_t, uint8_t*, std::size_t) override { return false; }

  bool writeRead(uint8_t address, const uint8_t* write_data,
                 std::size_t write_length, uint8_t* read_data,
                 std::size_t read_length) override
  {
    if (address == 0x69 && write_length == 1 && read_length == 1 &&
        write_data[0] == 0x00) {
      read_data[0] = 0xea;
      return true;
    }
    return false;
  }

  std::vector<std::pair<uint8_t, uint8_t>> writes;
};
}  // namespace

TEST(Ads1015, DecodesSignedLeftAlignedTwelveBitValues)
{
  EXPECT_EQ(radxa::Ads1015::decodeRaw(0x7f, 0xf0), 2047);
  EXPECT_EQ(radxa::Ads1015::decodeRaw(0x80, 0x00), -2048);
  EXPECT_EQ(radxa::Ads1015::decodeRaw(0xff, 0xf0), -1);
  EXPECT_FLOAT_EQ(radxa::Ads1015::rawToAdcVoltage(1000), 2.0F);
}

TEST(Dshot, GeneratesChecksumAndSpiWaveform)
{
  // DShot checksum: ((48 << 1) xor shifted nibbles) & 0xf = 0x6.
  EXPECT_EQ(radxa::DshotDriver::makePacket(48, false), 0x0606);
  EXPECT_EQ(radxa::DshotDriver::makePacket(248, false), 0x1f0e);
  EXPECT_EQ(radxa::DshotDriver::makePacket(0, false), 0x0000);
  EXPECT_EQ(radxa::DshotDriver::makePacket(7, true), 0x00ff);
  EXPECT_EQ(radxa::DshotDriver::makePacket(8, true), 0x0110);

  const auto zero = radxa::DshotDriver::encodePacket(0x0000);
  for (std::size_t i = 0; i < 16; ++i) {
    EXPECT_EQ(zero[i], 0xe0);
  }
  EXPECT_EQ(zero[16], 0x00);
  EXPECT_EQ(zero[17], 0x00);

  const auto compact3_zero = radxa::DshotDriver::encodePacket3Bit(0x0000);
  const std::array<uint8_t, 7> expected_compact3_zero{{
    0x92, 0x49, 0x24, 0x92, 0x49, 0x24, 0x00}};
  EXPECT_EQ(compact3_zero, expected_compact3_zero);

  const auto compact5_low_duty_zero =
      radxa::DshotDriver::encodePacket5BitLowDuty(0x0000);
  const std::array<uint8_t, 11> expected_compact5_low_duty_zero{{
    0x84, 0x21, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0x00}};
  EXPECT_EQ(compact5_low_duty_zero, expected_compact5_low_duty_zero);

  const auto compact_zero = radxa::DshotDriver::encodePacketCompact(0x0000);
  const std::array<uint8_t, 11> expected_compact_zero{{
    0xc6, 0x31, 0x8c, 0x63, 0x18, 0xc6, 0x31, 0x8c, 0x63, 0x18, 0x00}};
  EXPECT_EQ(compact_zero, expected_compact_zero);

  const auto compact_padded_zero =
      radxa::DshotDriver::encodePacketCompactPadded(0x0000);
  const std::array<uint8_t, 12> expected_compact_padded_zero{{
    0x00, 0xc6, 0x31, 0x8c, 0x63, 0x18, 0xc6, 0x31, 0x8c, 0x63, 0x18, 0x00}};
  EXPECT_EQ(compact_padded_zero, expected_compact_padded_zero);

  const auto one = radxa::DshotDriver::encodePacket(0x8000);
  EXPECT_EQ(one[0], 0xfc);
  EXPECT_EQ(one[1], 0xe0);
}

TEST(Dshot, ConvertsSpinalNormalizedOutputSafely)
{
  EXPECT_EQ(radxa::RadxaBoardIo::normalizedToDshot(1.0F, false), 0);
  EXPECT_EQ(radxa::RadxaBoardIo::normalizedToDshot(0.5F, true), 0);
  EXPECT_EQ(radxa::RadxaBoardIo::normalizedToDshot(0.5001F, true), 48);
  EXPECT_EQ(radxa::RadxaBoardIo::normalizedToDshot(0.55F, true), 248);
  EXPECT_EQ(radxa::RadxaBoardIo::normalizedToDshot(0.60F, true), 448);
  EXPECT_EQ(radxa::RadxaBoardIo::normalizedToDshot(1.0F, true), 2047);
}

TEST(Icm20948, ConvertsConfiguredFullScaleValuesToSi)
{
  // +1 g, -1 g, 0; +1000 dps, -1000 dps, 0; temperature raw 0.
  const std::array<uint8_t, 14> raw{{
      0x10, 0x00, 0xf0, 0x00, 0x00, 0x00,
      0x40, 0x10, 0xbf, 0xf0, 0x00, 0x00,
      0x00, 0x00}};
  radxa::ImuSample sample;
  radxa::Icm20948::convertAccelGyro(raw.data(), sample);
  EXPECT_NEAR(sample.acc_mps2[0], 9.80665, 1e-5);
  EXPECT_NEAR(sample.acc_mps2[1], -9.80665, 1e-5);
  EXPECT_NEAR(sample.gyro_radps[0], 17.4532925, 1e-4);
  EXPECT_NEAR(sample.gyro_radps[1], -17.4532925, 1e-4);
  EXPECT_NEAR(sample.temperature_c, 21.0, 1e-5);
}

TEST(Icm20948, RestoresPrimaryBusWhenMagnetometerBypassFails)
{
  MagnetometerUnavailableI2c i2c;
  radxa::Icm20948::Config config;
  config.enable_magnetometer = true;
  config.require_magnetometer = false;
  radxa::Icm20948 imu(i2c, config);

  ASSERT_TRUE(imu.init());
  ASSERT_FALSE(i2c.writes.empty());
  EXPECT_EQ(i2c.writes.back().first, 0x0f);
  EXPECT_EQ(i2c.writes.back().second, 0x00);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
