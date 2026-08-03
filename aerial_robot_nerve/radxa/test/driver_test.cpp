#include <array>
#include <cstdint>

#include <gtest/gtest.h>

#include <radxa/ads1015.h>
#include <radxa/dshot_driver.h>
#include <radxa/icm20948.h>
#include <radxa/radxa_board_io.h>

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
  EXPECT_EQ(radxa::DshotDriver::makePacket(0, false), 0x0000);

  const auto zero = radxa::DshotDriver::encodePacket(0x0000);
  for (std::size_t i = 0; i < 16; ++i) {
    EXPECT_EQ(zero[i], 0xe0);
  }
  EXPECT_EQ(zero[16], 0x00);
  EXPECT_EQ(zero[17], 0x00);

  const auto one = radxa::DshotDriver::encodePacket(0x8000);
  EXPECT_EQ(one[0], 0xfc);
  EXPECT_EQ(one[1], 0xe0);
}

TEST(Dshot, ConvertsSpinalNormalizedOutputSafely)
{
  EXPECT_EQ(radxa::RadxaBoardIo::normalizedToDshot(1.0F, false), 0);
  EXPECT_EQ(radxa::RadxaBoardIo::normalizedToDshot(0.5F, true), 0);
  EXPECT_EQ(radxa::RadxaBoardIo::normalizedToDshot(0.5001F, true), 48);
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
