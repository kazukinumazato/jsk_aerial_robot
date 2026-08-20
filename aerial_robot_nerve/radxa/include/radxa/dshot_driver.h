#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace radxa
{

class DshotDriver
{
public:
  explicit DshotDriver(std::string spi_device, uint32_t spi_speed_hz = 1600000);
  ~DshotDriver();

  bool open();
  void close();
  bool writeValue(uint16_t value, bool telemetry = false);
  bool writeDirectionCommand(bool reversed);
  bool writeStop();
  bool isOpen() const;

  static uint16_t makePacket(uint16_t value, bool telemetry);
  static std::array<uint8_t, 18> encodePacket(uint16_t packet);
  static std::array<uint8_t, 7> encodePacket3Bit(uint16_t packet);
  static std::array<uint8_t, 11> encodePacket5BitLowDuty(uint16_t packet);
  static std::array<uint8_t, 11> encodePacketCompact(uint16_t packet);
  static std::array<uint8_t, 12> encodePacketCompactPadded(uint16_t packet);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace radxa
