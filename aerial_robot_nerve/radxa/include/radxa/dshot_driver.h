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
  explicit DshotDriver(std::string spi_device, uint32_t spi_speed_hz = 2400000);
  ~DshotDriver();

  bool open();
  void close();
  bool writeValue(uint16_t value, bool telemetry = false);
  bool writeStop();
  bool isOpen() const;

  static uint16_t makePacket(uint16_t value, bool telemetry);
  static std::array<uint8_t, 18> encodePacket(uint16_t packet);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace radxa
