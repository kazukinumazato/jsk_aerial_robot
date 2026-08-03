#include <radxa/dshot_driver.h>

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>

namespace radxa
{

struct DshotDriver::Impl
{
  std::string device;
  uint32_t speed_hz;
  int fd{-1};

  Impl(std::string path, uint32_t speed)
    : device(std::move(path)), speed_hz(speed)
  {
  }
};

DshotDriver::DshotDriver(std::string spi_device, uint32_t spi_speed_hz)
  : impl_(std::make_unique<Impl>(std::move(spi_device), spi_speed_hz))
{
}

DshotDriver::~DshotDriver()
{
  close();
}

bool DshotDriver::open()
{
  if (impl_->fd >= 0) {
    return true;
  }
  impl_->fd = ::open(impl_->device.c_str(), O_RDWR | O_CLOEXEC);
  if (impl_->fd < 0) {
    std::cerr << "failed to open SPI device " << impl_->device << ": "
              << std::strerror(errno) << std::endl;
    return false;
  }

  uint8_t mode = SPI_MODE_0;
  uint8_t bits = 8;
  if (::ioctl(impl_->fd, SPI_IOC_WR_MODE, &mode) < 0 ||
      ::ioctl(impl_->fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
      ::ioctl(impl_->fd, SPI_IOC_WR_MAX_SPEED_HZ, &impl_->speed_hz) < 0) {
    std::cerr << "failed to configure SPI device " << impl_->device << ": "
              << std::strerror(errno) << std::endl;
    close();
    return false;
  }
  return true;
}

void DshotDriver::close()
{
  if (impl_->fd < 0) {
    return;
  }
  // Best-effort disarm before releasing the data line.
  writeStop();
  ::close(impl_->fd);
  impl_->fd = -1;
}

bool DshotDriver::isOpen() const
{
  return impl_->fd >= 0;
}

uint16_t DshotDriver::makePacket(uint16_t value, bool telemetry)
{
  value = static_cast<uint16_t>(value > 2047 ? 2047 : value);
  uint16_t packet = static_cast<uint16_t>((value << 1) | (telemetry ? 1 : 0));
  uint16_t checksum_data = packet;
  uint16_t checksum = 0;
  for (int i = 0; i < 3; ++i) {
    checksum ^= checksum_data;
    checksum_data >>= 4;
  }
  return static_cast<uint16_t>((packet << 4) | (checksum & 0x0f));
}

std::array<uint8_t, 18> DshotDriver::encodePacket(uint16_t packet)
{
  std::array<uint8_t, 18> encoded{};
  for (std::size_t bit = 0; bit < 16; ++bit) {
    // At 2.4 MHz, eight SPI clocks form one 3.333 us DShot300 bit.
    // 0: high for 3/8 bit; 1: high for 6/8 bit.
    encoded[bit] = (packet & 0x8000U) != 0 ? 0xfc : 0xe0;
    packet <<= 1;
  }
  // Two zero bytes provide a low inter-frame gap (> 6.6 us).
  encoded[16] = 0x00;
  encoded[17] = 0x00;
  return encoded;
}

bool DshotDriver::writeValue(uint16_t value, bool telemetry)
{
  if (impl_->fd < 0) {
    return false;
  }
  const auto encoded = encodePacket(makePacket(value, telemetry));
  spi_ioc_transfer transfer{};
  transfer.tx_buf = reinterpret_cast<unsigned long>(encoded.data());
  transfer.len = encoded.size();
  transfer.speed_hz = impl_->speed_hz;
  transfer.bits_per_word = 8;
  if (::ioctl(impl_->fd, SPI_IOC_MESSAGE(1), &transfer) < 0) {
    std::cerr << "DShot SPI transfer failed on " << impl_->device << ": "
              << std::strerror(errno) << std::endl;
    return false;
  }
  return true;
}

bool DshotDriver::writeStop()
{
  return writeValue(0, false);
}

}  // namespace radxa
