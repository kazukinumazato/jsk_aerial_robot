#include <radxa/dshot_driver.h>

#include <algorithm>
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
  uint8_t lsb_first = 0;
  if (::ioctl(impl_->fd, SPI_IOC_WR_MODE, &mode) < 0 ||
      ::ioctl(impl_->fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
      ::ioctl(impl_->fd, SPI_IOC_WR_LSB_FIRST, &lsb_first) < 0 ||
      ::ioctl(impl_->fd, SPI_IOC_WR_MAX_SPEED_HZ, &impl_->speed_hz) < 0) {
    std::cerr << "failed to configure SPI device " << impl_->device << ": "
              << std::strerror(errno) << std::endl;
    close();
    return false;
  }

  uint8_t actual_mode = 0xff;
  uint8_t actual_bits = 0;
  uint8_t actual_lsb_first = 0xff;
  uint32_t actual_speed_hz = 0;
  if (::ioctl(impl_->fd, SPI_IOC_RD_MODE, &actual_mode) < 0 ||
      ::ioctl(impl_->fd, SPI_IOC_RD_BITS_PER_WORD, &actual_bits) < 0 ||
      ::ioctl(impl_->fd, SPI_IOC_RD_LSB_FIRST, &actual_lsb_first) < 0 ||
      ::ioctl(impl_->fd, SPI_IOC_RD_MAX_SPEED_HZ, &actual_speed_hz) < 0) {
    std::cerr << "failed to read back SPI configuration " << impl_->device
              << ": " << std::strerror(errno) << std::endl;
    close();
    return false;
  }
  if (actual_mode != SPI_MODE_0 || actual_bits != 8 || actual_lsb_first != 0 ||
      actual_speed_hz != impl_->speed_hz) {
    std::cerr << "unexpected SPI configuration on " << impl_->device
              << ": mode=" << static_cast<int>(actual_mode)
              << " bits=" << static_cast<int>(actual_bits)
              << " lsb_first=" << static_cast<int>(actual_lsb_first)
              << " speed=" << actual_speed_hz << std::endl;
    close();
    return false;
  }
  std::cout << "DShot SPI ready: " << impl_->device << " mode=0 bits="
            << static_cast<int>(actual_bits) << " MSB-first "
            << actual_speed_hz << " Hz" << std::endl;
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

std::array<uint8_t, 7> DshotDriver::encodePacket3Bit(uint16_t packet)
{
  // Three continuous SPI clocks form one DShot bit. At 900 kHz this gives
  // the exact DShot300 bit period, with 0=100 and 1=110. Both pulse widths
  // remain safely on opposite sides of Bluejay's decode threshold.
  std::array<uint8_t, 7> encoded{};
  std::size_t output_bit = 0;
  for (std::size_t packet_bit = 0; packet_bit < 16; ++packet_bit) {
    const uint8_t symbol = (packet & 0x8000U) != 0 ? 0x06 : 0x04;
    packet <<= 1;
    for (int symbol_bit = 2; symbol_bit >= 0; --symbol_bit, ++output_bit) {
      if ((symbol & (1U << symbol_bit)) != 0) {
        encoded[output_bit / 8] |=
            static_cast<uint8_t>(1U << (7U - output_bit % 8));
      }
    }
  }
  // The seventh byte stays low and supplies an 8.89 us inter-frame gap.
  return encoded;
}

std::array<uint8_t, 11> DshotDriver::encodePacket5BitLowDuty(uint16_t packet)
{
  // Diagnostic DShot300-compatible waveform for Bluejay's decoder. At
  // 1.4 MHz, 0=10000 has a 0.714 us high pulse (above Bluejay's minimum and
  // below its 0/1 threshold), while 1=11110 is unambiguously high.
  std::array<uint8_t, 11> encoded{};
  std::size_t output_bit = 0;
  for (std::size_t packet_bit = 0; packet_bit < 16; ++packet_bit) {
    const uint8_t symbol = (packet & 0x8000U) != 0 ? 0x1e : 0x10;
    packet <<= 1;
    for (int symbol_bit = 4; symbol_bit >= 0; --symbol_bit, ++output_bit) {
      if ((symbol & (1U << symbol_bit)) != 0) {
        encoded[output_bit / 8] |=
            static_cast<uint8_t>(1U << (7U - output_bit % 8));
      }
    }
  }
  return encoded;
}

std::array<uint8_t, 11> DshotDriver::encodePacketCompact(uint16_t packet)
{
  // Five continuous SPI clocks form one DShot bit. At 1.6 MHz, 0=11000 and
  // 1=11110 produce the nominal DShot300 high times (1.25 us and 2.50 us)
  // with a 3.125 us bit period accepted by Bluejay's DShot300 timing window.
  // Packing avoids a controller-specific pause at every logical DShot bit.
  std::array<uint8_t, 11> encoded{};
  std::size_t output_bit = 0;
  for (std::size_t packet_bit = 0; packet_bit < 16; ++packet_bit) {
    const uint8_t symbol = (packet & 0x8000U) != 0 ? 0x1e : 0x18;
    packet <<= 1;
    for (int symbol_bit = 4; symbol_bit >= 0; --symbol_bit, ++output_bit) {
      if ((symbol & (1U << symbol_bit)) != 0) {
        encoded[output_bit / 8] |=
            static_cast<uint8_t>(1U << (7U - output_bit % 8));
      }
    }
  }
  // The eleventh byte stays low and supplies a 5.0 us inter-frame gap; the
  // trailing low clocks of the last symbol extend the total gap further.
  return encoded;
}

std::array<uint8_t, 12> DshotDriver::encodePacketCompactPadded(uint16_t packet)
{
  // Start the SPI transfer low. Without this preamble, MOSI can be driven to
  // the first (high) payload bit before SCLK starts, stretching only the first
  // DShot pulse and corrupting an otherwise valid stop frame.
  std::array<uint8_t, 12> encoded{};
  const auto compact = encodePacketCompact(packet);
  std::copy(compact.begin(), compact.end(), encoded.begin() + 1);
  return encoded;
}

bool DshotDriver::writeValue(uint16_t value, bool telemetry)
{
  if (impl_->fd < 0) {
    return false;
  }
  const uint16_t packet = makePacket(value, telemetry);
  spi_ioc_transfer transfer{};
  const auto compact3 = encodePacket3Bit(packet);
  const auto compact5_low_duty = encodePacket5BitLowDuty(packet);
  const auto compact_padded = encodePacketCompactPadded(packet);
  const auto legacy = encodePacket(packet);
  if (impl_->speed_hz == 900000) {
    transfer.tx_buf = reinterpret_cast<unsigned long>(compact3.data());
    transfer.len = compact3.size();
  } else if (impl_->speed_hz == 1400000) {
    transfer.tx_buf =
        reinterpret_cast<unsigned long>(compact5_low_duty.data());
    transfer.len = compact5_low_duty.size();
  } else if (impl_->speed_hz >= 1500000 && impl_->speed_hz <= 1600000) {
    transfer.tx_buf = reinterpret_cast<unsigned long>(compact_padded.data());
    transfer.len = compact_padded.size();
  } else {
    transfer.tx_buf = reinterpret_cast<unsigned long>(legacy.data());
    transfer.len = legacy.size();
  }
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

bool DshotDriver::writeDirectionCommand(bool reversed)
{
  // Bluejay 0.19.2 commands 7/8 select absolute normal/reversed direction.
  // Special DShot commands are valid only with the telemetry bit set and the
  // caller must repeat them; RadxaBoardIo handles that sequence.
  constexpr uint16_t kDirectionNormal = 7;
  constexpr uint16_t kDirectionReversed = 8;
  return writeValue(reversed ? kDirectionReversed : kDirectionNormal, true);
}

}  // namespace radxa
