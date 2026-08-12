#include <radxa/radxa_board_io.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

namespace radxa
{

RadxaBoardIo::RadxaBoardIo(RadxaBoardConfig config)
  : config_(std::move(config)),
    i2c_(config_.i2c_device),
    imu_(i2c_, config_.imu),
    adc_(i2c_, config_.adc)
{
}

RadxaBoardIo::~RadxaBoardIo()
{
  stopOutputs();
}

bool RadxaBoardIo::init()
{
  if (!i2c_.open()) {
    return false;
  }
  // Probe both devices during startup. Previously the IMU was initialized
  // first, so an IMU error hid a disconnected ADS1015 until much later.
  if (!adc_.probe()) {
    std::cerr << "ADS1015 probe failed" << std::endl;
    return false;
  }
  if (!imu_.init()) {
    return false;
  }

  dshot_.clear();
  const std::size_t dshot_count =
      std::min<std::size_t>(config_.dshot_spi_devices.size(), 4);
  for (std::size_t i = 0; i < dshot_count; ++i) {
    auto driver = std::make_unique<DshotDriver>(
        config_.dshot_spi_devices[i], config_.dshot_spi_speed_hz);
    if (!driver->open()) {
      if (config_.require_all_dshot_channels) {
        return false;
      }
      std::cerr << "DShot output " << (i + 1) << " is disabled" << std::endl;
    }
    dshot_.push_back(std::move(driver));
  }
  if (config_.require_all_dshot_channels && dshot_.size() != 4) {
    std::cerr << "four dshot_spi_devices are required" << std::endl;
    return false;
  }

  for (std::size_t i = 0; i < pwm_.size(); ++i) {
    pwm_[i] = std::make_unique<PwmDriver>(
        config_.pwm[i].chip, config_.pwm[i].channel,
        config_.pwm_frequency_hz);
    if (!pwm_[i]->open() || !pwm_[i]->setDutyCycle(0.0) ||
        !pwm_[i]->enable()) {
      std::cerr << "failed to initialize PWM output " << (i + 5)
                << " (pwmchip" << config_.pwm[i].chip << ", channel "
                << config_.pwm[i].channel << ")" << std::endl;
      return false;
    }
  }

  initialized_ = true;
  stopOutputs();
  return true;
}

bool RadxaBoardIo::readImu(ImuSample& sample)
{
  return initialized_ && imu_.read(sample);
}

bool RadxaBoardIo::readBatteryVoltage(float& voltage)
{
  return initialized_ && adc_.readVoltage(voltage);
}

uint16_t RadxaBoardIo::normalizedToDshot(float value, bool enabled)
{
  if (!enabled || !std::isfinite(value)) {
    return 0;
  }
  // Match spinal's DShot path: IDLE_DUTY itself is a stopped ESC, while a
  // value above it enters the legal throttle range (special commands 1..47
  // are never emitted by this conversion).
  if (value <= 0.5F) {
    return 0;
  }
  value = std::clamp(value, 0.5F, 1.0F);
  constexpr float kMinimumThrottle = 48.0F;
  constexpr float kMaximumThrottle = 2047.0F;
  const float scaled = (value - 0.5F) * 2.0F *
                           (kMaximumThrottle - kMinimumThrottle) +
                       kMinimumThrottle;
  return static_cast<uint16_t>(std::lround(scaled));
}

bool RadxaBoardIo::setMotorOutputs(const float* values, std::size_t count,
                                   bool dshot_enabled)
{
  if (!initialized_ || values == nullptr) {
    return false;
  }
  bool success = true;

  for (std::size_t i = 0; i < dshot_.size(); ++i) {
    const float value = i < count ? values[i] : 0.5F;
    if (dshot_[i]->isOpen() &&
        !dshot_[i]->writeValue(normalizedToDshot(value, dshot_enabled))) {
      success = false;
    }
  }

  for (std::size_t i = 0; i < pwm_.size(); ++i) {
    const std::size_t output_index = i + 4;
    float duty = output_index < count ? values[output_index] : 0.5F;
    if (!std::isfinite(duty)) {
      duty = 0.5F;
      success = false;
    }
    // This matches spinal TIM4: normalized output is the timer duty ratio.
    duty = std::clamp(duty, 0.0F, 1.0F);
    if (!pwm_[i]->setDutyCycle(duty)) {
      success = false;
    }
  }
  return success;
}

void RadxaBoardIo::stopOutputs()
{
  for (auto& driver : dshot_) {
    if (driver && driver->isOpen()) {
      driver->writeStop();
    }
  }
  for (auto& driver : pwm_) {
    if (driver) {
      driver->setDutyCycle(0.0);
    }
  }
}

void RadxaBoardIo::setAdcScale(float adc_scale)
{
  adc_.setAdcScale(adc_scale);
}

std::size_t RadxaBoardIo::dshotChannelCount() const
{
  return dshot_.size();
}

}  // namespace radxa
