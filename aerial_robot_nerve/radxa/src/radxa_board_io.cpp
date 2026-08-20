#include <radxa/radxa_board_io.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>
#include <utility>

namespace radxa
{
namespace
{
constexpr std::size_t kDirectionCommandFrames = 10;
constexpr auto kDirectionStartupStop = std::chrono::milliseconds(500);
constexpr auto kDirectionPreArmStop = std::chrono::milliseconds(1000);
constexpr auto kDirectionPostCommandStop = std::chrono::milliseconds(100);
constexpr auto kDirectionRefreshPeriod = std::chrono::seconds(2);

enum class DshotFrameMode
{
  Stop,
  Direction,
  Throttle
};
}  // namespace

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
    std::cout << "DShot output " << (i + 1) << " requested direction: "
              << (config_.dshot_reversed[i] ? "reversed" : "normal")
              << std::endl;
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
  direction_state_ = DshotDirectionState::Disarmed;
  direction_deadline_ = std::chrono::steady_clock::now() +
                        kDirectionStartupStop;
  direction_command_frames_sent_ = 0;
  direction_command_for_arm_ = false;
  last_dshot_enabled_ = false;
  // Leave the PWM channels enabled for the first servo update. stopOutputs()
  // disables them and is reserved for shutdown; only DShot needs an explicit
  // stop frame here.
  stopDshotOutputs();
  return true;
}

bool RadxaBoardIo::readImu(ImuSample& sample)
{
  return initialized_ && imu_.read(sample);
}

bool RadxaBoardIo::zeroImuGyro(double duration_sec, double sample_rate_hz,
                               std::array<float, 3>& applied_bias)
{
  if (!initialized_ || !std::isfinite(duration_sec) ||
      !std::isfinite(sample_rate_hz) || duration_sec <= 0.0 ||
      sample_rate_hz <= 0.0) {
    return false;
  }

  const auto sample_count = static_cast<std::size_t>(
      std::max(1.0, std::round(duration_sec * sample_rate_hz)));
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / sample_rate_hz));
  std::array<double, 3> residual_sum{{0.0, 0.0, 0.0}};
  auto next_sample = std::chrono::steady_clock::now();

  for (std::size_t sample_index = 0; sample_index < sample_count;
       ++sample_index) {
    ImuSample sample;
    if (!imu_.read(sample)) {
      return false;
    }
    for (std::size_t axis = 0; axis < residual_sum.size(); ++axis) {
      residual_sum[axis] += sample.gyro_radps[axis];
    }
    next_sample += period;
    if (sample_index + 1 < sample_count) {
      std::this_thread::sleep_until(next_sample);
    }
  }

  std::array<float, 3> residual_bias{};
  for (std::size_t axis = 0; axis < residual_bias.size(); ++axis) {
    residual_bias[axis] = static_cast<float>(
        residual_sum[axis] / static_cast<double>(sample_count));
  }
  // Samples above already have the YAML bias subtracted. Adding their mean
  // therefore preserves any configured bias and removes the remaining offset.
  imu_.addGyroBias(residual_bias);
  applied_bias = imu_.gyroBias();
  return true;
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

  const auto now = std::chrono::steady_clock::now();
  if (dshot_enabled && !last_dshot_enabled_) {
    // Never pass throttle on an arm edge until every ESC has had enough stop
    // frames to arm and has received a fresh direction command burst.
    direction_state_ = DshotDirectionState::PreArmStop;
    direction_deadline_ = now + kDirectionPreArmStop;
    direction_command_frames_sent_ = 0;
    direction_command_for_arm_ = true;
    std::cout << "DShot arm gate: holding stop before direction commands"
              << std::endl;
  } else if (!dshot_enabled && last_dshot_enabled_) {
    direction_state_ = DshotDirectionState::Disarmed;
    direction_deadline_ = now + kDirectionStartupStop;
    direction_command_frames_sent_ = 0;
    direction_command_for_arm_ = false;
  }
  last_dshot_enabled_ = dshot_enabled;

  DshotFrameMode dshot_mode = DshotFrameMode::Stop;
  if (dshot_enabled) {
    if (direction_state_ == DshotDirectionState::PreArmStop &&
        now >= direction_deadline_) {
      direction_state_ = DshotDirectionState::SendCommand;
      direction_command_frames_sent_ = 0;
      direction_command_for_arm_ = true;
    }
    if (direction_state_ == DshotDirectionState::SendCommand &&
        direction_command_for_arm_) {
      dshot_mode = DshotFrameMode::Direction;
    } else if (direction_state_ == DshotDirectionState::PostCommandStop) {
      if (now >= direction_deadline_) {
        direction_state_ = DshotDirectionState::Ready;
        dshot_mode = DshotFrameMode::Throttle;
        std::cout << "DShot direction command burst sent; throttle enabled"
                  << std::endl;
      }
    } else if (direction_state_ == DshotDirectionState::Ready) {
      dshot_mode = DshotFrameMode::Throttle;
    }
  } else {
    if (direction_state_ != DshotDirectionState::Disarmed &&
        !(direction_state_ == DshotDirectionState::SendCommand &&
          !direction_command_for_arm_)) {
      direction_state_ = DshotDirectionState::Disarmed;
      direction_deadline_ = now + kDirectionStartupStop;
      direction_command_frames_sent_ = 0;
      direction_command_for_arm_ = false;
    }
    if (direction_state_ == DshotDirectionState::Disarmed &&
        now >= direction_deadline_) {
      direction_state_ = DshotDirectionState::SendCommand;
      direction_command_frames_sent_ = 0;
      direction_command_for_arm_ = false;
    }
    if (direction_state_ == DshotDirectionState::SendCommand &&
        !direction_command_for_arm_) {
      dshot_mode = DshotFrameMode::Direction;
    }
  }

  for (std::size_t i = 0; i < dshot_.size(); ++i) {
    const float value = i < count ? values[i] : 0.5F;
    if (dshot_[i]->isOpen()) {
      bool written = false;
      if (dshot_mode == DshotFrameMode::Direction) {
        written = dshot_[i]->writeDirectionCommand(config_.dshot_reversed[i]);
      } else if (dshot_mode == DshotFrameMode::Throttle) {
        written = dshot_[i]->writeValue(normalizedToDshot(value, true), false);
      } else {
        written = dshot_[i]->writeStop();
      }
      if (!written) {
        success = false;
      }
    }
  }

  if (success && dshot_mode == DshotFrameMode::Direction) {
    ++direction_command_frames_sent_;
    if (direction_command_frames_sent_ >= kDirectionCommandFrames) {
      direction_command_frames_sent_ = 0;
      if (direction_command_for_arm_) {
        direction_state_ = DshotDirectionState::PostCommandStop;
        direction_deadline_ = now + kDirectionPostCommandStop;
      } else {
        direction_state_ = DshotDirectionState::Disarmed;
        direction_deadline_ = now + kDirectionRefreshPeriod;
      }
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

void RadxaBoardIo::stopDshotOutputs()
{
  for (auto& driver : dshot_) {
    if (driver && driver->isOpen()) {
      driver->writeStop();
    }
  }
}

void RadxaBoardIo::stopOutputs()
{
  stopDshotOutputs();
  for (auto& driver : pwm_) {
    if (driver) {
      // Do not command 0 degrees during shutdown. Keep the last duty value in
      // the PWM controller and stop the waveform directly so the servo never
      // sees a transient zero-duty command.
      driver->disable();
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
