#pragma once

#include <array>
#include <cstddef>

namespace radxa
{

struct ImuSample
{
  std::array<float, 3> acc_mps2{{0.0F, 0.0F, 0.0F}};
  std::array<float, 3> gyro_radps{{0.0F, 0.0F, 0.0F}};
  std::array<float, 3> mag_tesla{{0.0F, 0.0F, 0.0F}};
  float temperature_c{0.0F};
};

class BoardIo
{
public:
  virtual ~BoardIo() = default;

  virtual bool init() = 0;
  virtual bool readImu(ImuSample& sample) = 0;
  virtual bool readBatteryVoltage(float& voltage) = 0;

  // Values use spinal's normalized range (0.5 = minimum, 1.0 = maximum).
  // Outputs 0..3 are DShot300, outputs 4..7 are conventional PWM.
  virtual bool setMotorOutputs(const float* values, std::size_t count,
                               bool dshot_enabled) = 0;
  virtual void stopOutputs() = 0;
};

}  // namespace radxa
