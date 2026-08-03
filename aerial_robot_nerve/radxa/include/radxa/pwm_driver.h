#pragma once

#include <memory>

namespace radxa
{

  class PwmDriver
  {
  public:
    PwmDriver(int chip, int channel, double frequency_hz);
    ~PwmDriver();

    bool open();
    void close();

    bool enable();
    bool disable();

    bool setPulseWidthUs(double pulse_us);
    bool setDutyCycle(double duty_cycle);

    double frequencyHz() const;
    double periodUs() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
  };

}
