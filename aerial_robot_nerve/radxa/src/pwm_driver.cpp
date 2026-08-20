#include "radxa/pwm_driver.h"

#include <algorithm>
#include <iostream>

extern "C" {
#include <pwm.h>
}

namespace radxa
{

  struct PwmDriver::Impl
  {
    int chip;
    int channel;
    double frequency_hz;
    double period_us;
    pwm_t* pwm;

    Impl(int c, int ch, double freq)
      : chip(c),
	channel(ch),
	frequency_hz(freq),
	period_us(1000000.0 / freq),
	pwm(nullptr)
    {
    }
  };

  PwmDriver::PwmDriver(int chip, int channel, double frequency_hz)
    : impl_(std::make_unique<Impl>(chip, channel, frequency_hz))
  { 
  }

  PwmDriver::~PwmDriver()
  {
    close();
  }

  bool PwmDriver::open()
  {
    if (impl_->pwm) {
      return true;
    }

    impl_->pwm = pwm_new();
    if (!impl_->pwm) {
      std::cerr << "pwm_new failed" << std::endl;
      return false;
    }

    if (pwm_open(impl_->pwm, impl_->chip, impl_->channel) < 0) {
      std::cerr << "pwm_open failed: " << pwm_errmsg(impl_->pwm) << std::endl;
      pwm_free(impl_->pwm);
      impl_->pwm = nullptr;
      return false;
    }

    // A previous process may have left the channel enabled. Linux PWM does
    // not permit changing the period on every driver while enabled.
    pwm_disable(impl_->pwm);

    // PWM sysfs attributes survive process restarts.  If a previous program
    // left the channel inverted, a larger servo command produces a shorter
    // high pulse and every servo moves in the opposite direction.  Always
    // restore the active-high convention used by Spinal's ExtraServo output.
    if (pwm_set_polarity(impl_->pwm, PWM_POLARITY_NORMAL) < 0) {
      std::cerr << "pwm_set_polarity failed: " << pwm_errmsg(impl_->pwm)
                << std::endl;
      close();
      return false;
    }

    if (pwm_set_frequency(impl_->pwm, impl_->frequency_hz) < 0) {
      std::cerr << "pwm_set_frequency failed: " << pwm_errmsg(impl_->pwm) << std::endl;
      close();
      return false;
    }

    return true;
  }

  void PwmDriver::close()
  {
    if (!impl_->pwm) {
      return;
    }

    pwm_disable(impl_->pwm);
    pwm_close(impl_->pwm);
    pwm_free(impl_->pwm);
    impl_->pwm = nullptr;
  }

  bool PwmDriver::enable()
  {
    if (!impl_->pwm) {
      std::cerr << "pwm_enable failed: pwm is not opened" << std::endl;
      return false;
    }

    if (pwm_enable(impl_->pwm) < 0) {
      std::cerr << "pwm_enable failed: " << pwm_errmsg(impl_->pwm) << std::endl;
      return false;
    }

    return true;
  }

  bool PwmDriver::disable()
  {
    if (!impl_->pwm) {
      return false;
    }

    if (pwm_disable(impl_->pwm) < 0) {
      std::cerr << "pwm_disable failed: " << pwm_errmsg(impl_->pwm) << std::endl;
      return false;
    }

    return true;
  }

  bool PwmDriver::setPulseWidthUs(double pulse_us)
  {
    if (!impl_->pwm) {
      std::cerr << "pwm_set_duty_cycle failed: pwm is not opened" << std::endl;
      return false;
    }

    pulse_us = std::clamp(pulse_us, 0.0, impl_->period_us);

    const double duty = pulse_us / impl_->period_us;

    if (pwm_set_duty_cycle(impl_->pwm, duty) < 0) {
      std::cerr << "pwm_set_duty_cycle failed: " << pwm_errmsg(impl_->pwm) << std::endl;
      return false;
    }

    return true;
  }

  bool PwmDriver::setDutyCycle(double duty_cycle)
  {
    if (!impl_->pwm) {
      std::cerr << "pwm_set_duty_cycle failed: pwm is not opened" << std::endl;
      return false;
    }

    duty_cycle = std::clamp(duty_cycle, 0.0, 1.0);
    if (pwm_set_duty_cycle(impl_->pwm, duty_cycle) < 0) {
      std::cerr << "pwm_set_duty_cycle failed: " << pwm_errmsg(impl_->pwm)
                << std::endl;
      return false;
    }
    return true;
  }

  double PwmDriver::frequencyHz() const
  {
    return impl_->frequency_hz;
  }

  double PwmDriver::periodUs() const
  {
    return impl_->period_us;
  }

}
