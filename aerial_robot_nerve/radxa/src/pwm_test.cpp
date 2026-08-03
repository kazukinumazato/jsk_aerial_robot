#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

#include <radxa/pwm_driver.h>

int main(int argc, char** argv)
{
  if (argc != 5) {
    std::cerr << "usage: pwm_test CHIP CHANNEL FREQUENCY_HZ DUTY_0_TO_1\n";
    return 2;
  }
  const int chip = std::strtol(argv[1], nullptr, 0);
  const int channel = std::strtol(argv[2], nullptr, 0);
  const double frequency = std::strtod(argv[3], nullptr);
  const double duty = std::strtod(argv[4], nullptr);
  if (frequency <= 0.0 || duty < 0.0 || duty > 1.0) {
    std::cerr << "invalid frequency or duty cycle\n";
    return 2;
  }

  radxa::PwmDriver pwm(chip, channel, frequency);
  if (!pwm.open() || !pwm.setDutyCycle(0.0) || !pwm.enable()) {
    return 1;
  }
  std::cout << "Enabling pwmchip" << chip << "/pwm" << channel << " at "
            << frequency << " Hz, duty " << duty
            << " for 3 seconds. Disconnect propellers first." << std::endl;
  if (!pwm.setDutyCycle(duty)) {
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::seconds(3));
  pwm.setDutyCycle(0.0);
  pwm.disable();
  return 0;
}
