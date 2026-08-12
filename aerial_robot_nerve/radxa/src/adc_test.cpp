#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include <radxa/ads1015.h>
#include <radxa/i2c_driver.h>

int main(int argc, char** argv)
{
  const std::string device = argc > 1 ? argv[1] : "/dev/i2c-2";
  const int address = argc > 2 ? std::strtol(argv[2], nullptr, 0) : 0x48;
  const int channel = argc > 3 ? std::strtol(argv[3], nullptr, 0) : 0;
  const float divider_ratio =
      argc > 4 ? std::strtof(argv[4], nullptr) : 11.0F;
  const int count = argc > 5 ? std::strtol(argv[5], nullptr, 0) : 20;

  radxa::I2cDriver i2c(device);
  if (!i2c.open()) {
    return 1;
  }

  radxa::Ads1015::Config config;
  config.address = static_cast<uint8_t>(address);
  config.channel = static_cast<uint8_t>(channel);
  config.divider_ratio = divider_ratio;
  radxa::Ads1015 adc(i2c, config);
  if (!adc.probe()) {
    std::cerr << "ADS1015 probe failed" << std::endl;
    return 1;
  }

  for (int i = 0; count <= 0 || i < count; ++i) {
    float battery_voltage = 0.0F;
    if (!adc.readVoltage(battery_voltage)) {
      return 1;
    }
    std::cout << std::fixed << std::setprecision(4)
              << "battery[V] " << battery_voltage << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}
