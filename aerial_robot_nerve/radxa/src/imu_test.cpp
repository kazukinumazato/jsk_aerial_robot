#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include <radxa/i2c_driver.h>
#include <radxa/icm20948.h>

int main(int argc, char** argv)
{
  const std::string device = argc > 1 ? argv[1] : "/dev/i2c-7";
  const int address = argc > 2 ? std::strtol(argv[2], nullptr, 0) : 0x69;

  radxa::I2cDriver i2c(device);
  if (!i2c.open()) {
    return 1;
  }
  radxa::Icm20948::Config config;
  config.address = static_cast<uint8_t>(address);
  radxa::Icm20948 imu(i2c, config);
  if (!imu.init()) {
    return 1;
  }

  while (true) {
    radxa::ImuSample sample;
    if (!imu.read(sample)) {
      return 1;
    }
    std::cout << std::fixed << std::setprecision(6)
              << "acc[m/s2] " << sample.acc_mps2[0] << ' '
              << sample.acc_mps2[1] << ' ' << sample.acc_mps2[2]
              << "  gyro[rad/s] " << sample.gyro_radps[0] << ' '
              << sample.gyro_radps[1] << ' ' << sample.gyro_radps[2]
              << "  mag[T] " << sample.mag_tesla[0] << ' '
              << sample.mag_tesla[1] << ' ' << sample.mag_tesla[2]
              << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
