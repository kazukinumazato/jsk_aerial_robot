#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <radxa/dshot_driver.h>

int main(int argc, char** argv)
{
  const std::string device = argc > 1 ? argv[1] : "/dev/spidev1.0";
  const int duration_ms = argc > 2 ? std::strtol(argv[2], nullptr, 0) : 5000;
  const uint32_t speed_hz = argc > 3
      ? static_cast<uint32_t>(std::strtoul(argv[3], nullptr, 0))
      : 1600000U;
  if (duration_ms < 1000 || duration_ms > 60000) {
    std::cerr << "duration must be in [1000, 60000] ms" << std::endl;
    return 2;
  }

  if (speed_hz != 900000U && speed_hz != 1400000U &&
      (speed_hz < 1500000U || speed_hz > 1600000U) &&
      speed_hz != 2400000U) {
    std::cerr << "speed must be 900000 (packed 3-bit), 1400000 (low-duty "
                 "5-bit), 1500000..1600000 (packed 5-bit), or 2400000 "
                 "(one byte per DShot bit)"
              << std::endl;
    return 2;
  }
  radxa::DshotDriver driver(device, speed_hz);
  if (!driver.open()) {
    return 1;
  }

  std::cout << "sending only DShot stop frames for " << duration_ms << " ms"
            << std::endl;
  const auto end = std::chrono::steady_clock::now() +
                   std::chrono::milliseconds(duration_ms);
  while (std::chrono::steady_clock::now() < end) {
    if (!driver.writeStop()) {
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::cout << "stop-only arming test complete" << std::endl;
  return 0;
}
