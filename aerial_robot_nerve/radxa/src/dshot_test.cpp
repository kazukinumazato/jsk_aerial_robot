#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <radxa/dshot_driver.h>
#include <radxa/radxa_board_io.h>

namespace
{
std::atomic<bool> stop_requested{false};

void requestStop(int)
{
  stop_requested.store(true);
}

bool sendFor(radxa::DshotDriver& driver, uint16_t value,
             std::chrono::milliseconds duration, bool telemetry = false)
{
  const auto end = std::chrono::steady_clock::now() + duration;
  while (!stop_requested.load() && std::chrono::steady_clock::now() < end) {
    if (!driver.writeValue(value, telemetry)) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}
}  // namespace

int main(int argc, char** argv)
{
  const std::string device = argc > 1 ? argv[1] : "/dev/spidev1.0";
  const float normalized = argc > 2 ? std::strtof(argv[2], nullptr) : 0.55F;
  const int run_ms = argc > 3 ? std::strtol(argv[3], nullptr, 0) : 1000;
  const int arm_ms = argc > 4 ? std::strtol(argv[4], nullptr, 0) : 1000;
  const bool telemetry = argc > 5 && std::strtol(argv[5], nullptr, 0) != 0;

  // This executable is intentionally limited to a low bench-test command.
  if (!std::isfinite(normalized) || normalized <= 0.5F || normalized > 0.60F ||
      run_ms <= 0 || run_ms > 2000 || arm_ms < 1000 || arm_ms > 30000) {
    std::cerr << "normalized must be in (0.5, 0.60], duration in [1, 2000] ms, "
                 "and arming in [1000, 30000] ms"
              << std::endl;
    return 2;
  }

  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);

  radxa::DshotDriver driver(device, 1600000);
  if (!driver.open()) {
    return 1;
  }
  const uint16_t throttle =
      radxa::RadxaBoardIo::normalizedToDshot(normalized, true);

  std::cout << "arming " << device << " with DShot stop frames for " << arm_ms
            << " ms" << std::endl;
  if (!sendFor(driver, 0, std::chrono::milliseconds(arm_ms))) {
    driver.writeStop();
    return 1;
  }

  if (!stop_requested.load()) {
    std::cout << "running DShot value " << throttle << " for " << run_ms
              << " ms, telemetry=" << (telemetry ? "on" : "off") << std::endl;
    if (!sendFor(driver, throttle, std::chrono::milliseconds(run_ms), telemetry)) {
      driver.writeStop();
      return 1;
    }
  }

  std::cout << "stopping" << std::endl;
  const bool stopped = sendFor(driver, 0, std::chrono::milliseconds(500));
  driver.writeStop();
  return stopped ? 0 : 1;
}
