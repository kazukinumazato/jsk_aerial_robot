#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <radxa/dshot_driver.h>

namespace
{
bool sendStopFor(radxa::DshotDriver& driver, std::chrono::milliseconds duration)
{
  const auto end = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < end) {
    if (!driver.writeStop()) {
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
  radxa::DshotDriver driver(device, 1600000);
  if (!driver.open()) {
    return 1;
  }

  // Match Betaflight's protocol-detection and blocking-command timing:
  // wait at stop, wait 10 ms, send BEACON1 once with telemetry requested,
  // then leave at least the documented 260 ms beep duration before returning
  // to stop frames.
  std::cout << "sending stop frames for 3000 ms" << std::endl;
  if (!sendStopFor(driver, std::chrono::milliseconds(3000))) {
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  std::cout << "sending DShot BEACON1" << std::endl;
  if (!driver.writeValue(1, true)) {
    driver.writeStop();
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  std::cout << "returning to stop" << std::endl;
  return sendStopFor(driver, std::chrono::milliseconds(1000)) ? 0 : 1;
}
