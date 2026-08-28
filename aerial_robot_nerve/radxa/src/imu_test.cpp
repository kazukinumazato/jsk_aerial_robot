#include <chrono>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <ros/package.h>

#include <radxa/i2c_driver.h>
#include <radxa/icm20948.h>

namespace
{
std::string trim(const std::string& value)
{
  std::size_t begin = 0;
  while (begin < value.size() &&
         std::isspace(static_cast<unsigned char>(value[begin]))) {
    ++begin;
  }
  std::size_t end = value.size();
  while (end > begin &&
         std::isspace(static_cast<unsigned char>(value[end - 1]))) {
    --end;
  }
  return value.substr(begin, end - begin);
}

bool loadYamlScalar(const std::string& path, const std::string& key,
                    std::string& value)
{
  std::ifstream stream(path);
  if (!stream) {
    std::cerr << "failed to open Radxa config: " << path << std::endl;
    return false;
  }

  std::string line;
  while (std::getline(stream, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line.erase(comment);
    }
    const auto separator = line.find(':');
    if (separator == std::string::npos ||
        trim(line.substr(0, separator)) != key) {
      continue;
    }
    value = trim(line.substr(separator + 1));
    if (value.size() >= 2 &&
        ((value.front() == '"' && value.back() == '"') ||
         (value.front() == '\'' && value.back() == '\''))) {
      value = value.substr(1, value.size() - 2);
    }
    if (value.empty()) {
      std::cerr << key << " is empty in " << path << std::endl;
      return false;
    }
    return true;
  }

  std::cerr << key << " is missing from " << path << std::endl;
  return false;
}

bool parseI2cAddress(const std::string& value, uint8_t& address)
{
  try {
    std::size_t parsed = 0;
    const unsigned long numeric = std::stoul(value, &parsed, 0);
    if (parsed != value.size() || numeric > 0x7f) {
      throw std::out_of_range("not a 7-bit I2C address");
    }
    address = static_cast<uint8_t>(numeric);
    return true;
  } catch (const std::exception& error) {
    std::cerr << "invalid imu_address '" << value << "': " << error.what()
              << std::endl;
    return false;
  }
}
}  // namespace

int main(int argc, char** argv)
{
  const std::string package_path = ros::package::getPath("radxa");
  if (package_path.empty()) {
    std::cerr << "failed to locate the radxa ROS package" << std::endl;
    return 1;
  }
  const std::string config_path = package_path + "/config/cubie_a7z.yaml";

  std::string configured_device;
  std::string configured_address;
  if (!loadYamlScalar(config_path, "i2c_device", configured_device) ||
      !loadYamlScalar(config_path, "imu_address", configured_address)) {
    return 1;
  }

  const std::string device = argc > 1 ? argv[1] : configured_device;
  const std::string address_text =
      argc > 2 ? argv[2] : configured_address;
  uint8_t address = 0;
  if (!parseI2cAddress(address_text, address)) {
    return 1;
  }

  std::cout << "using " << config_path << ": " << device
            << ", IMU address 0x" << std::hex << static_cast<int>(address)
            << std::dec << std::endl;

  radxa::I2cDriver i2c(device);
  if (!i2c.open()) {
    return 1;
  }
  radxa::Icm20948::Config config;
  config.address = address;
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
