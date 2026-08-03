#include <iostream>
#include <thread>
#include <chrono>

#include <radxa/i2c_driver.h>

int main() {
  const std::string device = "/dev/i2c-7";
  radxa::I2cDriver i2c(device);
  
  if(!i2c.open()){
    std::cerr <<"fail to open: " << device << std::endl;
    return 1;
  }

  uint8_t imu = 0x69;
  uint8_t reg = 0x00;
  uint8_t value = 0;
  bool w_r_flag = i2c.writeRead(imu, &reg, 1, &value, 1);

  if (!w_r_flag){
    std::cerr << "fail to w_r: 0x" << std::hex << static_cast<int>(imu) << std::dec << std::endl;
    return 1;
  }

  std::cout << "WHO_AM_I: 0x" << std::hex << static_cast<int>(value) << std::dec << std::endl;

  if (value != 0xEA){
    std::cerr << "Wrong device accessed" << std::endl;
    return 1;
  }

  // awake imu
  uint8_t pwr[2] = {0x06, 0x01};

  if (!i2c.write(imu, pwr, 2)){
    std::cerr << "fail to write: 0x" << std::hex << static_cast<int>(pwr[0])
	      << std::dec << std::endl;
    return 1;
  }

  std::cout << "IMU wake up command sent" << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  while(true){
    uint8_t accel_x_reg = 0x2D;
    uint8_t data[6];

    if(!i2c.writeRead(imu, &accel_x_reg, 1, data, 6)){
      std::cerr << "fail to w_r: 0x" << std::hex << static_cast<int>(imu) << std::dec << std::endl;
      return 1;
    }
    int16_t ax = static_cast<int16_t>((data[0] << 8)| data[1]);
    int16_t ay = static_cast<int16_t>((data[2] << 8)| data[3]);
    int16_t az = static_cast<int16_t>((data[4] << 8)| data[5]);

    std::cout << "ACCEL raw: "
	      << "X=" << ax << " "
	      << "Y=" << ay << " "
	      << "Z=" << az
	      << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  i2c.close();
  return 0;
}
