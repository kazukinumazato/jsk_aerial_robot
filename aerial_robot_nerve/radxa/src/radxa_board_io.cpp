#include <iostream>
#include <cstdint>
#include <chrono>
#include <thread>

#include <radxa/radxa_board_io.h>
#include <radxa/pwm_driver.h>
#include <radxa/i2c_driver.h>

namespace radxa
{
  RadxaBoardIo::RadxaBoardIo()
    :pwm_configs({{10, 1},
		  {10, 2},
		  {10, 3},
		  {10, 6},
		  {10, 7},
		  {20, 1},
		  {20, 2},
		  {20, 3}}),
     i2c("/dev/i2c-7"),
     imu_who_am_i(0)
  {
  }

  bool RadxaBoardIo::Init(){
    for (PwmConfig config : pwm_configs)
      {
	pwm_drivers_.push_back(std::make_unique<PwmDriver>(config.pwm_chip, config.pwm_channel, pwm_freq));
      }

    for (auto& pwm_driver : pwm_drivers_)
      {
	if (!pwm_driver->open()) {
	  return false;
	}

	if (!pwm_driver->setPulseWidthUs(0)) {
	  return false;
	}

	if (!pwm_driver->enable()) {
	  return false;
	}
      }

    // awake imu
    if(!i2c.writeRead(imu, &imu_who_am_i_reg, 1, &imu_who_am_i, 1)){
      std::cerr << "fail to w_r: 0x" << std::hex << static_cast<int>(imu) << std::dec << std::endl;
      return false;
    };
    std::cout << "WHO_AM_I: 0x" << std::hex << static_cast<int>(imu_who_am_i)
	      << std::dec << std::endl;

    if (imu_who_am_i != 0xEA){
      std::cerr << "Wrong device accessed" << std::endl;
      return false;
    }
    
    if (!i2c.write(imu, pwr, 2)){
      std::cerr << "fail to write: 0x" << std::hex << static_cast<int>(pwr[0])
		<< std::dec << std::endl;
      return false;
    }

    std::cout << "IMU wake up command sent" << std::endl;
    return true;
  }

  bool RadxaBoardIo::getVoltage(float& voltage)
  {
    constexpr uint8_t adc_address = 0x48;

    constexpr uint8_t reg_conversion = 0x00;
    constexpr uint8_t reg_config = 0x01;

    constexpr uint8_t adc_channel = 0; // A0

    constexpr float divider_ratio = 3.0f; // 実際の抵抗値に合わせて変更

    // ADS1015 PGA = ±4.096V
    constexpr float full_scale_voltage = 4.096f;

    // ADS1015 config register
    constexpr uint16_t os_single = 0x8000;
    constexpr uint16_t pga_4_096 = 0x0200;
    constexpr uint16_t mode_single_shot = 0x0100;
    constexpr uint16_t data_rate_1600sps = 0x0080;
    constexpr uint16_t comp_disable = 0x0003;

    // single-ended AIN0/AIN1/AIN2/AIN3
    const uint16_t mux =
      static_cast<uint16_t>((0x04 + adc_channel) << 12);

    const uint16_t config =
      os_single |
      mux |
      pga_4_096 |
      mode_single_shot |
      data_rate_1600sps |
      comp_disable;

    uint8_t config_data[3];

    config_data[0] = reg_config;
    config_data[1] = static_cast<uint8_t>((config >> 8) & 0xFF);
    config_data[2] = static_cast<uint8_t>(config & 0xFF);

    if (!i2c.write(adc_address, config_data, 3)) {
      std::cerr << "failed to write ADS1015 config" << std::endl;
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    uint8_t raw_data[2];

    if (!i2c.writeRead(adc_address, &reg_conversion, 1, raw_data, 2)) {
      std::cerr << "failed to read ADS1015 conversion" << std::endl;
      return false;
    }

    uint16_t raw16 =
      static_cast<uint16_t>(raw_data[0]) << 8 |
      static_cast<uint16_t>(raw_data[1]);

    raw16 >>= 4;
    
    if (raw16 & 0x0800) {
      raw16 |= 0xF000;
    }

    const int16_t raw = static_cast<int16_t>(raw16);

    const float adc_voltage =
      static_cast<float>(raw) * full_scale_voltage / 2048.0f;

    voltage = adc_voltage * divider_ratio;

    return true;
  }
  bool RadxaBoardIo::readImu(ImuRaw& data) {
    // read acc
    uint8_t raw_acc[6];
    if(!i2c.writeRead(imu, &accel_reg, 1, raw_acc, 6)){
      std::cerr << "fail to w_r: 0x" << std::hex << static_cast<int>(imu) << std::dec << std::endl;
      return false;
    }
    data.acc[0] = static_cast<int16_t>((raw_acc[0] << 8)| raw_acc[1]); // ax
    data.acc[1] = static_cast<int16_t>((raw_acc[2] << 8)| raw_acc[3]); // ay
    data.acc[2] = static_cast<int16_t>((raw_acc[4] << 8)| raw_acc[5]); // az

    // read gyro
    uint8_t raw_gyro[6];
    if(!i2c.writeRead(imu, &gyro_reg, 1, raw_gyro, 6)){
      std::cerr << "fail to w_r: 0x" << std::hex << static_cast<int>(imu) << std::dec << std::endl;
      return false;
    }
    data.gyro[0] = static_cast<int16_t>((raw_gyro[0] << 8)| raw_gyro[1]); // gyro x
    data.gyro[1] = static_cast<int16_t>((raw_gyro[2] << 8)| raw_gyro[3]); // gyro y
    data.gyro[2] = static_cast<int16_t>((raw_gyro[4] << 8)| raw_gyro[5]); // gyro z

    return true;
  }

<<<<<<< Updated upstream
  bool RadxaBoardIo::setMotorPwms(const float* pwms, int motor_number) override {
    for (int i = 0; int < motor_number; i++){
=======
  bool RadxaBoardIo::setMotorPwms(const float* pwms, int motor_number) {
    for (int i = 0; i < motor_number; i++){
>>>>>>> Stashed changes
      if(!pwm_drivers_[i]->setPulseWidthUs(pwms[i] / pwm_freq * 1000000)){
	std::cerr << "fail to set pulse width: pwm" << i;
	return false;
      };
    }

    return true;
  }

  
}
