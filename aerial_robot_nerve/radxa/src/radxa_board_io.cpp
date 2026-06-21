#include <iostream>
#include <cstdint>

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
     i2c("/dev/i2c-7")
  {
    for (PwmConfig config : pwm_configs)
      {
	PwmDriver pwm(config.pwm_chip, config.pwm_channel, pwm_freq);
	pwm.open();
	pwm.setPulseWidthUs(0);
	pwm.enable();
	pwm_drivers_.push_back(std::unique_ptr<PwmDriver>(pwm));
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

    
  }
  
  RadxaBoardIo::~RadxaBoardIo() override
  {
    i2c.close();
    for (pwm : pwm_drivers_){
      pwm->disable();
      pwm->close();
    }
  }

  RadxaBoardIo::getVoltage(float& voltage){
    // added later
    std::cout << voltage;
  }

  RadxaBoardIo::readImu(ImuRaw& data) override {
    // read acc
    uint8_t raw_acc[6];
    if(!i2c.writeRead(imu, &accel_reg, 1, raw_acc, 6)){
      std::cerr << "fail to w_r: 0x" << std::hex << static_cast<int>(imu) << std::dec << std::endl;
      return false;
    }
    data->acc[0] = static_cast<int16_t>((raw_acc[0] << 8)| raw_acc[1]); // ax
    data->acc[1] = static_cast<int16_t>((raw_acc[2] << 8)| raw_acc[3]); // ay
    data->acc[2] = static_cast<int16_t>((raw_acc[4] << 8)| raw_acc[5]); // az

    // read gyro
    uint8_t raw_gyro[6];
    if(!i2c.writeRead(imu, &gyro_reg, 1, raw_gyro, 6)){
      std::cerr << "fail to w_r: 0x" << std::hex << static_cast<int>(imu) << std::dec << std::endl;
      return false;
    }
    data->gyro[0] = static_cast<int16_t>((raw_gyro[0] << 8)| raw_gyro[1]); // gyro x
    data->gyro[1] = static_cast<int16_t>((raw_gyro[2] << 8)| raw_gyro[3]); // gyro y
    data->gyro[2] = static_cast<int16_t>((raw_gyro[4] << 8)| raw_gyro[5]); // gyro z

    return true;
  }

  RadxaBoardIo::setMotorPwms(const float* pwms, int motor_number) override {
    for (int i = 0; int < motor_number; i++){
      if(!pwm_drivers_[i]->setPulseWidthUs(pwms[i] / pwm_freq * 1000000)){
	std::cerr << "fail to set pulse width: pwm" << i;
	return false;
      };
    }

    return true;
  }

  
}
