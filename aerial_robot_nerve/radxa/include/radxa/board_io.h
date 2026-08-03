#pragma once

namespace radxa
{

  class BoardIo
  {
  public:
    virtual ~BoardIo() = default;

    virtual bool Init() = 0;

    struct ImuRaw
    {
      float acc[3];
      float gyro[3];
    };
    
    virtual bool getVoltage(float& voltage) = 0;

    virtual bool readImu(ImuRaw& data) = 0;

    virtual bool setMotorPwms(const float* pwms, int motor_number) = 0;

  };

}
