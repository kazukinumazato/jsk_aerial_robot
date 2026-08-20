#include <array>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <spinal/PwmTest.h>
#include <spinal/ServoControlCmd.h>
#include <spinal/ServoStates.h>
#include <spinal/ServoTorqueCmd.h>

#include <flight_control/flight_control.h>
#include <state_estimate/state_estimate.h>

#include <radxa/radxa_board_io.h>

namespace
{

template <typename T, std::size_t N>
void loadArray(ros::NodeHandle& nh, const std::string& name,
               std::array<T, N>& values)
{
  std::vector<T> parameter;
  if (!nh.getParam(name, parameter)) {
    return;
  }
  if (parameter.size() != N) {
    ROS_WARN_STREAM("~" << name << " must contain " << N << " values");
    return;
  }
  std::copy(parameter.begin(), parameter.end(), values.begin());
}

radxa::RadxaBoardConfig loadBoardConfig(ros::NodeHandle& pnh)
{
  radxa::RadxaBoardConfig config;
  pnh.param("i2c_device", config.i2c_device, config.i2c_device);

  int imu_address = config.imu.address;
  pnh.param("imu_address", imu_address, imu_address);
  config.imu.address = static_cast<uint8_t>(imu_address);
  pnh.param("enable_magnetometer", config.imu.enable_magnetometer,
            config.imu.enable_magnetometer);
  pnh.param("require_magnetometer", config.imu.require_magnetometer,
            config.imu.require_magnetometer);
  loadArray(pnh, "imu_axis_map", config.imu.axis_map);
  loadArray(pnh, "imu_axis_sign", config.imu.axis_sign);
  loadArray(pnh, "mag_axis_map", config.imu.mag_axis_map);
  loadArray(pnh, "mag_axis_sign", config.imu.mag_axis_sign);
  loadArray(pnh, "accel_bias", config.imu.accel_bias);
  loadArray(pnh, "gyro_bias", config.imu.gyro_bias);
  loadArray(pnh, "mag_bias", config.imu.mag_bias);
  loadArray(pnh, "mag_scale", config.imu.mag_scale);

  int adc_address = config.adc.address;
  int adc_channel = config.adc.channel;
  pnh.param("adc_address", adc_address, adc_address);
  pnh.param("adc_channel", adc_channel, adc_channel);
  config.adc.address = static_cast<uint8_t>(adc_address);
  config.adc.channel = static_cast<uint8_t>(adc_channel);
  pnh.param("voltage_divider_ratio", config.adc.divider_ratio,
            config.adc.divider_ratio);
  pnh.param("adc_scale", config.adc.adc_scale, config.adc.adc_scale);

  pnh.getParam("dshot_spi_devices", config.dshot_spi_devices);
  int dshot_speed = static_cast<int>(config.dshot_spi_speed_hz);
  pnh.param("dshot_spi_speed_hz", dshot_speed, dshot_speed);
  config.dshot_spi_speed_hz = static_cast<uint32_t>(dshot_speed);
  loadArray(pnh, "dshot_reversed", config.dshot_reversed);
  pnh.param("require_all_dshot_channels", config.require_all_dshot_channels,
            config.require_all_dshot_channels);

  std::vector<int> pwm_chips;
  std::vector<int> pwm_channels;
  if (pnh.getParam("pwm_chips", pwm_chips) && pwm_chips.size() == 4) {
    for (std::size_t i = 0; i < 4; ++i) {
      config.pwm[i].chip = pwm_chips[i];
    }
  }
  if (pnh.getParam("pwm_channels", pwm_channels) &&
      pwm_channels.size() == 4) {
    for (std::size_t i = 0; i < 4; ++i) {
      config.pwm[i].channel = pwm_channels[i];
    }
  }
  pnh.param("pwm_frequency_hz", config.pwm_frequency_hz,
            config.pwm_frequency_hz);
  return config;
}

class RadxaSpinalNode
{
public:
  RadxaSpinalNode()
    : nh_(), pnh_("~"), board_(loadBoardConfig(pnh_))
  {
  }

  bool init()
  {
    pnh_.param("control_rate_hz", control_rate_hz_, 500.0);
    pnh_.param("imu_timeout", imu_timeout_, 0.1);
    pnh_.param("imu_zero_on_startup", imu_zero_on_startup_, true);
    pnh_.param("imu_zero_duration_sec", imu_zero_duration_sec_, 3.0);
    pnh_.param("imu_zero_sample_rate_hz", imu_zero_sample_rate_hz_, 500.0);
    pnh_.param("voltage_read_rate_hz", voltage_read_rate_hz_, 50.0);
    pnh_.param("voltage_filter_alpha", voltage_filter_alpha_, 0.01F);
    pnh_.param("pwm_angle_range", pwm_angle_range_, 180.0F);
    pnh_.param("servo_state_rate_hz", servo_state_rate_hz_, 50.0);
    loadArray(pnh_, "pwm_initial_duty", servo_duty_);
    voltage_filter_alpha_ = std::clamp(voltage_filter_alpha_, 0.0F, 1.0F);
    if (pwm_angle_range_ <= 0.0F) {
      ROS_FATAL("~pwm_angle_range must be positive");
      return false;
    }
    if (servo_state_rate_hz_ <= 0.0) {
      ROS_FATAL("~servo_state_rate_hz must be positive");
      return false;
    }
    for (float& duty : servo_duty_) {
      duty = std::clamp(duty, 0.0F, 1.0F);
    }

    if (!board_.init()) {
      ROS_FATAL("failed to initialize Cubie A7Z hardware");
      return false;
    }

    if (imu_zero_on_startup_) {
      ROS_WARN_STREAM("IMU gyro zeroing: keep Crobat completely stationary for "
                      << imu_zero_duration_sec_ << " seconds");
      std::array<float, 3> applied_bias{};
      if (!board_.zeroImuGyro(imu_zero_duration_sec_,
                              imu_zero_sample_rate_hz_, applied_bias)) {
        ROS_FATAL("failed to zero ICM-20948 gyroscope");
        return false;
      }
      ROS_INFO_STREAM("IMU gyro zero complete; bias [rad/s] = ["
                      << applied_bias[0] << ", " << applied_bias[1] << ", "
                      << applied_bias[2] << "]");
    }

    estimator_.init(&nh_);
    controller_.init(&nh_, &estimator_);

    battery_pub_ = nh_.advertise<std_msgs::Float32>("battery_voltage_status", 1);
    adc_scale_sub_ = nh_.subscribe("set_adc_scale", 1,
                                   &RadxaSpinalNode::adcScaleCallback, this);
    pwm_test_sub_ = nh_.subscribe("pwm_test", 1,
                                  &RadxaSpinalNode::pwmTestCallback, this);
    servo_control_sub_ = nh_.subscribe("extra_servo_cmd", 1,
        &RadxaSpinalNode::servoControlCallback, this);
    servo_torque_sub_ = nh_.subscribe("extra_servo_torque_enable", 1,
        &RadxaSpinalNode::servoTorqueCallback, this);
    servo_state_pub_ = nh_.advertise<spinal::ServoStates>("servo/states", 1);

    last_imu_success_ = ros::SteadyTime::now();
    next_voltage_read_ = ros::SteadyTime::now();
    next_servo_state_publish_ = ros::SteadyTime::now();
    ROS_INFO_STREAM("radxa spinal node ready: " << board_.dshotChannelCount()
                    << "/4 DShot channels configured");
    return true;
  }

  void run()
  {
    ros::WallRate rate(control_rate_hz_);
    while (ros::ok()) {
      ros::spinOnce();

      radxa::ImuSample imu;
      if (board_.readImu(imu)) {
        estimator_.getAttEstimator()->setAcc(
            imu.acc_mps2[0], imu.acc_mps2[1], imu.acc_mps2[2]);
        estimator_.getAttEstimator()->setGyro(
            imu.gyro_radps[0], imu.gyro_radps[1], imu.gyro_radps[2]);
        estimator_.getAttEstimator()->setMag(
            imu.mag_tesla[0], imu.mag_tesla[1], imu.mag_tesla[2]);
        estimator_.update();
        last_imu_success_ = ros::SteadyTime::now();
      } else {
        ROS_ERROR_THROTTLE(1.0, "failed to read ICM-20948");
      }

      updateBattery();
      publishServoStates();
      controller_.update();

      const bool imu_healthy =
          (ros::SteadyTime::now() - last_imu_success_).toSec() <= imu_timeout_;
      if (!imu_healthy) {
        ROS_ERROR_THROTTLE(1.0, "IMU timeout: forcing every DShot output low");
        std::array<float, 8> outputs{};
        fillServoOutputs(outputs);
        if (!board_.setMotorOutputs(outputs.data(), outputs.size(), false)) {
          ROS_ERROR_THROTTLE(1.0, "failed to hold PWM outputs during IMU timeout");
        }
      } else {
        std::array<float, 8> outputs{};
        bool controller_output_valid = true;
        for (std::size_t i = 0; i < 4; ++i) {
          outputs[i] = controller_.getAttController().getPwm(i);
          controller_output_valid =
              controller_output_valid && std::isfinite(outputs[i]);
        }
        if (!controller_output_valid) {
          ROS_ERROR_THROTTLE(
              1.0,
              "non-finite attitude-controller PWM: [%g, %g, %g, %g]; "
              "DShot stop is being sent",
              outputs[0], outputs[1], outputs[2], outputs[3]);
        }
        fillServoOutputs(outputs);
        if (!board_.setMotorOutputs(outputs.data(), outputs.size(),
                                    controller_.isArmed() || pwm_test_active_)) {
          ROS_ERROR_THROTTLE(1.0, "failed to update one or more motor outputs");
        }
      }
      rate.sleep();
    }
    board_.stopOutputs();
  }

private:
  void updateBattery()
  {
    const ros::SteadyTime now = ros::SteadyTime::now();
    if (now < next_voltage_read_) {
      return;
    }
    next_voltage_read_ = now + ros::WallDuration(1.0 / voltage_read_rate_hz_);

    float voltage = 0.0F;
    if (!board_.readBatteryVoltage(voltage)) {
      ROS_WARN_THROTTLE(1.0, "failed to read ADS1015 battery voltage");
      return;
    }
    if (voltage <= 0.0F) {
      ROS_WARN_THROTTLE(1.0,
                        "ADS1015 battery input is zero or negative");
      return;
    }
    if (!voltage_valid_) {
      filtered_voltage_ = voltage;
      voltage_valid_ = true;
    } else {
      filtered_voltage_ = (1.0F - voltage_filter_alpha_) * filtered_voltage_ +
                          voltage_filter_alpha_ * voltage;
    }
    controller_.getAttController().setMeasuredVoltage(filtered_voltage_);

    if ((now - last_voltage_publish_).toSec() >= 0.1) {
      std_msgs::Float32 message;
      message.data = filtered_voltage_;
      battery_pub_.publish(message);
      last_voltage_publish_ = now;
    }
  }

  void adcScaleCallback(const std_msgs::Float32ConstPtr& message)
  {
    if (message->data <= 0.0F) {
      ROS_WARN("set_adc_scale must be positive");
      return;
    }
    board_.setAdcScale(message->data);
    pnh_.setParam("adc_scale", message->data);
    voltage_valid_ = false;
    ROS_INFO_STREAM("ADC scale changed to " << message->data
                    << " battery volts/count");
  }

  void pwmTestCallback(const spinal::PwmTestConstPtr& message)
  {
    pwm_test_active_ = !message->pwms.empty();
  }

  void servoControlCallback(const spinal::ServoControlCmdConstPtr& message)
  {
    if (message->index.size() != message->angles.size()) {
      ROS_ERROR("extra_servo_cmd index/angles sizes do not match");
      return;
    }
    for (std::size_t i = 0; i < message->index.size(); ++i) {
      const uint8_t index = message->index[i];
      if (index < 4 || index > 7) {
        ROS_WARN_STREAM("extra_servo_cmd index " << static_cast<int>(index)
                        << " is not a Radxa PWM port (expected 4..7)");
        continue;
      }
      const float angle = std::clamp(static_cast<float>(message->angles[i]),
                                     0.0F, pwm_angle_range_);
      servo_duty_[index - 4] = angle / pwm_angle_range_;
      servo_enabled_[index - 4] = true;
    }
  }

  void servoTorqueCallback(const spinal::ServoTorqueCmdConstPtr& message)
  {
    if (message->index.size() != message->torque_enable.size()) {
      ROS_ERROR("extra_servo_torque_enable index/enable sizes do not match");
      return;
    }
    for (std::size_t i = 0; i < message->index.size(); ++i) {
      const uint8_t index = message->index[i];
      if (index >= 4 && index <= 7) {
        servo_enabled_[index - 4] = message->torque_enable[i] != 0;
      }
    }
  }

  void publishServoStates()
  {
    const ros::SteadyTime now = ros::SteadyTime::now();
    if (now < next_servo_state_publish_) {
      return;
    }
    next_servo_state_publish_ =
        now + ros::WallDuration(1.0 / servo_state_rate_hz_);

    /*
     * Ports 5..8 are ordinary PWM outputs and therefore have no position
     * feedback. Report the last commanded angle as the estimated state so the
     * existing spinal-compatible path
     *
     *   servo/states -> servo_bridge -> joint_states -> robot model
     *
     * remains active. Crobat's Servo.yaml uses servo IDs 1..4, while the
     * Radxa output ports themselves are addressed as 4..7.
     */
    spinal::ServoStates message;
    message.stamp = ros::Time::now();
    message.servos.resize(servo_duty_.size());
    for (std::size_t i = 0; i < servo_duty_.size(); ++i) {
      spinal::ServoState& state = message.servos[i];
      state.index = static_cast<uint8_t>(i + 1);
      state.angle = static_cast<int16_t>(
          std::lround(std::clamp(servo_duty_[i], 0.0F, 1.0F) *
                      pwm_angle_range_));
      state.temp = 0;
      state.load = 0;
      state.error = 0;
    }
    servo_state_pub_.publish(message);
  }

  void fillServoOutputs(std::array<float, 8>& outputs) const
  {
    for (std::size_t i = 0; i < servo_duty_.size(); ++i) {
      outputs[i + 4] = servo_enabled_[i] ? servo_duty_[i] : 0.0F;
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  radxa::RadxaBoardIo board_;
  StateEstimate estimator_;
  FlightControl controller_;

  ros::Publisher battery_pub_;
  ros::Publisher servo_state_pub_;
  ros::Subscriber adc_scale_sub_;
  ros::Subscriber pwm_test_sub_;
  ros::Subscriber servo_control_sub_;
  ros::Subscriber servo_torque_sub_;

  double control_rate_hz_{500.0};
  double imu_timeout_{0.1};
  bool imu_zero_on_startup_{true};
  double imu_zero_duration_sec_{3.0};
  double imu_zero_sample_rate_hz_{500.0};
  double voltage_read_rate_hz_{50.0};
  float voltage_filter_alpha_{0.01F};
  float filtered_voltage_{0.0F};
  float pwm_angle_range_{180.0F};
  double servo_state_rate_hz_{50.0};
  bool voltage_valid_{false};
  bool pwm_test_active_{false};
  std::array<float, 4> servo_duty_{{0.5F, 0.5F, 0.5F, 0.5F}};
  std::array<bool, 4> servo_enabled_{{true, true, true, true}};
  ros::SteadyTime last_imu_success_;
  ros::SteadyTime next_voltage_read_;
  ros::SteadyTime next_servo_state_publish_;
  ros::SteadyTime last_voltage_publish_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "radxa_spinal");
  RadxaSpinalNode node;
  if (!node.init()) {
    return 1;
  }
  node.run();
  return 0;
}
