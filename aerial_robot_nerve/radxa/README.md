# Cubie A7Z spinal backend

`radxa_spinal_node` runs the original spinal attitude-control core and the
hardware backend in one ROS Noetic process. rosserial is not used.

Implemented interfaces:

- output 1–4: DShot300 encoded as 2.4 MHz SPI waveforms;
- output 5–8: conventional Linux PWM; Crobat `extra_servo_cmd` indices 4–7
  retain spinal's angle/180 duty conversion;
- ICM-20948 over I2C: accel, gyro, AK09916 magnetometer, SI-unit conversion,
  calibration parameters, and the existing complementary attitude estimator;
- ADS1015 over I2C: divided battery voltage, spinal-compatible filtering, and
  `battery_voltage_status` / `set_adc_scale`;
- the existing `FlightControl` subscriptions, including `uav_info`,
  `flight_config_cmd`, `four_axes/command`, `motor_info`, and `pwm_test`.

The native ROS attitude controller is the same spinal PID/allocation core used
on the MCU. On Radxa, the filtered ADS1015 measurement is injected into its
thrust-to-PWM voltage compensation. Crobat's `gimbals_ctrl` command is converted
by `servo_bridge` to `extra_servo_cmd`, then drives PWM ports 5–8 independently
of the DShot arm state. Crobat's real-machine `FlightControl.yaml` enables
`gimbal_calc_in_fc`, so this native attitude loop computes both motor thrust and
tilt angles; the simulation YAML remains unchanged.

## Hardware constraints

Linux PWM produces a repeating fixed-duty signal and cannot produce a DShot
frame. This backend therefore requires **four independent SPI controllers and
four independent MOSI pins** for outputs 1–4. Each controller sends one motor's
waveform. Configure the Cubie A7Z pinmux overlays first, then set the four
`/dev/spidevX.Y` paths in `config/cubie_a7z.yaml` in motor order. Do not connect
SPI clock or chip-select to the ESC.

If four independent SPI MOSI outputs cannot be exposed simultaneously by the
selected A7Z overlay, use a small real-time DShot coprocessor. GPIO bit-banging
from normal Linux userspace is not flight-safe at 300 kbit/s.

The PWM chip numbers are kernel/overlay dependent. After applying overlays,
verify them with:

```bash
ls -l /sys/class/pwm
ls -l /dev/spidev* /dev/i2c-*
```

## I2C wiring and battery divider

Power the ICM-20948 and ADS1015 breakouts from 3.3 V and share SDA, SCL, and
ground. The default addresses are ICM `0x69`, internal AK09916 `0x0c`, and
ADS1015 `0x48`. Some ICM boards use `0x68`; change `imu_address` if necessary.

The ADS1015 board is not a high-voltage input. Add an external divider:

```text
battery + --- R_top ---+--- ADS1015 A0
                      |
                    R_bottom
                      |
battery - ------------+--- ADS1015 GND --- Cubie A7Z GND
```

Set `voltage_divider_ratio` to `(R_top + R_bottom) / R_bottom`. The default
11.0 corresponds to 100 kΩ / 10 kΩ. Check that the maximum battery voltage
divided by this ratio stays below the ADC supply voltage; the ADS1015 input
must never exceed `VDD + 0.3 V`.

Check device discovery on the host before starting ROS:

```bash
sudo i2cdetect -y 7
```

## Build and run in Docker

From this repository root:

```bash
docker compose build
docker compose up -d
docker compose exec jsk-aerial-robot bash
source /opt/ros/noetic/setup.bash
cd /root/ros/jsk_aerial_robot_ws
catkin config --cmake-args -DSPINAL_BUILD_MCU_ROS_LIB=OFF
catkin build spinal radxa
source devel/setup.bash
roslaunch crobat bridge.launch
```

The compose file passes `/dev` and `/sys` into the privileged container. The
host must enable I2C/PWM/SPI and install the overlays before container startup.

## Bench tests

Remove every propeller and power motors from a current-limited supply.

```bash
# ICM-20948 values (Ctrl-C to stop)
rosrun radxa imu_test /dev/i2c-7 0x69

# One conventional PWM channel for three seconds
rosrun radxa pwm_test 10 7 500 0.5

# Pure conversion/encoding tests (no hardware)
catkin run_tests radxa
catkin_test_results
```

Inspect ROS output with:

```bash
rostopic hz /crobat/imu
rostopic echo /crobat/battery_voltage_status
```

`set_adc_scale` retains spinal's meaning: battery volts per raw ADS1015 count.
For example, publish a calibrated value with
`rostopic pub -1 /crobat/set_adc_scale std_msgs/Float32 'data: 0.0061'`.
The value is mirrored to the ROS parameter server but is not written back to
the YAML file.

## Safety notes

- DShot outputs are forced low after an IMU timeout; PWM ports 5–8 hold their
  latest servo command. Every output is forced low when the node exits.
- DShot sends command 0 unless armed or `pwm_test` is active.
- Confirm the physical pin for every pwmchip/spidev path with an oscilloscope
  before connecting an ESC.
- Standard Ubuntu is not a hard real-time flight platform. Validate loop jitter
  under the final workload; use a PREEMPT_RT kernel or a real-time output
  coprocessor before free flight.
