# Cubie A7Z spinal backend

`radxa_spinal_node` runs the original spinal attitude-control core and the
hardware backend in one ROS Noetic process. rosserial is not used.

Implemented interfaces:

- output 1–4: DShot300 encoded as packed 1.6 MHz SPI waveforms;
- output 5–8: conventional Linux PWM; Crobat `extra_servo_cmd` indices 4–7
  retain spinal's angle/180 duty conversion;
- ICM-20948 over I2C: accel/gyro, optional AK09916 magnetometer, SI-unit
  conversion, calibration parameters, and the existing complementary attitude
  estimator;
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

Because ordinary PWM provides no position feedback, the Radxa node publishes
the last commanded servo angles on `servo/states` (50 Hz by default). This is a
commanded-state estimate, not a measured angle. It keeps the existing
`servo_bridge`/`joint_states` robot-model update path compatible with spinal.

## Crobat backend and URDF selection

The full-actuated real-machine model is selected with the same `backend`
argument as `bridge.launch`:

```bash
# Original spinal controller over rosserial
roslaunch crobat bringup.launch full_actuated:=true backend:=serial

# Cubie A7Z in-process controller
roslaunch crobat bringup.launch full_actuated:=true backend:=radxa
```

Starting `crobat bridge.launch` with the Radxa backend keeps DShot stopped and
averages the stationary gyroscope for three seconds before starting attitude
estimation. Keep the vehicle completely still until `IMU gyro zero complete`
appears. To change or explicitly disable this behavior:

```bash
roslaunch crobat bridge.launch backend:=radxa imu_zero_duration_sec:=5.0
roslaunch crobat bridge.launch backend:=radxa imu_zero_on_startup:=false
```

Run only one Radxa `bridge.launch` at a time. The backend takes a process lock
before registering with the ROS master, then holds an exclusive lock on its I2C
device. It rejects a second instance before it can replace the healthy ROS node
or write to the shared PWM and DShot outputs.

Only the gyroscope is zeroed. Acceleration is not forced to zero because its
stationary gravity vector is required to initialize roll and pitch. The
calculated bias is runtime-only and is recalculated at every bridge start.

The serial and Radxa entry points share the common geometry in
`full_actuated.urdf.xacro`; their wrapper files retain the backend-specific
base and battery masses. Gazebo has a separate entry point and is selected only
when `simulation:=true`. `flight_controller_backend:=...` remains available as
a compatibility override, and either hardware model path can be replaced with
`serial_robot_model:=...` or `radxa_robot_model:=...`.

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
The current breakout does not ACK its internal AK09916 in bypass mode, so
`cubie_a7z.yaml` keeps `enable_magnetometer: false`; attitude control uses the
working accel/gyro path. Enable it only after confirming address `0x0c` works.
The ROS backend maps acceleration and angular velocity to Crobat's body frame
with `imu_axis_sign: [1.0, -1.0, 1.0]`. Magnetometer mapping is independent and
uses `mag_axis_sign: [1.0, -1.0, -1.0]`, preserving spinal's AK09916 Z
reversal. The standalone `imu_test` reads the raw driver defaults and does not
load these YAML mappings; the mapped values are the ones consumed by the
attitude estimator in `radxa_spinal_node`.

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
sudo i2cdetect -y 2
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
rosrun radxa imu_test /dev/i2c-2 0x69
rosrun radxa adc_test /dev/i2c-2 0x48 0 11.0 20

# One DShot output: stop frames, at most 0.55 for at most two seconds, then stop
rosrun radxa dshot_test /dev/spidev1.0 0.55 1000
# Optional fifth argument extends stop-frame arming for an ESC power cycle.
rosrun radxa dshot_test /dev/spidev1.0 0.55 1000 30000
# Stop-only arming check (two short tones after the startup melody are expected)
rosrun radxa dshot_arm_test /dev/spidev1.0 30000 1600000
# Non-rotating validation of a non-zero DShot packet (BEACON1)
rosrun radxa dshot_beacon_test /dev/spidev1.0

# One conventional PWM channel for three seconds
rosrun radxa pwm_test 10 0 500 0.5

# Servos 1-4 in order: step to 60 deg, sweep to 120 deg in 3 s, then all 90 deg
# Run this while crobat bridge.launch is active.
rosrun radxa servo_test.py

# Pure conversion/encoding tests (no hardware)
catkin run_tests radxa
catkin_test_results
```

The DShot ports do not use servo-style minimum/maximum pulse widths. In the
spinal-compatible normalized command, `0.5` or lower sends DShot stop (`0`),
values just above `0.5` start at the legal DShot throttle minimum (`48`), and
`1.0` maps to the maximum (`2047`). The packed 1.6 MHz waveform and its leading
low preamble were bench-verified with Bluejay 0.19.2 on a JH40 target.

Set the runtime direction of DShot ports 1–4 independently in
`config/cubie_a7z.yaml`:

```yaml
dshot_reversed: [false, true, false, true]
```

`false` sends Bluejay command 7 (absolute normal) and `true` sends command 8
(absolute reversed). The backend holds every motor at stop for one second on
each arm edge, sends ten direction-command frames, waits another 100 ms, and
only then permits throttle. It also refreshes the commands while disarmed so an
ESC powered after the ROS node can still receive its setting. These commands
change runtime RAM only and do not repeatedly write the ESC EEPROM.

Inspect ROS output with:

```bash
rostopic hz /crobat/imu
rostopic echo /crobat/battery_voltage_status
```

`set_adc_scale` retains spinal's meaning: battery volts per raw ADS1015 count.
For example, publish the currently calibrated value with
`rostopic pub -1 /crobat/set_adc_scale std_msgs/Float32 'data: 0.0121'`.
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
