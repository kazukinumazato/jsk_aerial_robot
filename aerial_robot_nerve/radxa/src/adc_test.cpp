bool RadxaBoardIo::getVoltage(float& voltage)
{
  constexpr uint8_t adc_address = 0x48;

  constexpr uint8_t reg_conversion = 0x00;
  constexpr uint8_t reg_config = 0x01;

  constexpr uint8_t adc_channel = 0; // A0

  // Battery+ --- R1 --- ADC A0 --- R2 --- GND
  // divider_ratio = (R1 + R2) / R2
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

  // ADS1015 は 12bit ADC なので、上位12bitを使う
  raw16 >>= 4;

  // 12bit signed に変換
  if (raw16 & 0x0800) {
    raw16 |= 0xF000;
  }

  const int16_t raw = static_cast<int16_t>(raw16);

  const float adc_voltage =
    static_cast<float>(raw) * full_scale_voltage / 2048.0f;

  voltage = adc_voltage * divider_ratio;

  return true;
}
