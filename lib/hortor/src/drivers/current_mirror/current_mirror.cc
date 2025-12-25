// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include "current_mirror.h"

namespace hortor::drivers::current_mirror {

Error CurrentMirror::Init(const Config& config) {
  if (config.pin_adc == 0) {
    return Error::kInvalidParameter;
  }
  pin_adc_ = config.pin_adc;
  calibration_samples_ = config.calibration_samples;

  pinMode(pin_adc_, INPUT);

  // 计算 ADC → 电压转换系数
  // 例如：12-bit ADC, 3.3V → 3.3 / 4095 = 0.000806 V/LSB
  const uint32_t adc_max_value = (1 << config.adc_resolution_bits) - 1;
  adc_to_voltage_ = config.adc_vref_volts / static_cast<float>(adc_max_value);

  // 计算 电压 → 电流转换系数
  // I_load = V_IPROPI / (R_IPROPI × current_sense_ratio)
  // current_sense_ratio 需要从 μA/A 转换为 A/A
  const float current_sense_ratio = config.scaling_factor / 1e6f;
  voltage_to_current_ = 1.0f / (config.ripropi_ohms * current_sense_ratio);

  // 示例：R=1000Ω, ratio=1500μA/A=0.0015
  // voltage_to_current_ = 1.0 / (1000 × 0.0015) = 0.6667 A/V

  // 零点校准
  CHECK(CalibrateOffsets());

  return Error::kOk;
}

Error CurrentMirror::CalibrateOffsets() {
  zero_offset_voltage_ = 0.0f;

  for (uint16_t i = 0; i < calibration_samples_; ++i) {
    float voltage;
    CHECK(ReadADCVoltage(voltage));
    zero_offset_voltage_ += voltage;
    delay(2);  // 2ms × 50 = 100ms 总校准时间
  }

  zero_offset_voltage_ /= static_cast<float>(calibration_samples_);
  return Error::kOk;
}

Error CurrentMirror::GetCurrentImpl(float& current) {
  float voltage;
  CHECK(ReadADCVoltage(voltage));

  // 去除零点偏移后计算电流
  const float voltage_diff = voltage - zero_offset_voltage_;
  current = voltage_diff * voltage_to_current_;

  return Error::kOk;
}

Error CurrentMirror::ReadADCVoltage(float& voltage) {
  const uint16_t adc_raw = analogRead(pin_adc_);
  voltage = static_cast<float>(adc_raw) * adc_to_voltage_;
  return Error::kOk;
}

}  // namespace hortor::drivers::current_mirror