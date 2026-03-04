// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "base/current.h"

namespace hortor::drivers::current_mirror {

class CurrentMirror : public servo::Current<CurrentMirror> {
 public:
  struct Config {
    uint8_t pin_adc;               // ADC 引脚
    float ripropi_ohms;            // RIPROPI 电阻值（欧姆）
    float scaling_factor;          // 电流镜系数（μA/A）
    uint16_t adc_resolution_bits;  // ADC 分辨率（位）
    float adc_vref_volts;          // ADC 参考电压（伏特）
    uint16_t calibration_samples;  // 校准采样次数
  };

  /**
   * @brief 构造函数
   */
  CurrentMirror() = default;

  /**
   * @brief 初始化电流镜传感器
   * @param config 配置参数
   * @return Error 错误码
   */
  Error Init(const Config& config) {
    if (config.pin_adc == 0) {
      return Error::kInvalidArg;
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

  /**
   * @brief 获取当前电流值
   * @param current 当前测量的电流值（安培）
   * @return Error 错误码
   */
  Error ReadCurrentImpl(float& current) {
    float voltage;
    CHECK(ReadADCVoltage(voltage));

    // 去除零点偏移后计算电流
    const float voltage_diff = voltage - zero_offset_voltage_;
    current = voltage_diff * voltage_to_current_;

    return Error::kOk;
  }

 private:
  /** @brief ADC电压转换系数 */
  static constexpr float kAdcVoltageConv = 3.3f / 1024.0f;
  /**
   * @brief 校准ADC零点偏移
   * @details 通过多次采样计算ADC的零点偏移值
   */
  Error CalibrateOffsets() {
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
  /**
   * @brief 读取ADC电压值
   * @return float ADC测量的电压值
   */
  Error ReadADCVoltage(float& voltage) {
    const uint16_t adc_raw = analogRead(pin_adc_);
    voltage = static_cast<float>(adc_raw) * adc_to_voltage_;
    return Error::kOk;
  }

  /** @brief ADC原始值到电压的转换系数 */
  float adc_to_voltage_ = 0.0f;
  /** @brief 电压到电流的转换系数 */
  float voltage_to_current_ = 0.0f;
  /** @brief 零点偏移电压 */
  float zero_offset_voltage_ = 0.0f;

  /** @brief ADC引脚编号 */
  uint8_t pin_adc_ = 0;
  /** @brief 校准采样次数 */
  uint16_t calibration_samples_ = 50;
};

}  // namespace hortor::drivers::current_mirror