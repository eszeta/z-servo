// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "servo/current.h"

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
  Error Init(const Config& config);
  /**
   * @brief 获取当前电流值
   * @param current 当前测量的电流值（安培）
   * @return Error 错误码
   */
  Error GetCurrentImpl(float& current);

 private:
  /** @brief ADC电压转换系数 */
  constexpr static float kAdcVoltageConv = 3.3f / 1024.0f;
  /**
   * @brief 校准ADC零点偏移
   * @details 通过多次采样计算ADC的零点偏移值
   */
  Error CalibrateOffsets();
  /**
   * @brief 读取ADC电压值
   * @return float ADC测量的电压值
   */
  Error ReadADCVoltage(float& voltage);

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