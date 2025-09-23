// Copyright 2025 ES_ZETA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <Arduino.h>

#include "core/current.h"

namespace hortor_servo {
namespace generic_current {
class GenericCurrent : public Current {
 public:
  /**
   * @brief 构造函数
   */
  GenericCurrent() = default;

  /**
   * @brief 初始化
   * @param pin_adc ADC引脚编号
   * @param shunt_resistor 分流电阻值
   * @param factor 转换因子
   */
  void Init(const uint8_t pin_adc,
            const uint16_t shunt_resistor,
            const float factor);
  /**
   * @brief 获取当前电流值
   * @return float 当前测量的电流值（安培）
   */
  float GetCurrent() override;

 private:
  /** @brief ADC电压转换系数 */
  constexpr static float kAdcVoltageConv = 3.3f / 1024.0f;
  /**
   * @brief 校准ADC零点偏移
   * @details 通过多次采样计算ADC的零点偏移值
   */
  void CalibrateOffsets();
  /**
   * @brief 读取ADC电压值
   * @return float ADC测量的电压值
   */
  float ReadADCVoltage();

  /** @brief ADC引脚编号 */
  uint8_t pin_adc_ = 0;
  /** @brief 增益系数 */
  float gain_ = 0.0f;
  /** @brief ADC修正值 */
  float offset_ = 0.0f;
};

}  // namespace generic_current
}  // namespace hortor_servo