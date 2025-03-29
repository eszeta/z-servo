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

#include "../../core/current.h"
#include "../../core/motor_driver.h"

namespace hortor_servo {
namespace MP6515 {
/**
 * @brief MP6515电机驱动器
 * @details 使用两个PWM输出控制电机正反转
 */
class MP6515 final : public MotorDriver, public Current {
 public:
  /**
   * @brief 初始化电机驱动器
   * @param pin_phase 相位引脚编号
   * @param pin_enbl 使能引脚编号
   * @param pin_brake 刹车引脚编号
   * @param pin_sleep 睡眠引脚编号
   */
  void InitDriver(const uint8_t pin_phase, const uint8_t pin_enbl, const uint8_t pin_brake, const uint8_t pin_sleep);

  /**
   * @brief 使用指定参数初始化电流传感器
   * @param shunt_resistor 分流电阻值（欧姆）
   * @param factor 电流镜像比例因子（µA/A）
   */
  void InitCurrentSensor(const uint8_t pin_adc, const uint16_t shunt_resistor, const uint16_t factor);

  /**
   * @brief 设置PWM输出
   * @param pwm PWM值，范围为-1.0到1.0
   */
  void SetPWM(float pwm) override;

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

  /**
   * @brief 刹车引脚编号
   * 1: 刹车 0: 不刹车
   */
  uint8_t pin_brake_;
  /**
   * @brief 相位引脚编号
   * 1: 正转   0: 反转
   */
  uint8_t pin_phase_;

  /**
   * @brief 使能引脚编号
   */
  uint8_t pin_enbl_;

  /**
   * @brief 睡眠引脚编号
   * 1: 工作 0: 睡眠
   */
  uint8_t pin_sleep_;

  /**
   * @brief 电流检测引脚编号
   */
  uint8_t pin_visen_;
  /**
   * @brief 增益系数
   */
  float gain_;
  /**
   * @brief ADC偏移量
   */
  float offset_;
};
}  // namespace MP6515
}  // namespace hortor_servo