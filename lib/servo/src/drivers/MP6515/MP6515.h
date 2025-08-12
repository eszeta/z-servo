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
#include "core/motor.h"

namespace hortor_servo {
namespace MP6515 {
/**
 * @brief MP6515电机驱动器
 * @details 使用两个PWM输出控制电机正反转
 */
class MP6515 final : public Motor {
 public:
  /**
   * @brief 初始化电机驱动器
   * @param pin_phase 相位引脚编号
   * @param pin_enbl 使能引脚编号
   * @param pin_brake 刹车引脚编号
   * @param pin_sleep 睡眠引脚编号
   */
  void Init(const uint8_t pin_phase,
            const uint8_t pin_enbl,
            const uint8_t pin_brake,
            const uint8_t pin_sleep);
  /**
   * @brief 设置PWM输出
   * @param pwm PWM值，范围为-1.0到1.0
   */
  void SetPWM(float pwm) override;

  /**
   * @brief 断电
   */
  void Break() override;

 private:
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
};
}  // namespace MP6515
}  // namespace hortor_servo