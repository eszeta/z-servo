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

#include "servo/current.h"
#include "servo/motor.h"

namespace hortor::drivers::DRV8231A {
/**
 * @brief DRV8231A电机驱动器
 * @details 使用两个PWM输出控制电机正反转
 */
class DRV8231A final : public servo::Motor {
 public:
  /**
   * @brief 构造函数
   */
  DRV8231A() = default;

  /**
   * @brief 初始化电机驱动器
   * @param pin_a 正转PWM输出引脚
   * @param pin_b 反转PWM输出引脚
   */
  void Init(const uint8_t pin_a, const uint8_t pin_b);

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
  /** @brief PWM A输出引脚 */
  uint8_t pin_a_ = 0;
  /** @brief PWM B输出引脚 */
  uint8_t pin_b_ = 0;
};

}  // namespace hortor::drivers::DRV8231A