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

#include "servo/motor.h"

namespace hortor::drivers::MP6515 {

/**
 * @brief MP6515 电机驱动器
 * @details
 * H 桥电机驱动器，使用 Phase/Enable 控制模式。
 * 支持正反转控制、PWM 调速、制动和滑行功能。
 *
 * 控制模式：
 * - 正转：PHASE=1, ENABLE=PWM
 * - 反转：PHASE=0, ENABLE=PWM
 * - 滑行：ENABLE=0, BRAKE=0
 * - 制动：BRAKE=1
 */
class MP6515 final : public servo::Motor<MP6515> {
 public:
  struct Config {
    uint8_t pin_phase;       // PHASE 相位引脚
    uint8_t pin_enbl;        // ENABLE 使能引脚（PWM）
    uint8_t pin_brake;       // BRAKE 制动引脚
    uint8_t pin_sleep;       // SLEEP 睡眠引脚
  };

  /**
   * @brief 构造函数
   */
  MP6515() = default;

  /**
   * @brief 初始化电机驱动器
   * @param config 配置参数
   * @return Error 错误码
   */
  Error Init(const Config& config);

  /**
   * @brief 设置 PWM 输出
   * @param pwm PWM 值，范围为 -1.0 到 1.0
   */
  void SetPWMImpl(float pwm);

  /**
   * @brief 制动（快速停止）
   */
  void BrakeImpl();

  /**
   * @brief 滑行（自由停止）
   */
  void CoastImpl();

 private:
  /** @brief PHASE 相位引脚（1: 正转, 0: 反转） */
  uint8_t pin_phase_ = 0;
  /** @brief ENABLE 使能引脚（PWM 控制速度） */
  uint8_t pin_enbl_ = 0;
  /** @brief BRAKE 制动引脚（1: 制动, 0: 不制动） */
  uint8_t pin_brake_ = 0;
  /** @brief SLEEP 睡眠引脚（1: 工作, 0: 睡眠） */
  uint8_t pin_sleep_ = 0;
};

}  // namespace hortor::drivers::MP6515
