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

namespace hortor::drivers::DRV8231A {

/**
 * @brief DRV8231A 电机驱动器
 * @details
 * H 桥电机驱动器，支持正反转控制、PWM 调速和制动功能。
 * 集成了过流保护、热关断和欠压锁定保护。
 *
 * 控制模式（自适应衰减）：
 *
 * 低速模式（慢速衰减）：
 * - 正转：IN1=1, IN2=PWM反相 → 电流缓慢衰减，适合低速大扭矩
 * - 反转：IN1=PWM反相, IN2=1 → 电流缓慢衰减，适合低速大扭矩
 *
 * 高速模式（快速衰减）：
 * - 正转：IN1=PWM, IN2=0 → 电流快速衰减，响应快
 * - 反转：IN1=0, IN2=PWM → 电流快速衰减，响应快
 *
 * 其他模式：
 * - 滑行：IN1=0, IN2=0 → 高阻态
 * - 制动：IN1=1, IN2=1 → 低边导通
 */
class DRV8231A final : public servo::Motor<DRV8231A> {
 public:
  struct Config {
    uint8_t pin_in1;                    // IN1 控制引脚
    uint8_t pin_in2;                    // IN2 控制引脚
    uint8_t pin_nfault = 0;             // nFAULT 引脚（0 表示不使用）
    float slow_decay_threshold = 0.3f;  // 低速阈值（低于此值使用慢速衰减）
    servo::Reverse direction;         // 电机方向
  };

  /**
   * @brief 构造函数
   */
  DRV8231A() = default;

  /**
   * @brief 初始化电机驱动器
   * @param config 配置参数
   * @return Error 错误码
   */
  Error Init(const Config& config);

  /**
   * @brief 设置 PWM 输出
   * @param pwm PWM 值，范围为 -1.0 到 1.0
   * @details
   * 根据 PWM 绝对值自动选择衰减模式：
   * - |pwm| < slow_decay_threshold：慢速衰减（适合低速）
   * - |pwm| >= slow_decay_threshold：快速衰减（适合高速）
   */
  void SetPWMImpl(float pwm);

  /**
   * @brief 制动（低边导通）
   * @details IN1=1, IN2=1，电机快速停止
   */
  void BrakeImpl();

  /**
   * @brief 滑行（高阻态）
   * @details IN1=0, IN2=0，电机自由滑行
   */
  void CoastImpl();
  /**
   * @brief 检查是否有故障
   * @return true 如果检测到故障，false 否则
   */
  bool HasFault() const;

 private:
  /**
   * @brief 快速衰减模式设置 PWM
   * @param pwm PWM 值（-1.0 到 1.0）
   */
  void SetPWMFastDecay(float pwm);

  /**
   * @brief 慢速衰减模式设置 PWM
   * @param pwm PWM 值（-1.0 到 1.0）
   */
  void SetPWMSlowDecay(float pwm);

  /** @brief IN1 控制引脚 */
  uint8_t pin_in1_ = 0;
  /** @brief IN2 控制引脚 */
  uint8_t pin_in2_ = 0;
  /** @brief nFAULT 引脚 */
  uint8_t pin_nfault_ = 0;
  /** @brief 慢速衰减阈值 */
  float slow_decay_threshold_ = 0.3f;
};

}  // namespace hortor::drivers::DRV8231A
