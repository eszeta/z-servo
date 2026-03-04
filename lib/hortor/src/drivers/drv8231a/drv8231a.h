// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "base/motor.h"

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
class DRV8231A;
using DRV8231ABase = servo::Motor<DRV8231A>;
class DRV8231A final : public DRV8231ABase {
 public:
  struct Config {
    uint8_t pin_in1;                    // IN1 控制引脚
    uint8_t pin_in2;                    // IN2 控制引脚
    uint8_t pin_nfault = 0;             // nFAULT 引脚（0 表示不使用）
    float slow_decay_threshold = 0.3f;  // 低速阈值（低于此值使用慢速衰减）
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
  Error Init(const Config& config) {
    VERIFY(config.pin_in1 != 0 && config.pin_in2 != 0, Error::kInvalidArg);

    CHECK(DRV8231ABase::Init());

    pin_in1_ = config.pin_in1;
    pin_in2_ = config.pin_in2;
    pin_nfault_ = config.pin_nfault;
    slow_decay_threshold_ = config.slow_decay_threshold;

    // 配置 IN1 和 IN2 为输出
    pinMode(pin_in1_, OUTPUT);
    pinMode(pin_in2_, OUTPUT);

    // 配置 nFAULT 为输入（如果使用）
    if (pin_nfault_ != 0) {
      pinMode(pin_nfault_, INPUT_PULLUP);
    }

    // 初始状态：滑行模式
    digitalWrite(pin_in1_, LOW);
    digitalWrite(pin_in2_, LOW);

    return Error::kOk;
  }

  /**
   * @brief 设置 PWM 输出
   * @param pwm PWM 值，范围为 -1.0 到 1.0
   * @details
   * 根据 PWM 绝对值自动选择衰减模式：
   * - |pwm| < slow_decay_threshold：慢速衰减（适合低速）
   * - |pwm| >= slow_decay_threshold：快速衰减（适合高速）
   */
  void SetPWMImpl(float pwm) {
    // 反转 PWM
    pwm = pwm * static_cast<int8_t>(reverse_);
    // 限制 PWM 范围
    pwm = constrain(pwm, -1.0f, 1.0f);

    // 根据 PWM 绝对值选择衰减模式
    const float abs_pwm = abs(pwm);

    if (abs_pwm < slow_decay_threshold_) {
      // 低速：使用慢速衰减模式
      SetPWMSlowDecay(pwm);
    } else {
      // 高速：使用快速衰减模式
      SetPWMFastDecay(pwm);
    }
  }

  /**
   * @brief 制动（低边导通）
   * @details IN1=1, IN2=1，电机快速停止
   */
  void BrakeImpl() {
    // 制动模式：IN1=1, IN2=1（低边导通）
    digitalWrite(pin_in1_, HIGH);
    digitalWrite(pin_in2_, HIGH);
  }

  /**
   * @brief 滑行（高阻态）
   * @details IN1=0, IN2=0，电机自由滑行
   */
  void CoastImpl() {
    // 滑行模式：IN1=0, IN2=0（高阻态）
    digitalWrite(pin_in1_, LOW);
    digitalWrite(pin_in2_, LOW);
  }

  /**
   * @brief 检查是否有故障
   * @return true 如果检测到故障，false 否则
   */
  bool HasFault() const {
    if (pin_nfault_ == 0) {
      return false;  // 未配置故障检测引脚
    }
    // nFAULT 为低电平时表示有故障
    return digitalRead(pin_nfault_) == LOW;
  }

 private:
  /**
   * @brief 快速衰减模式设置 PWM
   * @param pwm PWM 值（-1.0 到 1.0）
   */
  void SetPWMFastDecay(float pwm) {
    if (pwm > 0.0f) {
      // 正向驱动：IN1=PWM, IN2=0（快速衰减）
      analogWrite(pin_in1_, static_cast<uint32_t>(255 * pwm));
      digitalWrite(pin_in2_, LOW);
    } else if (pwm < 0.0f) {
      // 反向驱动：IN1=0, IN2=PWM（快速衰减）
      digitalWrite(pin_in1_, LOW);
      analogWrite(pin_in2_, static_cast<uint32_t>(255 * (-pwm)));
    } else {
      // 滑行模式：IN1=0, IN2=0
      digitalWrite(pin_in1_, LOW);
      digitalWrite(pin_in2_, LOW);
    }
  }

  /**
   * @brief 慢速衰减模式设置 PWM
   * @param pwm PWM 值（-1.0 到 1.0）
   */
  void SetPWMSlowDecay(float pwm) {
    if (pwm > 0.0f) {
      // 正向驱动：IN1=1, IN2=PWM反相（慢速衰减）
      // PWM 高时制动，PWM 低时驱动，电流缓慢衰减
      digitalWrite(pin_in1_, HIGH);
      analogWrite(pin_in2_, static_cast<uint32_t>(255 * (1.0f - pwm)));
    } else if (pwm < 0.0f) {
      // 反向驱动：IN1=PWM反相, IN2=1（慢速衰减）
      // PWM 高时制动，PWM 低时驱动，电流缓慢衰减
      analogWrite(pin_in1_, static_cast<uint32_t>(255 * (1.0f + pwm)));
      digitalWrite(pin_in2_, HIGH);
    } else {
      // 滑行模式：IN1=0, IN2=0
      digitalWrite(pin_in1_, LOW);
      digitalWrite(pin_in2_, LOW);
    }
  }

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
