// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "base/motor.h"

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
class MP6515;
using MP6515Base = servo::Motor<MP6515>;
class MP6515 final : public MP6515Base {
 public:
  struct Config {
    uint8_t pin_phase;  // PHASE 相位引脚
    uint8_t pin_enbl;   // ENABLE 使能引脚（PWM）
    uint8_t pin_brake;  // BRAKE 制动引脚
    uint8_t pin_sleep;  // SLEEP 睡眠引脚
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
  Error Init(const Config& config) {
    if (config.pin_phase == 0 || config.pin_enbl == 0 ||
        config.pin_brake == 0 || config.pin_sleep == 0) {
      return Error::kInvalidParameter;
    }

    CHECK(MP6515Base::Init());

    pin_phase_ = config.pin_phase;
    pin_enbl_ = config.pin_enbl;
    pin_brake_ = config.pin_brake;
    pin_sleep_ = config.pin_sleep;
    // 配置引脚模式
    pinMode(pin_phase_, OUTPUT);
    pinMode(pin_enbl_, OUTPUT);
    pinMode(pin_brake_, OUTPUT);
    pinMode(pin_sleep_, OUTPUT);

    // 初始状态：滑行模式
    digitalWrite(pin_phase_, LOW);
    digitalWrite(pin_enbl_, LOW);
    digitalWrite(pin_brake_, LOW);
    digitalWrite(pin_sleep_, HIGH);  // 退出睡眠

    return Error::kOk;
  }

  /**
   * @brief 设置 PWM 输出
   * @param pwm PWM 值，范围为 -1.0 到 1.0
   */
  void SetPWMImpl(float pwm) {
    // 反转 PWM
    pwm = pwm * static_cast<int8_t>(reverse_);
    // 限制 PWM 范围
    pwm = constrain(pwm, -1.0f, 1.0f);

    // 关闭制动
    digitalWrite(pin_brake_, LOW);

    if (pwm > 0.0f) {
      // 正转：PHASE=1, ENABLE=PWM
      digitalWrite(pin_phase_, HIGH);
      analogWrite(pin_enbl_, static_cast<uint32_t>(255 * pwm));
    } else if (pwm < 0.0f) {
      // 反转：PHASE=0, ENABLE=PWM
      digitalWrite(pin_phase_, LOW);
      analogWrite(pin_enbl_, static_cast<uint32_t>(255 * (-pwm)));
    } else {
      // 滑行模式：ENABLE=0
      digitalWrite(pin_phase_, LOW);
      digitalWrite(pin_enbl_, LOW);
    }
  }

  /**
   * @brief 制动（快速停止）
   */
  void BrakeImpl() {
    // 制动模式：BRAKE=1
    digitalWrite(pin_brake_, HIGH);
  }

  /**
   * @brief 滑行（自由停止）
   */
  void CoastImpl() {
    // 滑行模式：BRAKE=0, ENABLE=0
    digitalWrite(pin_brake_, LOW);
    digitalWrite(pin_enbl_, LOW);
  }

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
