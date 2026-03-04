// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

namespace hortor::info_led {

/**
 * @brief 模式枚举
 */
enum class LedMode {
  /**
   * @brief 开漏输出
   */
  kOpenDrain = 0,
  /**
   * @brief 推挽输出
   */
  kPushPull = 1,
};

/**
 * @brief LED类
 * @tparam M 输出模式（编译期确定，零运行时分支）
 */
template <LedMode M>
class LED final {
 public:
  /**
   * @brief 初始化
   */
  void Init(uint32_t pin);
  /**
   * @brief 初始化
   * @param pinName 引脚名
   */
  void Init(PinName pinName);
  /**
   * @brief 设置状态
   * @param value 状态
   */
  void Turn(bool value);
  /**
   * @brief 切换状态
   */
  void Toggle();

 private:
  /**
   * @brief 状态
   */
  bool state_ = false;
  /**
   * @brief 引脚
   */
  PinName pin_name_ = NC;
};

template <LedMode M>
void LED<M>::Init(uint32_t pin) {
  Init(digitalPinToPinName(pin));
}

template <LedMode M>
void LED<M>::Init(PinName pinName) {
  pin_name_ = pinName;
  if constexpr (M == LedMode::kOpenDrain) {
    pinMode(pinName, OUTPUT_OPEN_DRAIN);
  } else {
    pinMode(pinName, OUTPUT);
  }
  Turn(false);
}

template <LedMode M>
void LED<M>::Turn(bool value) {
  if (value == state_) {
    return;
  }
  state_ = value;
  if constexpr (M == LedMode::kOpenDrain) {
    digitalWrite(pin_name_, value ? LOW : HIGH);
  } else {
    digitalWrite(pin_name_, value ? HIGH : LOW);
  }
}

template <LedMode M>
void LED<M>::Toggle() {
  Turn(!state_);
}

}  // namespace hortor::info_led
