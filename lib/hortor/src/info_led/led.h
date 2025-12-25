// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

namespace hortor::info_led {

/**
 * @brief 状态枚举
 */
enum class State { kOff = 0, kOn = 1 };

/**
 * @brief 模式枚举
 */
enum class Mode {
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
 */
class LED final {
 public:
  /**
   * @brief 初始化
   */
  void Init(uint32_t pin, Mode mode = Mode::kOpenDrain);
  /**
   * @brief 初始化
   * @param pinName 引脚名
   * @param mode 模式
   */
  void Init(PinName pinName, Mode mode = Mode::kOpenDrain);
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
  State state_ = State::kOff;
  /**
   * @brief 引脚
   */
  PinName pin_name_ = NC;
  /**
   * @brief 模式
   */
  Mode mode_ = Mode::kOpenDrain;
};

}  // namespace hortor::info_led
