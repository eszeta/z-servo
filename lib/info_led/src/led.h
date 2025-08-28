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
namespace hortor_servo {
namespace InfoLED {

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
}  // namespace InfoLED
}  // namespace hortor_servo
