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

#include "math/pid.h"

namespace hortor::servo {

/**
 * @brief 电机控制类型
 */
enum class ServoMode : uint8_t {
  /**
   * @brief 位置伺服模式
   */
  kPosition = 0,
  /**
   * @brief 恒速模式
   */
  kVelocity = 1,
  /**
   * @brief PWM开环调速度模式
   */
  kPwm = 2,
  /**
   * @brief 步进伺服模式
   */
  kStep = 3
};

/**
 * @brief 电机状态位
 */
namespace ServoStatusMask {
static constexpr uint8_t kVoltage = 1 << 0;
static constexpr uint8_t kSensor = 1 << 1;
static constexpr uint8_t kTemperature = 1 << 2;
static constexpr uint8_t kCurrent = 1 << 3;
static constexpr uint8_t kLoad = 1 << 5;
};  // namespace ServoStatusMask

/**
 * @brief 方向
 */
enum class Direction : int8_t {
  CW = 1,      // clockwise
  CCW = -1,    // counterclockwise
  unknown = 0  // not yet known or invalid state
};

}  // namespace hortor::servo