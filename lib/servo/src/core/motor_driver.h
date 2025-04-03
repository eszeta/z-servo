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

#include "./object_interface.h"

namespace hortor_servo {
/**
 * @brief 电机驱动器抽象基类
 * @details
 * 所有具体的电机驱动器实现都必须继承自此类。该类提供了电机驱动器的基本接口定义。
 */
class MotorDriver : public ObjectInterface {
 public:
  /**
   * @brief 设置电机PWM
   * @param pwm (-1..1)
   */
  virtual void SetPWM(float pwm) = 0;

  /**
   * @brief 断电
   */
  virtual void Break() = 0;
};
}  // namespace hortor_servo
