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

#include "hortor.h"

namespace hortor::servo {

/**
 * @brief 电机驱动器抽象基类
 * @details
 * 所有具体的电机驱动器实现都必须继承自此类。该类提供了电机驱动器的基本接口定义。
 */
template <typename Derived>
class Motor {
 protected:
  Derived& AsDerived() { return static_cast<Derived&>(*this); }

  const Derived& AsDerived() const {
    return static_cast<const Derived&>(*this);
  }

 public:
  /**
   * @brief 设置电机PWM
   * @param pwm (-1..1)
   */
  void SetPWM(float pwm) { return AsDerived().SetPWMImpl(pwm); }

  /**
   * @brief 制动（快速停止）
   */
  void Brake() { return AsDerived().BrakeImpl(); }

  /**
   * @brief 滑行（自由停止）
   */
  void Coast() { return AsDerived().CoastImpl(); }
};
}  // namespace hortor::servo
