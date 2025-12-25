// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "hortor.h"
#include "types.h"

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

  /** @brief 电机反转 */
  Reverse reverse_ = Reverse::kNormal;

 public:
  /**
   * @brief 初始化电机驱动器
   * @param reverse 电机反转
   */
  Error Init() {
    return Error::kOk;
  }
  /**
   * @brief 设置电机PWM
   * @param pwm (-1..1)
   */
  void SetPWM(float pwm) {
    return AsDerived().SetPWMImpl(pwm * static_cast<int8_t>(reverse_));
  }

  /**
   * @brief 制动（快速停止）
   */
  void Brake() { return AsDerived().BrakeImpl(); }

  /**
   * @brief 滑行（自由停止）
   */
  void Coast() { return AsDerived().CoastImpl(); }

  /**
   * @brief 获取电机反转
   * @return 电机反转
   */
  Reverse GetReverse() const { return reverse_; }
  /**
   * @brief 设置电机反转
   * @param reverse 电机反转
   */
  void SetReverse(const Reverse reverse) { reverse_ = reverse; }
};
}  // namespace hortor::servo
