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
 public:
  /**
   * @brief 获取电机反转
   * @return 电机反转
   */
  Reverse reverse() const { return reverse_; }
  /**
   * @brief 设置电机反转
   * @param reverse 电机反转
   */
  void set_reverse(const Reverse reverse) { reverse_ = reverse; }

  /**
   * @brief 初始化电机驱动器
   * @param reverse 电机反转
   */
  Error Init() { return Error::kOk; }
  /**
   * @brief 设置电机PWM
   * @param pwm (-1..1)
   */
  void SetPWM(float pwm) { static_cast<Derived*>(this)->SetPWMImpl(pwm); }

  /**
   * @brief 制动（快速停止）
   */
  void Brake() { static_cast<Derived*>(this)->BrakeImpl(); }

  /**
   * @brief 滑行（自由停止）
   */
  void Coast() { static_cast<Derived*>(this)->CoastImpl(); }

 protected:
  /** @brief 电机反转 */
  Reverse reverse_ = Reverse::kNormal;
};
}  // namespace hortor::servo
