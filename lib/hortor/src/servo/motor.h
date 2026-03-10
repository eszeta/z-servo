// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "servo/types.h"

namespace hortor::servo {

/**
 * @brief 电机驱动器抽象基类
 * @details
 * 所有具体的电机驱动器实现都必须继承自此类。该类提供了电机驱动器的基本接口定义。
 */
template <typename DerivedType>
class Motor : public hortor::Noncopyable {
 public:
  /**
   * @brief 获取电机反转
   * @return 电机反转
   */
  Reverse reverse() const;

  /**
   * @brief 设置电机反转
   * @param reverse 电机反转
   */
  void set_reverse(const Reverse reverse);

  /**
   * @brief 初始化电机驱动器
   * @param reverse 电机反转
   */
  Error Init();

  /**
   * @brief 设置电机PWM
   * @param pwm (-1..1)
   */
  void SetPWM(float pwm);

  /**
   * @brief 制动（快速停止）
   */
  void Brake();

  /**
   * @brief 滑行（自由停止）
   */
  void Coast();

 protected:
  /** @brief 电机反转 */
  Reverse reverse_ = Reverse::kNormal;
};

}  // namespace hortor::servo

namespace hortor::servo {

template <typename DerivedType>
Reverse Motor<DerivedType>::reverse() const {
  return reverse_;
}

template <typename DerivedType>
void Motor<DerivedType>::set_reverse(const Reverse reverse) {
  reverse_ = reverse;
}

template <typename DerivedType>
Error Motor<DerivedType>::Init() {
  return Error::kOk;
}

template <typename DerivedType>
void Motor<DerivedType>::SetPWM(float pwm) {
  static_cast<DerivedType*>(this)->SetPWMImpl(pwm);
}

template <typename DerivedType>
void Motor<DerivedType>::Brake() {
  static_cast<DerivedType*>(this)->BrakeImpl();
}

template <typename DerivedType>
void Motor<DerivedType>::Coast() {
  static_cast<DerivedType*>(this)->CoastImpl();
}

}  // namespace hortor::servo
