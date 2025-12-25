// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "hortor.h"
#include "types.h"

namespace hortor::servo {

/**
 * @brief 电流传感器抽象基类
 * @details
 * 所有具体的电流传感器实现都必须继承自此类。该类提供了电流检测的基本接口定义。
 */
template <typename Derived>
class Current {
 protected:
  /**
   * @brief 获取派生类引用
   * @return 派生类引用
   */
  Derived& AsDerived() { return static_cast<Derived&>(*this); }

  /**
   * @brief 获取派生类常量引用
   * @return 派生类常量引用
   */
  const Derived& AsDerived() const {
    return static_cast<const Derived&>(*this);
  }

 public:
  /**
   * @brief 获取当前电流读数
   * @param current 以安培(A)为单位的电流值
   * @return Error 错误码
   */
  Error GetCurrent(float& current) {
    return AsDerived().GetCurrentImpl(current);
  }

  /**
   * @brief 读取并平均多次电流采样值
   * @param n 采样次数
   * @param current 平均电流值
   * @return Error 错误码
   */
  Error ReadAverageCurrents(int n, float& current) {
    CHECK(GetCurrent(current));
    for (int i = 0; i < n; ++i) {
      float new_current;
      CHECK(GetCurrent(new_current));
      current = current * 0.6f + 0.4f * new_current;
      delay(3);
    }
    return Error::kOk;
  }
};

}  // namespace hortor::servo
