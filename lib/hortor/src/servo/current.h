// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "error.h"
#include "noncopyable.h"
#include "servo/types.h"

namespace hortor::servo {

/**
 * @brief 电流传感器抽象基类
 * @details
 * 所有具体的电流传感器实现都必须继承自此类。该类提供了电流检测的基本接口定义。
 */
template <typename DerivedType>
class Current : public hortor::Noncopyable {
 public:
  /**
   * @brief 初始化电流传感器
   * @return Error 错误码
   */
  Error Init();
  /**
   * @brief 获取当前电流读数
   * @param current 以安培(A)为单位的电流值
   * @return Error 错误码
   */
  Error ReadCurrent(float& current);

  /**
   * @brief 读取并平均多次电流采样值
   * @param n 采样次数
   * @param current 平均电流值
   * @return Error 错误码
   */
  Error ReadAverageCurrents(int n, float& current);
};

}  // namespace hortor::servo

namespace hortor::servo {

template <typename DerivedType>
Error Current<DerivedType>::Init() {
  return Error::kOk;
}

template <typename DerivedType>
Error Current<DerivedType>::ReadCurrent(float& current) {
  return static_cast<DerivedType*>(this)->ReadCurrentImpl(current);
}

template <typename DerivedType>
Error Current<DerivedType>::ReadAverageCurrents(int n, float& current) {
  CHECK(ReadCurrent(current));
  for (int i = 0; i < n; ++i) {
    float new_current;
    CHECK(ReadCurrent(new_current));
    current = current * 0.6f + 0.4f * new_current;
    delay(3);
  }
  return Error::kOk;
}

}  // namespace hortor::servo
