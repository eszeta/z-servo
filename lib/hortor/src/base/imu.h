// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "hortor.h"
#include "types.h"

namespace hortor::servo {

/**
 * @brief IMU类
 * @details 定义了IMU的基本接口
 */
template <typename DERIVED>
class IMU {
 public:
  /**
   * @brief 读取加速度
   * @param x 加速度X轴,单位:g
   * @param y 加速度Y轴,单位:g
   * @param z 加速度Z轴,单位:g
   * @return IMUError 读取结果
   */
  Error ReadAcceleration(float& x, float& y, float& z) {
    return static_cast<DERIVED*>(this)->ReadAccelerationImpl(x, y, z);
  }

  /**
   * @brief 检查加速度是否可用
   * @return bool 加速度是否可用
   */
  bool AccelerationAvailable() {
    return static_cast<DERIVED*>(this)->AccelerationAvailableImpl();
  }

  /**
   * @brief 读取陀螺仪
   * @param x 陀螺仪X轴,单位:度/秒
   * @param y 陀螺仪Y轴,单位:度/秒
   * @param z 陀螺仪Z轴,单位:度/秒
   * @return IMUError 读取结果
   */
  Error ReadGyroscope(float& x, float& y, float& z) {
    return static_cast<DERIVED*>(this)->ReadGyroscopeImpl(x, y, z);
  }

  /**
   * @brief 检查陀螺仪是否可用
   * @return bool 陀螺仪是否可用
   */
  bool GyroscopeAvailable() {
    return static_cast<DERIVED*>(this)->GyroscopeAvailableImpl();
  }

  /**
   * @brief 读取温度
   * @param temperature_deg 温度
   * @return IMUError 读取结果
   */
  Error ReadTemperature(float& temperature_deg) {
    return static_cast<DERIVED*>(this)->ReadTemperatureImpl(temperature_deg);
  }

  /**
   * @brief 检查温度是否可用
   * @return bool 温度是否可用
   */
  bool TemperatureAvailable() {
    return static_cast<DERIVED*>(this)->TemperatureAvailableImpl();
  }
};

}  // namespace hortor::servo
