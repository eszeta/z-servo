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

#include "../servo_types.h"
#include "./object_interface.h"

namespace hortor_servo {
/**
 * @brief IMU类
 * @details 定义了IMU的基本接口
 */
class IMU : public ObjectInterface {
 public:
  /**
   * @brief 读取加速度
   * @param x 加速度X轴,单位:g
   * @param y 加速度Y轴,单位:g
   * @param z 加速度Z轴,单位:g
   * @return IMUError 读取结果
   */
  virtual Error ReadAcceleration(float* x, float* y, float* z) = 0;

  /**
   * @brief 检查加速度是否可用
   * @return bool 加速度是否可用
   */
  virtual bool AccelerationAvailable() = 0;

  /**
   * @brief 读取陀螺仪
   * @param x 陀螺仪X轴,单位:度/秒
   * @param y 陀螺仪Y轴,单位:度/秒
   * @param z 陀螺仪Z轴,单位:度/秒
   * @return IMUError 读取结果
   */
  virtual Error ReadGyroscope(float* x, float* y, float* z) = 0;

  /**
   * @brief 检查陀螺仪是否可用
   * @return bool 陀螺仪是否可用
   */
  virtual bool GyroscopeAvailable() = 0;

  /**
   * @brief 读取温度
   * @param temperature_deg 温度
   * @return IMUError 读取结果
   */
  virtual Error ReadTemperature(float* temperature_deg) = 0;

  /**
   * @brief 检查温度是否可用
   * @return bool 温度是否可用
   */
  virtual bool TemperatureAvailable() = 0;
};
}  // namespace hortor_servo
