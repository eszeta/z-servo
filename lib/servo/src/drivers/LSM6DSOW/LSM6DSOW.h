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
#include <Wire.h>

#include "LSM6DSOW_accessor.h"
#include "LSM6DSOW_types.h"
#include "core/imu.h"
#include "register/register_accessor.h"
#include "register/register_i2c_adapter.h"

namespace hortor_servo {
namespace LSM6DSOW {

/**
 * @brief LSM6DSOW传感器类，继承自IMU类，提供与LSM6DSOW传感器的接口
 */
class LSM6DSOW final : public IMU {
 public:
  /**
   * @brief 默认构造函数
   */
  explicit LSM6DSOW() = default;

  /**
   * @brief 初始化I2C通信
   * @param wire I2C通信接口指针
   * @return 初始化结果
   */
  Error InitI2C(TwoWire* wire);

  /**
   * @brief 读取加速度数据
   * @param x 加速度X轴数据,单位：g
   * @param y 加速度Y轴数据,单位：g
   * @param z 加速度Z轴数据,单位：g
   * @return 读取结果
   */
  Error ReadAcceleration(float* x, float* y, float* z) override {
    return accessor_.ReadAcceleration(x, y, z);
  }

  /**
   * @brief 检查加速度数据是否可用
   * @return 是否可用
   */
  bool AccelerationAvailable() override {
    return accessor_.AccelerationAvailable();
  }

  /**
   * @brief 读取陀螺仪数据
   * @param x 陀螺仪X轴数据,单位：度/秒
   * @param y 陀螺仪Y轴数据
   * @param z 陀螺仪Z轴数据
   * @return 读取结果
   */
  Error ReadGyroscope(float* x, float* y, float* z) override {
    return accessor_.ReadGyroscope(x, y, z);
  }

  /**
   * @brief 检查陀螺仪数据是否可用
   * @return 是否可用
   */
  bool GyroscopeAvailable() override { return accessor_.GyroscopeAvailable(); }

  /**
   * @brief 读取温度数据
   * @param temperature_deg 温度数据
   * @return 读取结果
   */
  Error ReadTemperature(float* temperature_deg) override {
    return accessor_.ReadTemperature(temperature_deg);
  }

  /**
   * @brief 检查温度数据是否可用
   * @return 是否可用
   */
  bool TemperatureAvailable() override {
    return accessor_.TemperatureAvailable();
  }

 private:
  /** @brief 访问器实例，负责与传感器的具体通信操作 */
  LSM6DSOW_accessor accessor_;
  /** @brief I2C通信实例，提供底层I2C接口 */
  RegisterI2CAdapter i2c_transport_;
};
}  // namespace LSM6DSOW
}  // namespace hortor_servo