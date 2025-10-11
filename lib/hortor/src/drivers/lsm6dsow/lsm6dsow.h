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

#include "regmap.h"
#include "regmap/regmap_i2c_bus.h"
#include "servo/imu.h"
#include "types.h"

namespace hortor::drivers::LSM6DSOW {

/**
 * @brief LSM6DSOW传感器类，继承自IMU类，提供与LSM6DSOW传感器的接口
 */
class LSM6DSOW final : public servo::IMU {
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
  Error ReadAcceleration(float& x, float& y, float& z) override {
    return regmap_.ReadAcceleration(x, y, z);
  }

  /**
   * @brief 检查加速度数据是否可用
   * @return 是否可用
   */
  bool AccelerationAvailable() override {
    return regmap_.AccelerationAvailable();
  }

  /**
   * @brief 读取陀螺仪数据
   * @param x 陀螺仪X轴数据,单位：度/秒
   * @param y 陀螺仪Y轴数据
   * @param z 陀螺仪Z轴数据
   * @return 读取结果
   */
  Error ReadGyroscope(float& x, float& y, float& z) override {
    return regmap_.ReadGyroscope(x, y, z);
  }

  /**
   * @brief 检查陀螺仪数据是否可用
   * @return 是否可用
   */
  bool GyroscopeAvailable() override { return regmap_.GyroscopeAvailable(); }

  /**
   * @brief 读取温度数据
   * @param temperature_deg 温度数据
   * @return 读取结果
   */
  Error ReadTemperature(float& temperature_deg) override {
    return regmap_.ReadTemperature(temperature_deg);
  }

  /**
   * @brief 检查温度数据是否可用
   * @return 是否可用
   */
  bool TemperatureAvailable() override {
    return regmap_.TemperatureAvailable();
  }

 private:
  /** @brief 寄存器映射实例，负责与传感器的具体通信操作 */
  RegMap regmap_;
  /** @brief I2C通信实例，提供底层I2C接口 */
  regmap::RegMapI2CBus i2c_transport_;
};
}  // namespace hortor::drivers::LSM6DSOW