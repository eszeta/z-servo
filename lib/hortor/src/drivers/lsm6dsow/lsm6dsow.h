// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

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
class LSM6DSOW final : public servo::IMU<LSM6DSOW> {
 public:
  /**
   * @brief 初始化I2C通信
   * @param wire I2C通信接口指针
   * @return 初始化结果
   */
  Error InitI2C(TwoWire* wire) {
    CHECK(regmap_.Init(wire, kI2CAddress));
    return Error::kOk;
  }

  /**
   * @brief 读取加速度数据
   * @param x 加速度X轴数据,单位：g
   * @param y 加速度Y轴数据,单位：g
   * @param z 加速度Z轴数据,单位：g
   * @return 读取结果
   */
  Error ReadAccelerationImpl(float& x, float& y, float& z) {
    return regmap_.ReadAcceleration(x, y, z);
  }

  /**
   * @brief 检查加速度数据是否可用
   * @return 是否可用
   */
  bool AccelerationAvailableImpl() { return regmap_.AccelerationAvailable(); }

  /**
   * @brief 读取陀螺仪数据
   * @param x 陀螺仪X轴数据,单位：度/秒
   * @param y 陀螺仪Y轴数据
   * @param z 陀螺仪Z轴数据
   * @return 读取结果
   */
  Error ReadGyroscopeImpl(float& x, float& y, float& z) {
    return regmap_.ReadGyroscope(x, y, z);
  }

  /**
   * @brief 检查陀螺仪数据是否可用
   * @return 是否可用
   */
  bool GyroscopeAvailableImpl() { return regmap_.GyroscopeAvailable(); }

  /**
   * @brief 读取温度数据
   * @param temperature_deg 温度数据
   * @return 读取结果
   */
  Error ReadTemperatureImpl(float& temperature_deg) {
    return regmap_.ReadTemperature(temperature_deg);
  }

  /**
   * @brief 检查温度数据是否可用
   * @return 是否可用
   */
  bool TemperatureAvailableImpl() { return regmap_.TemperatureAvailable(); }

 private:
  /** @brief 寄存器映射实例（包含I2C通信层），负责与传感器的具体通信操作 */
  RegMap regmap_;
};
}  // namespace hortor::drivers::LSM6DSOW