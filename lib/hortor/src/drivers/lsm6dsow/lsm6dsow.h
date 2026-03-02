// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "base/imu.h"
#include "regmap.h"
#include "types.h"

namespace hortor::drivers::LSM6DSOW {

/**
 * @brief LSM6DSOW传感器类，继承自IMU类，提供与LSM6DSOW传感器的接口
 */
class LSM6DSOW;
using LSM6DSOWBase = servo::IMU<LSM6DSOW>;
class LSM6DSOW final : public LSM6DSOWBase {
 public:
  /**
 * @brief LSM6DSOW传感器I2C地址
 * @note 默认地址为0x6A
 */
  static constexpr uint8_t kI2CAddress = 0x6A;

  struct Config {
    TwoWire* wire;
  };

  /**
   * @brief 初始化I2C通信
   * @param wire I2C通信接口指针
   * @return 初始化结果
   */
  Error Init(const Config& config) {
    CHECK(regmap_.Init(config.wire, kI2CAddress));
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