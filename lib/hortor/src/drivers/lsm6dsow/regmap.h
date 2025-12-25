// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "regmap/regmap_i2c_bus.h"
#include "servo/types.h"
#include "types.h"

namespace hortor::drivers::LSM6DSOW {

/**
 * @brief LSM6DSOW控制器类（CRTP模式），负责与LSM6DSOW传感器的通信和数据读取
 *
 * 继承自 regmap::RegMapI2CBus，通过 CRTP 实现编译期静态多态。
 */
class RegMap : public regmap::RegMapI2CBus {
 public:
  /**
   * @brief 初始化控制器
   * @param wire Arduino Wire对象指针
   * @param address 从机地址
   * @return 初始化结果
   */
  Error Init(TwoWire* wire, const int address);

  /**
   * @brief 读取加速度数据
   * @param x 加速度X轴数据
   * @param y 加速度Y轴数据
   * @param z 加速度Z轴数据
   * @return 读取结果
   */
  Error ReadAcceleration(float& x, float& y, float& z);

  /**
   * @brief 检查加速度数据是否可用
   * @return 是否可用
   */
  bool AccelerationAvailable();

  /**
   * @brief 读取陀螺仪数据
   * @param x 陀螺仪X轴数据,单位：度/秒
   * @param y 陀螺仪Y轴数据,单位：度/秒
   * @param z 陀螺仪Z轴数据,单位：度/秒
   * @return 读取结果
   */
  Error ReadGyroscope(float& x, float& y, float& z);

  /**
   * @brief 检查陀螺仪数据是否可用
   * @return 是否可用
   */
  bool GyroscopeAvailable();

  /**
   * @brief 读取温度数据
   * @param temperature_deg 温度数据
   * @return 读取结果
   */
  Error ReadTemperature(float& temperature_deg);

  /**
   * @brief 检查温度数据是否可用
   * @return 是否可用
   */
  bool TemperatureAvailable();
};

}  // namespace hortor::drivers::LSM6DSOW
