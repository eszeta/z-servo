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

#include "register/register_accessor.h"
#include "servo/types.h"
#include "types.h"

namespace hortor::drivers::LSM6DSOW {

/**
 * @brief LSM6DSOW控制器类，负责与LSM6DSOW传感器的通信和数据读取
 */
class LSM6DSOW_accessor : public reg::RegisterAccessor {
 public:
  /**
   * @brief 初始化控制器
   * @return 初始化结果
   */
  Error Init();

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
