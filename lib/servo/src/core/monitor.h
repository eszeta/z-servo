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

#include "../servo.h"

namespace hortor_servo {
enum class MonitorBitmap : uint8_t {
  kTarget = 0b1000000,    // 监控目标值
  kVoltage = 0b0100000,   // 监控电压值
  kCurrent = 0b0001000,   // 监控电流值
  kVelocity = 0b0000010,  // 监控速度值
  kPosition = 0b0000001   // 监控位置值
};

/**
 * @brief 监控类，用于监控电机状态
 */
class Monitor {
 public:
  /**
   * @brief 使用监控
   * @param serial 串口
   */
  void UseMonitoring(Print &serial);

  /**
   * @brief 处理监控
   * @param dt 时间间隔(秒)
   */
  void Process(float dt);

  /**·
   * @brief 链接电机
   * @param servo 伺服电机
   */
  void LinkMotor(Servo *servo);

 private:
  /**
   * @brief 监控变量
   */
  uint8_t variables_ = static_cast<uint8_t>(MonitorBitmap::kTarget) |
                       static_cast<uint8_t>(MonitorBitmap::kVoltage) |
                       static_cast<uint8_t>(MonitorBitmap::kVelocity) |
                       static_cast<uint8_t>(MonitorBitmap::kPosition);
  /**
   * @brief 伺服电机
   */
  Servo *servo_ = nullptr;
  /**
   * @brief 监控输出
   */
  Print *monitorPort_ = nullptr;
  /**
   * @brief 监控输出分隔符
   */
  char separator_ = '\t';  // 监控输出数据分隔符
  /**
   * @brief 监控输出小数位数
   */
  uint8_t decimals_ = 4;  // 监控输出数据的小数位数
  /**
   * @brief 监控周期（单位：秒）
   * @note 默认值为0.1秒（100ms）
   */
  float period_ = 0.1f;
};
}  // namespace hortor_servo
