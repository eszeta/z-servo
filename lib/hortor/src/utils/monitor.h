// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "hortor.h"

namespace hortor::utils {

/**
 * @brief 监控类，用于监控电机状态
 */
template <typename ServoType>
class Monitor {
 public:
  enum MonitorBitmap : uint8_t {
    kTarget = 0b1000000,    // 监控目标值
    kPwm = 0b0100000,       // 监控PWM
    kCurrent = 0b0001000,   // 监控电流值
    kVelocity = 0b0000010,  // 监控速度值
    kPosition = 0b0000001   // 监控位置值
  };

  /**
   * @brief 监控输出小数位数
   */
  constexpr static uint8_t kDecimals = 4;  // 监控输出数据的小数位数
  /**
   * @brief 使用监控
   * @param serial 串口
   */
  void LinkPort(Print* serial) { monitorPort_ = serial; }

  /**·
   * @brief 链接电机
   * @param servo 伺服电机
   */
  void LinkMotor(ServoType* servo) { servo_ = servo; }

  /**
   * @brief 处理监控
   * @param dt 时间间隔(秒)
   */
  Error Process(float dt) {
    if (!monitorPort_) return Error::kOk;

    if (variables_ & MonitorBitmap::kTarget) {
      monitorPort_->print(F(">target:"));
      monitorPort_->println(servo_->GetGoalPosition());
    }

    if (variables_ & MonitorBitmap::kPwm) {
      monitorPort_->print(F(">pwm:"));
      monitorPort_->println(servo_->GetPresentPwm(), kDecimals);
    }

    if (variables_ & MonitorBitmap::kCurrent) {
      monitorPort_->print(F(">current:"));
      monitorPort_->println(servo_->GetPresentCurrent(), kDecimals);
    }

    if (variables_ & MonitorBitmap::kVelocity) {
      monitorPort_->print(F(">velocity:"));
      monitorPort_->println(servo_->GetPresentVelocity(), kDecimals);
    }

    if (variables_ & MonitorBitmap::kPosition) {
      monitorPort_->print(F(">position:"));
      monitorPort_->println(servo_->GetPresentPosition());
    }

    return Error::kOk;
  }

 private:
  /**
   * @brief 监控变量
   */
  uint8_t variables_ = kTarget | kPwm | kCurrent | kVelocity | kPosition;
  /**
   * @brief 伺服电机
   */
  ServoType* servo_ = nullptr;
  /**
   * @brief 监控输出
   */
  Print* monitorPort_ = nullptr;
};
}  // namespace hortor::utils
