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

#include "../utils/register/register_accessor.h"
#include "../utils/register/register_local_transport.h"
#include "./inst_types.h"

namespace hortor_servo {

class InstAccessor : public RegisterAccessor {
 public:
  using Regs = ServoRegs;

  Error Init();
  Error RecoveryEeprom();
  Error LoadEEPROM();
  Error StoreEeprom();
  Error ResetRam();

  /**
   * @brief 获取固件主版本号 (0x00)
   * @return 固件主版本号
   */
  uint8_t GetFirmwareMajor() {
    uint8_t major;
    ReadRegField(Regs::kFirmwareMajor, &major);
    return major;
  }

  /**
   * @brief 获取固件次版本号 (0x01)
   * @return 固件次版本号
   */
  uint8_t GetFirmwareMinor() {
    uint8_t minor;
    ReadRegField(Regs::kFirmwareMinor, &minor);
    return minor;
  }

  /**
   * @brief 获取舵机ID (0x05)
   * @return 舵机ID
   */
  uint8_t GetId() {
    uint8_t id;
    ReadRegField(Regs::kId, &id);
    return id;
  }

  /**
   * @brief 获取响应延迟时间 (0x07)
   * @return 响应延迟时间(2us)
   */
  uint16_t GetResponseDelay() {
    uint8_t delay;
    ReadRegField(Regs::kResponseDelay, &delay);
    return delay * 2;
  }

  /**
   * @brief 获取响应级别 (0x08)
   * @return 响应级别
   */
  bool GetResponseLevel() {
    uint8_t response_level;
    ReadRegField(Regs::kResponseLevel, &response_level);
    return response_level == 1;
  }

  /**
   * @brief 获取最小位置 (0x09-0x0A)
   * @return 最小位置(步)
   */
  uint16_t GetMinPosition() {
    uint16_t position;
    ReadRegField(Regs::kMinPositionH, Regs::kMinPositionL, &position);
    return position;
  }

  /**
   * @brief 获取最大位置 (0x0B-0x0C)
   * @return 最大位置(步)
   */
  uint16_t GetMaxPosition() {
    uint16_t position;
    ReadRegField(Regs::kMaxPositionH, Regs::kMaxPositionL, &position);
    return position;
  }

  /**
   * @brief 获取最大温度 (0x0D)
   * @return 最大温度(℃)
   */
  uint8_t GetMaxTemperature() {
    uint8_t temperature;
    ReadRegField(Regs::kMaxTemperature, &temperature);
    return temperature;
  }

  /**
   * @brief 获取最大电压 (0x0E)
   * @return 最大电压(0.1V)
   */
  float GetMaxVoltage() {
    uint8_t voltage;
    ReadRegField(Regs::kMaxVoltage, &voltage);
    return voltage * 0.1f;
  }

  /**
   * @brief 获取最小电压 (0x0F)
   * @return 最小电压(0.1V)
   */
  float GetMinVoltage() {
    uint8_t voltage;
    ReadRegField(Regs::kMinVoltage, &voltage);
    return voltage * 0.1f;
  }

  /**
   * @brief 获取最大扭矩 (0x10-0x11)
   * @return 最大扭矩(0.10%)
   */
  float GetMaxTorque() {
    uint16_t torque;
    ReadRegField(Regs::kMaxTorqueH, Regs::kMaxTorqueL, &torque);
    return torque * 0.001f;
  }

  /**
   * @brief 获取位置PID比例系数 (0x15)
   * @return 位置PID比例系数(0.1)
   */
  float GetPosPidKp() {
    uint8_t kp;
    ReadRegField(Regs::kPosPidKp, &kp);
    return kp * 0.1f;
  }

  /**
   * @brief 获取位置PID微分系数 (0x16)
   * @return 位置PID微分系数(0.1)
   */
  float GetPosPidKd() {
    uint8_t kd;
    ReadRegField(Regs::kPosPidKd, &kd);
    return kd * 0.1f;
  }

  /**
   * @brief 获取位置PID积分系数 (0x17)
   * @return 位置PID积分系数(0.1)
   */
  float GetPosPidKi() {
    uint8_t ki;
    ReadRegField(Regs::kPosPidKi, &ki);
    return ki * 0.1f;
  }

  /**
   * @brief 获取最小启动力 (0x18)
   * @return 最小启动力(0.1%)
   */
  float GetMinStartupForce() {
    uint8_t force;
    ReadRegField(Regs::kMinStartupForce, &force);
    return force * 0.001f;
  }

  /**
   * @brief 获取位置PID限制值 (0x19)
   * @return 位置PID限制值(4)
   */
  float GetPosPidLimit() {
    uint8_t limit;
    ReadRegField(Regs::kPosPidLimit, &limit);
    return limit * 4.0f;
  }

  /**
   * @brief 获取顺时针死区 (0x1A)
   * @return 顺时针死区(步)
   */
  uint8_t GetCWInsensitiveArea() {
    uint8_t area;
    ReadRegField(Regs::kCWInsensitiveArea, &area);
    return area;
  }

  /**
   * @brief 获取逆时针死区 (0x1B)
   * @return 逆时针死区(步)
   */
  uint8_t GetCCWInsensitiveArea() {
    uint8_t area;
    ReadRegField(Regs::kCCWInsensitiveArea, &area);
    return area;
  }

  /**
   * @brief 获取电流保护阈值 (0x1C-0x1D)
   * @return 电流保护阈值(6.5mA)
   */
  float GetCurrentProtectionThreshold() {
    uint16_t threshold;
    ReadRegField(Regs::kCurrentProtectionThH, Regs::kCurrentProtectionThL, &threshold);
    return threshold * 6.5f;
  }

  /**
   * @brief 获取角度分辨率 (0x1E)
   * @return 角度分辨率(度/步)
   */
  uint8_t GetAngularResolution() {
    uint8_t resolution;
    ReadRegField(Regs::kAngularResolution, &resolution);
    return resolution;
  }

  /**
   * @brief 获取位置校正值 (0x1F-0x20)
   * @return 位置校正值(步)
   */
  uint16_t GetPositionCorrection() {
    uint16_t correction;
    ReadRegField(Regs::kPositionCorrectionH, Regs::kPositionCorrectionL, &correction);
    return bit_utils::SignToTwos(correction, 11);
  }

  /**
   * @brief 获取电机模式 (0x21)
   * @return 电机模式
   */
  MotorMode GetMode() {
    uint8_t mode;
    ReadRegField(Regs::kMode, &mode);
    return static_cast<MotorMode>(mode);
  }

  /**
   * @brief 获取扭矩保护阈值 (0x22)
   * @return 扭矩保护阈值(1.0%)
   */
  float GetTorqueProtectionThreshold() {
    uint8_t threshold;
    ReadRegField(Regs::kTorqueProtectionTh, &threshold);
    return threshold * 0.01f;
  }

  /**
   * @brief 获取扭矩保护时间 (0x23)
   * @return 扭矩保护时间(10ms)
   */
  uint16_t GetTorqueProtectionTime() {
    uint8_t time;
    ReadRegField(Regs::kTorqueProtectionTime, &time);
    return time * 10;
  }

  /**
   * @brief 获取过载扭矩 (0x24)
   * @return 过载扭矩(1.0%)
   */
  float GetOverloadTorque() {
    uint8_t torque;
    ReadRegField(Regs::kOverloadTorque, &torque);
    return torque * 0.01f;
  }

  /**
   * @brief 获取速度PID比例系数 (0x25)
   * @return 速度PID比例系数(0.1)
   */
  float GetVelPidKp() {
    uint8_t kp;
    ReadRegField(Regs::kVelPidKp, &kp);
    return kp * 0.1f;
  }

  /**
   * @brief 获取过流保护时间 (0x26)
   * @return 过流保护时间(10ms)
   */
  uint16_t GetOvercurrentProtectionTime() {
    uint8_t time;
    ReadRegField(Regs::kOvercurrentProtectionTime, &time);
    return time * 10;
  }

  /**
   * @brief 获取速度PID积分系数 (0x27)
   * @return 速度PID积分系数(0.1)
   */
  float GetVelPidKi() {
    uint8_t ki;
    ReadRegField(Regs::kVelPidKi, &ki);
    return ki * 0.1f;
  }

  /**
   * @brief 获取使能状态 (0x28)
   * @return 使能状态
   */
  bool GetTorqueEnable() {
    uint8_t enable;
    ReadRegField(Regs::kTorqueEnable, &enable);
    return enable == 1;
  }

  /**
   * @brief 获取目标加速度 (0x29)
   * @return 目标加速度(100步/s²)
   */
  uint16_t GetTargetAcceleration() {
    uint8_t acceleration;
    ReadRegField(Regs::kTargetAcceleration, &acceleration);
    return acceleration * 100;
  }

  /**
   * @brief 获取目标位置 (0x2A-0x2B)
   * @return 目标位置(步)
   */
  int16_t GetTargetPosition() {
    uint16_t position;
    ReadRegField(Regs::kTargetPositionH, Regs::kTargetPositionL, &position);
    return bit_utils::SignToTwos(position, 15);
  }

  /**
   * @brief 获取目标PWM (0x2C-0x2D)
   * @return 目标PWM值(0.10%)
   */
  float GetTargetPwm() {
    uint16_t pwm;
    ReadRegField(Regs::kTargetTimeH, Regs::kTargetTimeL, &pwm);
    return pwm * 0.01f;
  }

  /**
   * @brief 获取目标时间 (0x2C-0x2D)
   * @return 目标时间(0.10%)
   */
  uint16_t GetTargetTime() {
    uint16_t time;
    ReadRegField(Regs::kTargetTimeH, Regs::kTargetTimeL, &time);
    return time;
  }

  /**
   * @brief 获取目标速度 (0x2E-0x2F)
   * @return 目标速度(步/s)
   */
  int16_t GetTargetVelocity() {
    uint16_t velocity;
    ReadRegField(Regs::kTargetVelocityH, Regs::kTargetVelocityL, &velocity);
    return bit_utils::SignToTwos(velocity, 15);
  }

  /**
   * @brief 获取扭矩限制 (0x30-0x31)
   * @return 扭矩限制(1.0%)
   */
  float GetTorqueLimit() {
    uint16_t limit;
    ReadRegField(Regs::kTorqueLimitH, Regs::kTorqueLimitL, &limit);
    return limit * 0.01f;
  }

  /**
   * @brief 获取EEPROM写保护状态 (0x37)
   * @return 是否锁定
   */
  bool GetWriteLock() {
    uint8_t lock;
    ReadRegField(Regs::kWriteLock, &lock);
    return lock == 1;
  }

  /**
   * @brief 获取当前位置 (0x38-0x39)
   * @return 当前位置(步)
   */
  int16_t GetPresentPosition() {
    uint16_t position;
    ReadRegField(Regs::kPresentPositionH, Regs::kPresentPositionL, &position);
    return bit_utils::SignToTwos(position, 15);
  }

  /**
   * @brief 设置当前位置 (0x38-0x39)
   * @param position 位置值(步)
   */
  void SetPresentPosition(const int16_t position) {
    WriteRegField(Regs::kPresentPositionH, Regs::kPresentPositionL, bit_utils::SignToTwos(position, 15));
  }

  /**
   * @brief 获取当前速度 (0x3A-0x3B)
   * @return 当前速度(步/s)
   */
  int16_t GetPresentVelocity() {
    uint16_t velocity;
    ReadRegField(Regs::kPresentVelocityH, Regs::kPresentVelocityL, &velocity);
    return bit_utils::SignToTwos(velocity, 15);
  }

  /**
   * @brief 设置当前速度 (0x3A-0x3B)
   * @param velocity 速度值(步/s)
   */
  void SetPresentVelocity(const int16_t velocity) {
    WriteRegField(Regs::kPresentVelocityH, Regs::kPresentVelocityL, bit_utils::SignToTwos(velocity, 15));
  }

  /**
   * @brief 设置当前负载 (0x3C-0x3D)
   * @param load 负载值(%)
   */
  void SetPresentLoad(const float load) {
    WriteRegField(Regs::kPresentLoadH, Regs::kPresentLoadL, bit_utils::SignToTwos(load * 10, 10));
  }

  /**
   * @brief 获取当前电压 (0x3E)
   * @return 当前电压(0.1V)
   */
  float GetPresentVoltage() {
    uint8_t voltage;
    ReadRegField(Regs::kPresentVoltage, &voltage);
    return voltage * 0.1f;
  }

  /**
   * @brief 设置当前电压 (0x3E)
   * @param voltage 电压值(0.1V)
   */
  void SetPresentVoltage(const float voltage) {
    WriteRegField(Regs::kPresentVoltage, static_cast<uint8_t>(voltage * 10));
  }

  /**
   * @brief 设置当前温度 (0x3F)
   * @param temperature 温度值(℃)
   */
  void SetPresentTemperature(const uint8_t temperature) { WriteRegField(Regs::kPresentTemperature, temperature); }

  /**
   * @brief 设置异步写入状态 (0x40)
   * @param enable 是否启用异步写入
   */
  void SetAsynWrite(const bool enable) { WriteRegField(Regs::kAsynWriteSt, enable ? 1 : 0); }

  /**
   * @brief 获取舵机状态 (0x41)
   * @return 舵机状态
   */
  uint8_t GetStatus() {
    uint8_t status;
    ReadRegField(Regs::kStatus, &status);
    return status;
  }

  /**
   * @brief 设置舵机状态 (0x41)
   * @param status 状态值
   */
  void SetStatus(const uint8_t status) { WriteRegField(Regs::kStatus, status); }

  /**
   * @brief 设置运动状态 (0x42)
   * @param moving 是否在运动
   */
  void SetMoving(const bool moving) { WriteRegField(Regs::kMoving, moving ? 1 : 0); }

  /**
   * @brief 设置当前电流 (0x45-0x46)
   * @param current 电流值(mA)
   */
  void SetPresentCurrent(const float current) {
    WriteRegField(Regs::kPresentCurrentH, Regs::kPresentCurrentL, static_cast<uint16_t>(current / 6.5f));
  }

  /**
   * @brief 获取电机方向 (0x60)
   * @return 电机方向
   */
  Direction GetMotorDirection() {
    uint8_t direction;
    ReadRegField(Regs::kMotorDirection, &direction);
    return direction == 1 ? Direction::CCW : Direction::CW;
  }

  /**
   * @brief 获取传感器方向 (0x61)
   * @return 传感器方向
   */
  Direction GetSensorDirection() {
    uint8_t direction;
    ReadRegField(Regs::kSensorDirection, &direction);
    return direction == 1 ? Direction::CCW : Direction::CW;
  }

  /**
   * @brief 获取位置PID前馈系数 (0x64)
   * @return 位置PID前馈系数(0.1)
   */
  float GetPosPidFf() {
    uint8_t ff;
    ReadRegField(Regs::kPosPidKf, &ff);
    return ff * 0.1f;
  }

  /**
   * @brief 获取位置PID斜坡系数 (0x68)
   * @return 位置PID斜坡系数(0.1)
   */
  float GetPosPidRamp() {
    uint8_t ramp;
    ReadRegField(Regs::kPosPidRamp, &ramp);
    return ramp * 0.1f;
  }

  /**
   * @brief 获取位置滤波系数 (0x6C)
   * @return 位置滤波系数(0.001)
   */
  float GetPosFilter() {
    uint8_t filter;
    ReadRegField(Regs::kPosFilter, &filter);
    return filter * 0.001f;
  }

  /**
   * @brief 获取电流滤波系数 (0x70)
   * @return 电流滤波系数(0.001)
   */
  float GetCurrentFilter() {
    uint8_t filter;
    ReadRegField(Regs::kCurrentFilter, &filter);
    return filter * 0.001f;
  }

  /**
   * @brief 获取速度滤波系数 (0x74)
   * @return 速度滤波系数(0.001)
   */
  float GetVelFilter() {
    uint8_t filter;
    ReadRegField(Regs::kVelocityFilter, &filter);
    return filter * 0.001f;
  }

 private:
  RegisterLocalTransport local_transport_;
  uint8_t regs_[RegsBlocks::kTotal.size()] = {};
};
}  // namespace hortor_servo
