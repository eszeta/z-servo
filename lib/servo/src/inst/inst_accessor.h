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

#include "inst/inst_types.h"
#include "register/register_accessor.h"
#include "register/register_raw_adapter.h"

namespace hortor_servo {

class InstAccessor : public RegisterAccessor {
 public:
  using Regs = ServoRegs;

  Error Init();
  Error RecoveryEeprom();
  Error LoadEeprom();
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
   * @brief 设置固件主版本号 (0x00)
   * @param major 固件主版本号
   */
  void SetFirmwareMajor(const uint8_t major) {
    WriteRegField(Regs::kFirmwareMajor, major);
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
   * @brief 设置固件次版本号 (0x01)
   * @param minor 固件次版本号
   */
  void SetFirmwareMinor(const uint8_t minor) {
    WriteRegField(Regs::kFirmwareMinor, minor);
  }

  /**
   * @brief 获取结束标志 (0x02)
   * @return 结束标志
   */
  uint8_t GetEnd() {
    uint8_t end;
    ReadRegField(Regs::kEnd, &end);
    return end;
  }

  /**
   * @brief 设置结束标志 (0x02)
   * @param end 结束标志
   */
  void SetEnd(const uint8_t end) { WriteRegField(Regs::kEnd, end); }

  /**
   * @brief 获取舵机主版本号 (0x03)
   * @return 舵机主版本号
   */
  uint8_t GetServoMajor() {
    uint8_t major;
    ReadRegField(Regs::kServoMajor, &major);
    return major;
  }

  /**
   * @brief 设置舵机主版本号 (0x03)
   * @param major 舵机主版本号
   */
  void SetServoMajor(const uint8_t major) {
    WriteRegField(Regs::kServoMajor, major);
  }

  /**
   * @brief 获取舵机次版本号 (0x03)
   * @return 舵机次版本号
   */
  uint8_t GetServoMinor() {
    uint8_t minor;
    ReadRegField(Regs::kServoMinor, &minor);
    return minor;
  }

  /**
   * @brief 设置舵机次版本号 (0x03)
   * @param minor 舵机次版本号
   */
  void SetServoMinor(const uint8_t minor) {
    WriteRegField(Regs::kServoMinor, minor);
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
   * @brief 设置舵机ID (0x05)
   * @param id 舵机ID
   */
  void SetId(const uint8_t id) { WriteRegField(Regs::kId, id); }

  /**
   * @brief 获取波特率 (0x06)
   * @return 波特率
   */
  uint8_t GetBaudrate() {
    uint8_t baudrate;
    ReadRegField(Regs::kBaudrate, &baudrate);
    return kBaudrateTable[baudrate];
  }

  /**
   * @brief 设置波特率 (0x06)
   * @param baudrate 波特率
   */
  void SetBaudrate(const uint16_t baudrate) {
    uint8_t baudrate_index = 0;
    for (uint8_t i = 0; i < sizeof(kBaudrateTable) / sizeof(kBaudrateTable[0]);
         i++) {
      if (baudrate == kBaudrateTable[i]) {
        baudrate_index = i;
        break;
      }
    }
    WriteRegField(Regs::kBaudrate, baudrate_index);
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
   * @brief 设置响应延迟时间 (0x07)
   * @param delay 响应延迟时间(2us)
   */
  void SetResponseDelay(const uint16_t delay) {
    WriteRegField(Regs::kResponseDelay, static_cast<uint8_t>(delay / 2));
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
   * @brief 设置响应级别 (0x08)
   * @param response_level 响应级别
   */
  void SetResponseLevel(const bool response_level) {
    WriteRegField(Regs::kResponseLevel, response_level ? 1 : 0);
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
   * @brief 设置最小位置 (0x09-0x0A)
   * @param position 最小位置(步)
   */
  void SetMinPosition(const uint16_t position) {
    WriteRegField(Regs::kMinPositionH, Regs::kMinPositionL, position);
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
   * @brief 设置最大位置 (0x0B-0x0C)
   * @param position 最大位置(步)
   */
  void SetMaxPosition(const uint16_t position) {
    WriteRegField(Regs::kMaxPositionH, Regs::kMaxPositionL, position);
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
   * @brief 设置最大温度 (0x0D)
   * @param temperature 最大温度(℃)
   */
  void SetMaxTemperature(const uint8_t temperature) {
    WriteRegField(Regs::kMaxTemperature, temperature);
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
   * @brief 设置最大电压 (0x0E)
   * @param voltage 最大电压(0.1V)
   */
  void SetMaxVoltage(const float voltage) {
    WriteRegField(Regs::kMaxVoltage, static_cast<uint8_t>(voltage * 10));
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
   * @brief 设置最小电压 (0x0F)
   * @param voltage 最小电压(0.1V)
   */
  void SetMinVoltage(const float voltage) {
    WriteRegField(Regs::kMinVoltage, static_cast<uint8_t>(voltage * 10));
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
   * @brief 设置最大扭矩 (0x10-0x11)
   * @param torque 最大扭矩(0.10%)
   */
  void SetMaxTorque(const float torque) {
    WriteRegField(Regs::kMaxTorqueH,
                  Regs::kMaxTorqueL,
                  static_cast<uint16_t>(torque * 1000));
  }

  /**
   * @brief 获取选项 (0x12)
   * @return 选项
   */
  uint8_t GetOption() {
    uint8_t option;
    ReadRegField(Regs::kOption, &option);
    return option;
  }

  /**
   * @brief 设置选项 (0x12)
   * @param option 选项
   */
  void SetOption(const uint8_t option) { WriteRegField(Regs::kOption, option); }

  /**
   * @brief 获取卸载条件 (0x13)
   * @return 卸载条件
   */
  uint8_t GetUnloadCondition() {
    uint8_t condition;
    ReadRegField(Regs::kUnloadCondition, &condition);
    return condition;
  }

  /**
   * @brief 设置卸载条件 (0x13)
   * @param condition 卸载条件
   */
  void SetUnloadCondition(const uint8_t condition) {
    WriteRegField(Regs::kUnloadCondition, condition);
  }

  /**
   * @brief 获取LED报警条件 (0x14)
   * @return LED报警条件
   */
  uint8_t GetLedAlarmCondition() {
    uint8_t condition;
    ReadRegField(Regs::kLedAlarmCondition, &condition);
    return condition;
  }

  /**
   * @brief 设置LED报警条件 (0x14)
   * @param condition LED报警条件
   */
  void SetLedAlarmCondition(const uint8_t condition) {
    WriteRegField(Regs::kLedAlarmCondition, condition);
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
   * @brief 设置位置PID比例系数 (0x15)
   * @param kp 位置PID比例系数(0.1)
   */
  void SetPosPidKp(const float kp) {
    WriteRegField(Regs::kPosPidKp, static_cast<uint8_t>(kp * 10));
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
   * @brief 设置位置PID微分系数 (0x16)
   * @param kd 位置PID微分系数(0.1)
   */
  void SetPosPidKd(const float kd) {
    WriteRegField(Regs::kPosPidKd, static_cast<uint8_t>(kd * 10));
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
   * @brief 设置位置PID积分系数 (0x17)
   * @param ki 位置PID积分系数(0.1)
   */
  void SetPosPidKi(const float ki) {
    WriteRegField(Regs::kPosPidKi, static_cast<uint8_t>(ki * 10));
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
   * @brief 设置最小启动力 (0x18)
   * @param force 最小启动力(0.1%)
   */
  void SetMinStartupForce(const float force) {
    WriteRegField(Regs::kMinStartupForce, static_cast<uint8_t>(force * 1000));
  }

  /**
   * @brief 获取积分限制值 (0x19)
   * @return 积分限制值(4)
   */
  float GetPosPidILimit() {
    uint8_t limit;
    ReadRegField(Regs::kPosPidILimit, &limit);
    return limit * 4.0f;
  }

  /**
   * @brief 设置积分限制值 (0x19)
   * @param limit 积分限制值(4)
   */
  void SetPosPidILimit(const float limit) {
    WriteRegField(Regs::kPosPidILimit, static_cast<uint8_t>(limit / 4));
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
   * @brief 设置顺时针死区 (0x1A)
   * @param area 顺时针死区(步)
   */
  void SetCWInsensitiveArea(const uint8_t area) {
    WriteRegField(Regs::kCWInsensitiveArea, area);
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
   * @brief 设置逆时针死区 (0x1B)
   * @param area 逆时针死区(步)
   */
  void SetCCWInsensitiveArea(const uint8_t area) {
    WriteRegField(Regs::kCCWInsensitiveArea, area);
  }

  /**
   * @brief 获取电流保护阈值 (0x1C-0x1D)
   * @return 电流保护阈值(6.5mA)
   */
  float GetCurrentProtectionThreshold() {
    uint16_t threshold;
    ReadRegField(
        Regs::kCurrentProtectionThH, Regs::kCurrentProtectionThL, &threshold);
    return threshold * 6.5f;
  }

  /**
   * @brief 设置电流保护阈值 (0x1C-0x1D)
   * @param threshold 电流保护阈值(6.5mA)
   */
  void SetCurrentProtectionThreshold(const float threshold) {
    WriteRegField(Regs::kCurrentProtectionThH,
                  Regs::kCurrentProtectionThL,
                  static_cast<uint16_t>(threshold / 6.5f));
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
   * @brief 设置角度分辨率 (0x1E)
   * @param resolution 角度分辨率(度/步)
   */
  void SetAngularResolution(const uint8_t resolution) {
    WriteRegField(Regs::kAngularResolution, resolution);
  }

  /**
   * @brief 获取位置校正值 (0x1F-0x20)
   * @return 位置校正值(步)
   */
  int16_t GetPositionCorrection() {
    uint16_t correction;
    ReadRegField(
        Regs::kPositionCorrectionH, Regs::kPositionCorrectionL, &correction);
    return bit_utils::SignToTwos(correction, 11);
  }

  /**
   * @brief 设置位置校正值 (0x1F-0x20)
   * @param correction 位置校正值(步)
   */
  void SetPositionCorrection(const int16_t correction) {
    WriteRegField(Regs::kPositionCorrectionH,
                  Regs::kPositionCorrectionL,
                  bit_utils::SignToTwos(correction, 11));
  }

  /**
   * @brief 获取电机模式 (0x21)
   * @return 电机模式
   */
  ServoMode GetMode() {
    uint8_t mode;
    ReadRegField(Regs::kMode, &mode);
    return static_cast<ServoMode>(mode);
  }

  /**
   * @brief 设置电机模式 (0x21)
   * @param mode 电机模式
   */
  void SetMode(const ServoMode mode) {
    WriteRegField(Regs::kMode, static_cast<uint8_t>(mode));
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
   * @brief 设置扭矩保护阈值 (0x22)
   * @param threshold 扭矩保护阈值(1.0%)
   */
  void SetTorqueProtectionThreshold(const float threshold) {
    WriteRegField(Regs::kTorqueProtectionTh,
                  static_cast<uint8_t>(threshold * 100));
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
   * @brief 设置扭矩保护时间 (0x23)
   * @param time 扭矩保护时间(10ms)
   */
  void SetTorqueProtectionTime(const uint16_t time) {
    WriteRegField(Regs::kTorqueProtectionTime, static_cast<uint8_t>(time / 10));
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
   * @brief 设置过载扭矩 (0x24)
   * @param torque 过载扭矩(1.0%)
   */
  void SetOverloadTorque(const float torque) {
    WriteRegField(Regs::kOverloadTorque, static_cast<uint8_t>(torque * 100));
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
   * @brief 设置速度PID比例系数 (0x25)
   * @param kp 速度PID比例系数(0.1)
   */
  void SetVelPidKp(const float kp) {
    WriteRegField(Regs::kVelPidKp, static_cast<uint8_t>(kp * 10));
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
   * @brief 设置过流保护时间 (0x26)
   * @param time 过流保护时间(10ms)
   */
  void SetOvercurrentProtectionTime(const uint16_t time) {
    WriteRegField(Regs::kOvercurrentProtectionTime,
                  static_cast<uint8_t>(time / 10));
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
   * @brief 设置速度PID积分系数 (0x27)
   * @param ki 速度PID积分系数(0.1)
   */
  void SetVelPidKi(const float ki) {
    WriteRegField(Regs::kVelPidKi, static_cast<uint8_t>(ki * 10));
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
   * @brief 设置使能状态 (0x28)
   * @param enable 使能状态
   */
  void SetTorqueEnable(const bool enable) {
    WriteRegField(Regs::kTorqueEnable, enable ? 1 : 0);
  }

  /**
   * @brief 获取目标加速度 (0x29)
   * @return 目标加速度(100步/s²)
   */
  float GetGoalAcceleration() {
    uint8_t acceleration;
    ReadRegField(Regs::kGoalAcceleration, &acceleration);
    return static_cast<float>(acceleration) * 100.0f;
  }

  /**
   * @brief 设置目标加速度 (0x29)
   * @param acceleration 目标加速度(100步/s²)
   */
  void SetGoalAcceleration(const float acceleration) {
    WriteRegField(Regs::kGoalAcceleration,
                  static_cast<uint8_t>(acceleration / 100.0f));
  }

  /**
   * @brief 获取目标位置 (0x2A-0x2B)
   * @return 目标位置(步)
   */
  float GetGoalPosition() {
    uint16_t position;
    ReadRegField(Regs::kGoalPositionH, Regs::kGoalPositionL, &position);
    return static_cast<float>(bit_utils::SignToTwos(position, 15));
  }

  /**
   * @brief 设置目标位置 (0x2A-0x2B)
   * @param position 目标位置(步)
   */
  void SetGoalPosition(const float position) {
    WriteRegField(Regs::kGoalPositionH,
                  Regs::kGoalPositionL,
                  static_cast<uint16_t>(bit_utils::SignToTwos(position, 15)));
  }

  /**
   * @brief 获取目标PWM (0x2C-0x2D)
   * @return 目标PWM值(0.10%)
   */
  float GetGoalPwm() {
    uint16_t pwm;
    ReadRegField(Regs::kGoalTimeH, Regs::kGoalTimeL, &pwm);
    return pwm * 0.01f;
  }

  /**
   * @brief 设置目标PWM (0x2C-0x2D)
   * @param pwm 目标PWM值(0.10%)
   */
  void SetGoalPwm(const float pwm) {
    WriteRegField(
        Regs::kGoalTimeH, Regs::kGoalTimeL, static_cast<uint16_t>(pwm * 100));
  }

  /**
   * @brief 获取目标时间 (0x2C-0x2D)
   * @return 目标时间(0.10%)
   */
  uint16_t GetGoalTime() {
    uint16_t time;
    ReadRegField(Regs::kGoalTimeH, Regs::kGoalTimeL, &time);
    return time;
  }

  /**
   * @brief 设置目标时间 (0x2C-0x2D)
   * @param time 目标时间(0.10%)
   */
  void SetGoalTime(const uint16_t time) {
    WriteRegField(Regs::kGoalTimeH, Regs::kGoalTimeL, time);
  }

  /**
   * @brief 获取目标速度 (0x2E-0x2F)
   * @return 目标速度(步/s)
   */
  float GetGoalVelocity() {
    uint16_t velocity;
    ReadRegField(Regs::kGoalVelocityH, Regs::kGoalVelocityL, &velocity);
    return static_cast<float>(bit_utils::SignToTwos(velocity, 15));
  }

  /**
   * @brief 设置目标速度 (0x2E-0x2F)
   * @param velocity 目标速度(步/s)
   */
  void SetGoalVelocity(const float velocity) {
    WriteRegField(Regs::kGoalVelocityH,
                  Regs::kGoalVelocityL,
                  static_cast<uint16_t>(bit_utils::SignToTwos(velocity, 15)));
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
   * @brief 设置扭矩限制 (0x30-0x31)
   * @param limit 扭矩限制(1.0%)
   */
  void SetTorqueLimit(const float limit) {
    WriteRegField(Regs::kTorqueLimitH,
                  Regs::kTorqueLimitL,
                  static_cast<uint16_t>(limit * 100));
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
   * @brief 设置EEPROM写保护状态 (0x37)
   * @param lock 是否锁定
   */
  void SetWriteLock(const bool lock) {
    WriteRegField(Regs::kWriteLock, lock ? 1 : 0);
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
    WriteRegField(Regs::kPresentPositionH,
                  Regs::kPresentPositionL,
                  bit_utils::SignToTwos(position, 15));
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
    WriteRegField(Regs::kPresentVelocityH,
                  Regs::kPresentVelocityL,
                  bit_utils::SignToTwos(velocity, 15));
  }

  /**
   * @brief 设置当前负载 (0x3C-0x3D)
   * @param load 负载值(%)
   */
  void SetPresentLoad(const float load) {
    WriteRegField(Regs::kPresentLoadH,
                  Regs::kPresentLoadL,
                  bit_utils::SignToTwos(load * 10, 10));
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
  void SetPresentTemperature(const uint8_t temperature) {
    WriteRegField(Regs::kPresentTemperature, temperature);
  }

  /**
   * @brief 设置异步写入状态 (0x40)
   * @param enable 是否启用异步写入
   */
  void SetAsyncWrite(const bool enable) {
    WriteRegField(Regs::kAsyncWriteSt, enable ? 1 : 0);
  }

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
  void SetMoving(const bool moving) {
    WriteRegField(Regs::kMoving, moving ? 1 : 0);
  }

  /**
   * @brief 设置当前电流 (0x45-0x46)
   * @param current 电流值(mA)
   */
  void SetPresentCurrent(const float current) {
    WriteRegField(Regs::kPresentCurrentH,
                  Regs::kPresentCurrentL,
                  static_cast<uint16_t>(current / 6.5f));
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
   * @brief 设置传感器方向 (0x61)
   * @param direction 传感器方向
   */
  void SetSensorDirection(const Direction direction) {
    WriteRegField(Regs::kSensorDirection, direction == Direction::CCW ? 1 : 0);
  }

  /**
   * @brief 获取电机旋转方向 (0x61)
   * @return 电机旋转方向
   */
  Direction GetMotorDirection() {
    uint8_t direction;
    ReadRegField(Regs::kMotorDirection, &direction);
    return direction == 1 ? Direction::CCW : Direction::CW;
  }

  /**
   * @brief 设置电机旋转方向 (0x61)
   * @param direction 电机旋转方向
   */
  void SetMotorDirection(const Direction direction) {
    WriteRegField(Regs::kMotorDirection, direction == Direction::CCW ? 1 : 0);
  }

  /**
   * @brief 获取ADC分流电阻 (0x62)
   * @return ADC分流电阻(100Ω)
   */
  uint16_t GetAdcShuntResistor() {
    uint8_t resistor;
    ReadRegField(Regs::kAdcShuntResistor, &resistor);
    return resistor * 100.0f;
  }

  /**
   * @brief 设置ADC分流电阻 (0x62)
   * @param resistor ADC分流电阻(100Ω)
   */
  void SetAdcShuntResistor(const uint16_t resistor) {
    WriteRegField(Regs::kAdcShuntResistor,
                  static_cast<uint8_t>(resistor / 100));
  }

  /**
   * @brief 获取ADC电流系数 (0x63)
   * @return ADC电流系数(100)
   */
  uint16_t GetAdcCurrentFactor() {
    uint8_t factor;
    ReadRegField(Regs::kAdcCurrentFactor, &factor);
    return factor * 100;
  }

  /**
   * @brief 设置ADC电流系数 (0x63)
   * @param factor ADC电流系数(100)
   */
  void SetAdcCurrentFactor(const uint16_t factor) {
    WriteRegField(Regs::kAdcCurrentFactor, static_cast<uint8_t>(factor / 100));
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
   * @brief 设置位置PID前馈系数 (0x64)
   * @param ff 位置PID前馈系数(0.1)
   */
  void SetPosPidFf(const float ff) {
    WriteRegField(Regs::kPosPidKf, static_cast<uint8_t>(ff * 10));
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
   * @brief 设置位置滤波系数 (0x6C)
   * @param filter 位置滤波系数(0.001)
   */
  void SetPosFilter(const float filter) {
    WriteRegField(Regs::kPosFilter, static_cast<uint8_t>(filter * 1000));
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
   * @brief 设置电流滤波系数 (0x70)
   * @param filter 电流滤波系数(0.001)
   */
  void SetCurrentFilter(const float filter) {
    WriteRegField(Regs::kCurrentFilter, static_cast<uint8_t>(filter * 1000));
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

  /**
   * @brief 设置速度滤波系数 (0x74)
   * @param filter 速度滤波系数(0.001)
   */
  void SetVelFilter(const float filter) {
    WriteRegField(Regs::kVelocityFilter, static_cast<uint8_t>(filter * 1000));
  }

 private:
  RegisterRawAdapter raw_adapter_;
  uint8_t regs_[RegsBlocks::kTotal.size()] = {};
};
}  // namespace hortor_servo
