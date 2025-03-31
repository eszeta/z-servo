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
#include "inst_accessor.h"

#ifdef ARDUINO_ARCH_STM32
#include <EEPROM.h>
#endif

#include "../core/register.h"
#include "./inst_types.h"

namespace hortor_servo {
using Regs = ServoRegs;

Error InstAccessor::Init() {
  CHECK_ERROR(local_transport_.Init(regs_, sizeof(regs_)));
  CHECK_ERROR(local_transport_.LinkAccessor(*this));
  CHECK_ERROR(LoadEEPROM());
  if (GetFirmwareMajor() == 0 && GetFirmwareMinor() == 0) {
    CHECK_ERROR(RecoveryEeprom());
    CHECK_ERROR(StoreEeprom());
  }
  CHECK_ERROR(ResetRam());
  return Error::kOk;
}

Error InstAccessor::RecoveryEeprom() {
  // 固件版本
  WriteRegField(Regs::kFirmwareMajor, RegsDefaultValues::kDefFirmwareMajor);
  WriteRegField(Regs::kFirmwareMinor, RegsDefaultValues::kDefFirmwareMinor);
  WriteRegField(Regs::kEnd, RegsDefaultValues::kDefEnd);
  WriteRegField(Regs::kServoMajor, RegsDefaultValues::kDefServoMajor);
  WriteRegField(Regs::kServoMinor, RegsDefaultValues::kDefServoMinor);

  // 基本参数
  WriteRegField(Regs::kId, RegsDefaultValues::kDefId);
  WriteRegField(Regs::kBaudrate, RegsDefaultValues::kDefBaudrate);
  WriteRegField(Regs::kResponseDelay, RegsDefaultValues::kDefResponseDelay);
  WriteRegField(Regs::kResponseLevel, RegsDefaultValues::kDefResponseLevel);

  // 角度限制
  WriteRegField(Regs::kMinPositionL, RegsDefaultValues::kDefMinPositionL);
  WriteRegField(Regs::kMinPositionH, RegsDefaultValues::kDefMinPositionH);
  WriteRegField(Regs::kMaxPositionL, RegsDefaultValues::kDefMaxPositionL);
  WriteRegField(Regs::kMaxPositionH, RegsDefaultValues::kDefMaxPositionH);

  // 温度和电压限制
  WriteRegField(Regs::kMaxTemperature, RegsDefaultValues::kDefMaxTemperature);
  WriteRegField(Regs::kMaxVoltage, RegsDefaultValues::kDefMaxVoltage);
  WriteRegField(Regs::kMinVoltage, RegsDefaultValues::kDefMinVoltage);

  // 扭矩相关
  WriteRegField(Regs::kMaxTorqueL, RegsDefaultValues::kDefMaxTorqueL);
  WriteRegField(Regs::kMaxTorqueH, RegsDefaultValues::kDefMaxTorqueH);

  // 选项
  WriteRegField(Regs::kOption, RegsDefaultValues::kDefOption);

  // 卸载条件
  WriteRegField(Regs::kUnloadCondition, RegsDefaultValues::kDefUnloadCondition);

  // LED报警条件
  WriteRegField(Regs::kLedAlarmCondition, RegsDefaultValues::kDefLedAlarmCondition);

  // 位置环PID参数
  WriteRegField(Regs::kPosPidKp, RegsDefaultValues::kDefPosPidKp);
  WriteRegField(Regs::kPosPidKd, RegsDefaultValues::kDefPosPidKd);
  WriteRegField(Regs::kPosPidKi, RegsDefaultValues::kDefPosPidKi);
  WriteRegField(Regs::kPosPidLimit, RegsDefaultValues::kDefPosPidLimit);

  // 最小启动力
  WriteRegField(Regs::kMinStartupForce, RegsDefaultValues::kDefMinStartupForce);

  // 不灵敏区设置
  WriteRegField(Regs::kCWInsensitiveArea, RegsDefaultValues::kDefCWInsensitiveArea);
  WriteRegField(Regs::kCCWInsensitiveArea, RegsDefaultValues::kDefCCWInsensitiveArea);

  // 电流保护
  WriteRegField(Regs::kCurrentProtectionThL, RegsDefaultValues::kDefCurrentProtectionThL);
  WriteRegField(Regs::kCurrentProtectionThH, RegsDefaultValues::kDefCurrentProtectionThH);
  WriteRegField(Regs::kOvercurrentProtectionTime, RegsDefaultValues::kDefOvercurrentProtectionTime);

  // 角度分辨率
  WriteRegField(Regs::kAngularResolution, RegsDefaultValues::kDefAngularResolution);

  // 位置校正
  WriteRegField(Regs::kPositionCorrectionL, RegsDefaultValues::kDefPositionCorrectionL);
  WriteRegField(Regs::kPositionCorrectionH, RegsDefaultValues::kDefPositionCorrectionH);

  // 运行模式
  WriteRegField(Regs::kMode, RegsDefaultValues::kDefMode);

  // 扭矩保护
  WriteRegField(Regs::kTorqueProtectionTh, RegsDefaultValues::kDefTorqueProtectionTh);
  WriteRegField(Regs::kTorqueProtectionTime, RegsDefaultValues::kDefTorqueProtectionTime);
  WriteRegField(Regs::kOverloadTorque, RegsDefaultValues::kDefOverloadTorque);

  // 速度环PID参数
  WriteRegField(Regs::kVelPidKp, RegsDefaultValues::kDefVelPidKp);
  WriteRegField(Regs::kVelPidKi, RegsDefaultValues::kDefVelPidKi);

  //-----------内部EEPROM（读写）-------------------
  //
  WriteRegField(Regs::kMotorDirection, RegsDefaultValues::kDefMotorDirection);
  WriteRegField(Regs::kSensorDirection, RegsDefaultValues::kDefSensorDirection);
  WriteRegField(Regs::kAdcShuntResistor, RegsDefaultValues::kDefAdcShuntResistor);
  WriteRegField(Regs::kAdcCurrentFactor, RegsDefaultValues::kDefAdcCurrentFactor);

  // 位置滤波器
  WriteRegField(Regs::kPosFilter, RegsDefaultValues::kDefPosFilter);
  WriteRegField(Regs::kCurrentFilter, RegsDefaultValues::kDefCurrentFilter);
  WriteRegField(Regs::kVelocityFilter, RegsDefaultValues::kDefVelocityFilter);

  // 位置PID
  WriteRegField(Regs::kPosPidKf, RegsDefaultValues::kDefPosPidFf);
  WriteRegField(Regs::kPosPidRamp, RegsDefaultValues::kDefPosPidRamp);
  return Error::kOk;
}

Error InstAccessor::LoadEEPROM() {
#ifdef ARDUINO_ARCH_STM32
  int pos = 0;
  for (uint8_t address = RegsBlocks::kEeprom.begin; address < RegsBlocks::kEeprom.end; address++) {
    this->regs_[address] = EEPROM.read(pos++);
  }
  for (uint8_t address = RegsBlocks::kInternalEeprom.begin; address < RegsBlocks::kInternalEeprom.end; address++) {
    this->regs_[address] = EEPROM.read(pos++);
  }
#endif
  return Error::kOk;
}

Error InstAccessor::StoreEeprom() {
#ifdef ARDUINO_ARCH_STM32
  int pos = 0;
  for (uint8_t address = RegsBlocks::kEeprom.begin; address < RegsBlocks::kEeprom.end; address++) {
    EEPROM.update(pos++, this->regs_[address]);
  }
  for (uint8_t address = RegsBlocks::kInternalEeprom.begin; address < RegsBlocks::kInternalEeprom.end; address++) {
    EEPROM.update(pos++, this->regs_[address]);
  }
#endif
  return Error::kOk;
}

Error InstAccessor::ResetRam() {
  // 扭矩开关
  WriteRegField(Regs::kTorqueEnable, 0);

  // 动作
  WriteRegField(Regs::kTargetAcceleration, 0);
  WriteRegField(Regs::kTargetPositionL, 0);
  WriteRegField(Regs::kTargetPositionH, 0);
  WriteRegField(Regs::kTargetTimeL, 0);
  WriteRegField(Regs::kTargetTimeH, 0);
  WriteRegField(Regs::kTargetVelocityL, 0);
  WriteRegField(Regs::kTargetVelocityH, 0);

  // 转矩限制
  WriteRegField(Regs::kTorqueLimitL, GetUint8(Regs::kMaxTorqueL));
  WriteRegField(Regs::kTorqueLimitH, GetUint8(Regs::kMaxTorqueH));

  // 写入锁
  WriteRegField(Regs::kWriteLock, 0);

  // 当前位置
  WriteRegField(Regs::kPresentPositionL, 0);
  WriteRegField(Regs::kPresentPositionH, 0);

  // 当前速度
  WriteRegField(Regs::kPresentVelocityL, 0);
  WriteRegField(Regs::kPresentVelocityH, 0);

  // 当前负载
  WriteRegField(Regs::kPresentLoadL, 0);
  WriteRegField(Regs::kPresentLoadH, 0);

  // 当前电压
  WriteRegField(Regs::kPresentVoltage, 0);

  // 当前温度
  WriteRegField(Regs::kPresentTemperature, 0);

  // 异步写标志
  WriteRegField(Regs::kAsynWriteSt, 0);

  // 舵机状态
  WriteRegField(Regs::kStatus, 0);

  // 移动标志
  WriteRegField(Regs::kMoving, 0);

  // 当前电流
  WriteRegField(Regs::kPresentCurrentL, 0);
  WriteRegField(Regs::kPresentCurrentH, 0);
  return Error::kOk;
}
}  // namespace hortor_servo
