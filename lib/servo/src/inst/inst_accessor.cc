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

#include "../core/register_field.h"
#include "./inst_types.h"

namespace hortor_servo {
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
  WriteRegField(ServoRegisters::kFirmwareMajor, RegsDefaultValues::kDefFirmwareMajor);
  WriteRegField(ServoRegisters::kFirmwareMinor, RegsDefaultValues::kDefFirmwareMinor);
  WriteRegField(ServoRegisters::kEnd, RegsDefaultValues::kDefEnd);
  WriteRegField(ServoRegisters::kServoMajor, RegsDefaultValues::kDefServoMajor);
  WriteRegField(ServoRegisters::kServoMinor, RegsDefaultValues::kDefServoMinor);

  // 基本参数
  WriteRegField(ServoRegisters::kId, RegsDefaultValues::kDefId);
  WriteRegField(ServoRegisters::kBaudrate, RegsDefaultValues::kDefBaudrate);
  WriteRegField(ServoRegisters::kResponseDelay, RegsDefaultValues::kDefResponseDelay);
  WriteRegField(ServoRegisters::kResponseLevel, RegsDefaultValues::kDefResponseStatusLevel);

  // 角度限制
  WriteRegField(ServoRegisters::kMinPositionL, RegsDefaultValues::kDefMinPositionL);
  WriteRegField(ServoRegisters::kMinPositionH, RegsDefaultValues::kDefMinPositionH);
  WriteRegField(ServoRegisters::kMaxPositionL, RegsDefaultValues::kDefMaxPositionL);
  WriteRegField(ServoRegisters::kMaxPositionH, RegsDefaultValues::kDefMaxPositionH);

  // 温度和电压限制
  WriteRegField(ServoRegisters::kMaxTemperature, RegsDefaultValues::kDefMaxTemperature);
  WriteRegField(ServoRegisters::kMaxVoltage, RegsDefaultValues::kDefMaxVoltage);
  WriteRegField(ServoRegisters::kMinVoltage, RegsDefaultValues::kDefMinVoltage);

  // 扭矩相关
  WriteRegField(ServoRegisters::kMaxTorqueL, RegsDefaultValues::kDefMaxTorqueL);
  WriteRegField(ServoRegisters::kMaxTorqueH, RegsDefaultValues::kDefMaxTorqueH);

  // 选项
  WriteRegField(ServoRegisters::kOption, RegsDefaultValues::kDefOption);

  // 卸载条件
  WriteRegField(ServoRegisters::kUnloadCondition, RegsDefaultValues::kDefUnloadCondition);

  // LED报警条件
  WriteRegField(ServoRegisters::kLedAlarmCondition, RegsDefaultValues::kDefLedAlarmCondition);

  // 位置环PID参数
  WriteRegField(ServoRegisters::kPosProportionalGain, RegsDefaultValues::kDefPosProportionalGain);
  WriteRegField(ServoRegisters::kPosDerivativeGain, RegsDefaultValues::kDefPosDerivativeGain);
  WriteRegField(ServoRegisters::kPosIntegralGain, RegsDefaultValues::kDefPosIntegralGain);
  WriteRegField(ServoRegisters::kIntegralLimit, RegsDefaultValues::kDefIntegralLimit);

  // 最小启动力
  WriteRegField(ServoRegisters::kMinStartupForce, RegsDefaultValues::kDefMinStartupForce);

  // 不灵敏区设置
  WriteRegField(ServoRegisters::kCWInsensitiveArea, RegsDefaultValues::kDefCWInsensitiveArea);
  WriteRegField(ServoRegisters::kCCWInsensitiveArea, RegsDefaultValues::kDefCCWInsensitiveArea);

  // 电流保护
  WriteRegField(ServoRegisters::kCurrentProtectionThL, RegsDefaultValues::kDefCurrentProtectionThL);
  WriteRegField(ServoRegisters::kCurrentProtectionThH, RegsDefaultValues::kDefCurrentProtectionThH);
  WriteRegField(ServoRegisters::kOvercurrentProtectionTime, RegsDefaultValues::kDefOvercurrentProtectionTime);

  // 角度分辨率
  WriteRegField(ServoRegisters::kAngularResolution, RegsDefaultValues::kDefAngularResolution);

  // 位置校正
  WriteRegField(ServoRegisters::kPositionCorrectionL, RegsDefaultValues::kDefPositionCorrectionL);
  WriteRegField(ServoRegisters::kPositionCorrectionH, RegsDefaultValues::kDefPositionCorrectionH);

  // 运行模式
  WriteRegField(ServoRegisters::kMode, RegsDefaultValues::kDefMode);

  // 扭矩保护
  WriteRegField(ServoRegisters::kTorqueProtectionTh, RegsDefaultValues::kDefTorqueProtectionTh);
  WriteRegField(ServoRegisters::kTorqueProtectionTime, RegsDefaultValues::kDefTorqueProtectionTime);
  WriteRegField(ServoRegisters::kOverloadTorque, RegsDefaultValues::kDefOverloadTorque);

  // 速度环PID参数
  WriteRegField(ServoRegisters::kVelocityProportionalGain, RegsDefaultValues::kDefVelocityProportionalGain);
  WriteRegField(ServoRegisters::kVelocityIntegralGain, RegsDefaultValues::kDefVelocityIntegralGain);

  //-----------内部EEPROM（读写）-------------------
  //
  WriteRegField(ServoRegisters::kMotorDirection, RegsDefaultValues::kDefMotorDirection);
  WriteRegField(ServoRegisters::kSensorDirection, RegsDefaultValues::kDefSensorDirection);
  WriteRegField(ServoRegisters::kAdcShuntResistor, RegsDefaultValues::kDefAdcShuntResistor);
  WriteRegField(ServoRegisters::kAdcCurrentFactor, RegsDefaultValues::kDefAdcCurrentFactor);

  // 位置滤波器
  WriteRegField(ServoRegisters::kPosFilter, RegsDefaultValues::kDefPosFilter);
  WriteRegField(ServoRegisters::kCurrentFilter, RegsDefaultValues::kDefCurrentFilter);
  WriteRegField(ServoRegisters::kVelocityFilter, RegsDefaultValues::kDefVelocityFilter);

  // 位置PID
  WriteRegField(ServoRegisters::kPosPidFf, RegsDefaultValues::kDefPosPidFf);
  WriteRegField(ServoRegisters::kPosPidRamp, RegsDefaultValues::kDefPosPidRamp);
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
  WriteRegField(ServoRegisters::kTorqueEnable, 0);

  // 动作
  WriteRegField(ServoRegisters::kTargetAcceleration, 0);
  WriteRegField(ServoRegisters::kTargetPositionL, 0);
  WriteRegField(ServoRegisters::kTargetPositionH, 0);
  WriteRegField(ServoRegisters::kTargetTimeL, 0);
  WriteRegField(ServoRegisters::kTargetTimeH, 0);
  WriteRegField(ServoRegisters::kTargetVelocityL, 0);
  WriteRegField(ServoRegisters::kTargetVelocityH, 0);

  // 转矩限制
  WriteRegField(ServoRegisters::kTorqueLimitL, GetUint8(ServoRegisters::kMaxTorqueL));
  WriteRegField(ServoRegisters::kTorqueLimitH, GetUint8(ServoRegisters::kMaxTorqueH));

  // 写入锁
  WriteRegField(ServoRegisters::kWriteLock, 0);

  // 当前位置
  WriteRegField(ServoRegisters::kPresentPositionL, 0);
  WriteRegField(ServoRegisters::kPresentPositionH, 0);

  // 当前速度
  WriteRegField(ServoRegisters::kPresentVelocityL, 0);
  WriteRegField(ServoRegisters::kPresentVelocityH, 0);

  // 当前负载
  WriteRegField(ServoRegisters::kPresentLoadL, 0);
  WriteRegField(ServoRegisters::kPresentLoadH, 0);

  // 当前电压
  WriteRegField(ServoRegisters::kPresentVoltage, 0);

  // 当前温度
  WriteRegField(ServoRegisters::kPresentTemperature, 0);

  // 异步写标志
  WriteRegField(ServoRegisters::kAsynchronousWriteSt, 0);

  // 舵机状态
  WriteRegField(ServoRegisters::kStatus, 0);

  // 移动标志
  WriteRegField(ServoRegisters::kMoving, 0);

  // 当前电流
  WriteRegField(ServoRegisters::kPresentCurrentL, 0);
  WriteRegField(ServoRegisters::kPresentCurrentH, 0);
  return Error::kOk;
}
}  // namespace hortor_servo
