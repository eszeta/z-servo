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
using Def = RegsDefaultValues;
Error InstAccessor::Init() {
  CHECK_ERROR(local_transport_.Init(regs_, sizeof(regs_)));
  CHECK_ERROR(local_transport_.LinkAccessor(*this));
  CHECK_ERROR(LoadEeprom());
  if (GetFirmwareMajor() == 0 && GetFirmwareMinor() == 0) {
    CHECK_ERROR(RecoveryEeprom());
    CHECK_ERROR(StoreEeprom());
  }
  CHECK_ERROR(ResetRam());
  return Error::kOk;
}

Error InstAccessor::RecoveryEeprom() {
  // 固件版本
  SetFirmwareMajor(Def::kDefFirmwareMajor);
  SetFirmwareMinor(Def::kDefFirmwareMinor);
  SetEnd(Def::kDefEnd);
  SetServoMajor(Def::kDefServoMajor);
  SetServoMinor(Def::kDefServoMinor);

  // 基本参数
  SetId(Def::kDefId);
  SetBaudrate(Def::kDefBaudrate);
  SetResponseDelay(Def::kDefResponseDelay);
  SetResponseLevel(Def::kDefResponseLevel);

  // 角度限制
  SetMinPosition(Def::kDefMinPosition);
  SetMaxPosition(Def::kDefMaxPosition);

  // 温度和电压限制
  SetMaxTemperature(Def::kDefMaxTemperature);
  SetMaxVoltage(Def::kDefMaxVoltage);
  SetMinVoltage(Def::kDefMinVoltage);

  // 扭矩相关
  SetMaxTorque(Def::kDefMaxTorque);

  // 选项
  SetOption(Def::kDefOption);

  // 卸载条件
  SetUnloadCondition(Def::kDefUnloadCondition);

  // LED报警条件
  SetLedAlarmCondition(Def::kDefLedAlarmCondition);

  // 位置环PID参数
  SetPosPidKp(Def::kDefPosPidKp);
  SetPosPidKd(Def::kDefPosPidKd);
  SetPosPidKi(Def::kDefPosPidKi);
  SetPosPidLimit(Def::kDefPosPidLimit);

  // 最小启动力
  SetMinStartupForce(Def::kDefMinStartupForce);

  // 不灵敏区设置
  SetCWInsensitiveArea(Def::kDefCWInsensitiveArea);
  SetCCWInsensitiveArea(Def::kDefCCWInsensitiveArea);

  // 电流保护
  SetCurrentProtectionThreshold(Def::kDefCurrentProtectionTh);
  SetOvercurrentProtectionTime(Def::kDefOvercurrentProtectionTime);

  // 角度分辨率
  SetAngularResolution(Def::kDefAngularResolution);

  // 位置校正
  SetPositionCorrection(Def::kDefPositionCorrection);

  // 运行模式
  SetMode(Def::kDefMode);

  // 扭矩保护
  SetTorqueProtectionThreshold(Def::kDefTorqueProtectionTh);
  SetTorqueProtectionTime(Def::kDefTorqueProtectionTime);
  SetOverloadTorque(Def::kDefOverloadTorque);

  // 速度环PID参数
  SetVelPidKp(Def::kDefVelPidKp);
  SetVelPidKi(Def::kDefVelPidKi);

  //-----------内部EEPROM（读写）-------------------
  //
  SetMotorDirection(Def::kDefMotorDirection);
  SetSensorDirection(Def::kDefSensorDirection);
  SetAdcShuntResistor(Def::kDefAdcShuntResistor);
  SetAdcCurrentFactor(Def::kDefAdcCurrentFactor);

  // 位置滤波器
  SetPosFilter(Def::kDefPosFilter);
  SetCurrentFilter(Def::kDefCurrentFilter);
  SetVelFilter(Def::kDefVelocityFilter);

  // 位置PID
  SetPosPidFf(Def::kDefPosPidFf);
  SetPosPidRamp(Def::kDefPosPidRamp);
  return Error::kOk;
}

Error InstAccessor::LoadEeprom() {
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
  SetTorqueEnable(false);

  // 动作
  SetTargetAcceleration(0);
  SetTargetPosition(0);
  SetTargetTime(0);
  SetTargetVelocity(0);

  // 转矩限制
  SetTorqueLimit(GetMaxTorque());

  // 写入锁
  SetWriteLock(false);

  // 当前位置
  SetPresentPosition(0);

  // 当前速度
  SetPresentVelocity(0);

  // 当前负载
  SetPresentLoad(0);

  // 当前电压
  SetPresentVoltage(0);

  // 当前温度
  SetPresentTemperature(0);

  // 异步写标志
  SetAsyncWrite(false);

  // 舵机状态
  SetStatus(0);

  // 移动标志
  SetMoving(false);

  // 当前电流
  SetPresentCurrent(0);
  return Error::kOk;
}
}  // namespace hortor_servo
