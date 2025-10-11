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

#ifndef EEPROM_DISABLE
#include <EEPROM.h>
#endif

#include "regmap.h"
namespace hortor::servo_slave {

Error RegMap::Init() {
  CHECK(regmap::RegMapMmio::Init(table_, sizeof(table_)));
  CHECK(LoadEeprom());
  return Error::kOk;
}

Error RegMap::GetStatusImpl(uint8_t& status) { return Error::kOk; }
Error RegMap::RecoveryEepromImpl() { return Error::kOk; }
Error RegMap::LoadEepromImpl() { return Error::kOk; }
Error RegMap::StoreEepromImpl() { return Error::kOk; }
Error RegMap::StoreEepromImpl(const uint8_t address, const uint8_t size) {
  return Error::kOk;
}
}  // namespace hortor::servo_slave

// #include "inst/inst_types.h"
// #include "regmap/regmap.h"

// namespace hortor {
// using Regs = ServoRegs;
// using Def = RegsDefaultValues;
// Error InstAccessor::Init() {
//   CHECK(raw_adapter_.Init(regs_, sizeof(regs_)));
//   CHECK(raw_adapter_.LinkAccessor(*this));
//   CHECK(LoadEeprom());
//   if (GetFirmwareMajor() == 0) {
//     CHECK(RecoveryEeprom());
//     CHECK(StoreEeprom());
//   }
//   CHECK(ResetRam());
//   return Error::kOk;
// }

// Error InstAccessor::RecoveryEeprom() {
//   // 固件版本
//   SetFirmwareMajor(Def::kDefFirmwareMajor);
//   SetFirmwareMinor(Def::kDefFirmwareMinor);
//   SetEnd(Def::kDefEnd);
//   SetServoMajor(Def::kDefServoMajor);
//   SetServoMinor(Def::kDefServoMinor);

//   // 基本参数
//   SetId(Def::kDefId);
//   SetBaudrate(Def::kDefBaudrate);
//   SetResponseDelay(Def::kDefResponseDelay);
//   SetResponseLevel(Def::kDefResponseLevel);

//   // 角度限制
//   SetMinPosition(Def::kDefMinPosition);
//   SetMaxPosition(Def::kDefMaxPosition);

//   // 温度和电压限制
//   SetMaxTemperature(Def::kDefMaxTemperature);
//   SetMaxVoltage(Def::kDefMaxVoltage);
//   SetMinVoltage(Def::kDefMinVoltage);

//   // 扭矩相关
//   SetMaxTorque(Def::kDefMaxTorque);

//   // 选项
//   SetOption(Def::kDefOption);

//   // 卸载条件
//   SetUnloadCondition(Def::kDefUnloadCondition);

//   // LED报警条件
//   SetLedAlarmCondition(Def::kDefLedAlarmCondition);

//   // 位置环PID参数
//   SetPosPidKp(Def::kDefPosPidKp);
//   SetPosPidKd(Def::kDefPosPidKd);
//   SetPosPidKi(Def::kDefPosPidKi);
//   SetPosPidILimit(Def::kDefPosPidILimit);

//   // 最小启动力
//   SetMinStartupForce(Def::kDefMinStartupForce);

//   // 不灵敏区设置
//   SetCWInsensitiveArea(Def::kDefCWInsensitiveArea);
//   SetCCWInsensitiveArea(Def::kDefCCWInsensitiveArea);

//   // 电流保护
//   SetCurrentProtectionThreshold(Def::kDefCurrentProtectionTh);
//   SetOvercurrentProtectionTime(Def::kDefOvercurrentProtectionTime);

//   // 角度分辨率
//   SetAngularResolution(Def::kDefAngularResolution);

//   // 位置校正
//   SetPositionCorrection(Def::kDefPositionCorrection);

//   // 运行模式
//   SetMode(Def::kDefMode);

//   // 扭矩保护
//   SetTorqueProtectionThreshold(Def::kDefTorqueProtectionTh);
//   SetTorqueProtectionTime(Def::kDefTorqueProtectionTime);
//   SetOverloadTorque(Def::kDefOverloadTorque);

//   // 速度环PID参数
//   SetVelPidKp(Def::kDefVelPidKp);
//   SetVelPidKi(Def::kDefVelPidKi);

//   //-----------内部EEPROM（读写）-------------------
//   // 传感器方向
//   SetSensorDirection(Def::kDefSensorDirection);
//   // 电机旋转方向
//   SetMotorDirection(Def::kDefMotorDirection);
//   // ADC采样电阻
//   SetAdcShuntResistor(Def::kDefAdcShuntResistor);
//   // ADC电流镜系数
//   SetAdcCurrentFactor(Def::kDefAdcCurrentFactor);

//   // 位置滤波器
//   SetPosFilter(Def::kDefPosFilter);
//   SetCurrentFilter(Def::kDefCurrentFilter);
//   SetVelFilter(Def::kDefVelocityFilter);

//   // 位置PID
//   SetPosPidFf(Def::kDefPosPidFf);
//   return Error::kOk;
// }

// Error InstAccessor::LoadEeprom() {
// #ifndef EEPROM_DISABLE
//   int pos = 0;
//   for (uint8_t address = RegsBlocks::kEeprom.begin;
//        address < RegsBlocks::kEeprom.end;
//        address++) {
//     regs_[address] = EEPROM.read(pos++);
//   }
//   for (uint8_t address = RegsBlocks::kInternalEeprom.begin;
//        address < RegsBlocks::kInternalEeprom.end;
//        address++) {
//     regs_[address] = EEPROM.read(pos++);
//   }
// #endif
//   return Error::kOk;
// }

// Error InstAccessor::StoreEeprom() {
// #ifndef EEPROM_DISABLE
//   int pos = 0;
//   for (uint8_t address = RegsBlocks::kEeprom.begin;
//        address < RegsBlocks::kEeprom.end;
//        address++) {
//     EEPROM.update(pos++, regs_[address]);
//   }
//   for (uint8_t address = RegsBlocks::kInternalEeprom.begin;
//        address < RegsBlocks::kInternalEeprom.end;
//        address++) {
//     EEPROM.update(pos++, regs_[address]);
//   }
// #endif
//   return Error::kOk;
// }

// Error InstAccessor::ResetRam() {
//   // 扭矩开关
//   SetTorqueEnable(false);

//   // 动作
//   SetGoalAcceleration(0);
//   SetGoalPosition(0);
//   SetGoalTime(0);
//   SetGoalVelocity(0);

//   // 转矩限制
//   SetTorqueLimit(GetMaxTorque());

//   // 写入锁
//   SetWriteLock(false);

//   // 当前位置
//   SetPresentPosition(0);

//   // 当前速度
//   SetPresentVelocity(0);

//   // 当前负载
//   SetPresentLoad(0);

//   // 当前电压
//   SetPresentVoltage(0);

//   // 当前温度
//   SetPresentTemperature(0);

//   // 异步写标志
//   SetAsyncWrite(false);

//   // 舵机状态
//   SetStatus(0);

//   // 移动标志
//   SetMoving(false);

//   // 当前电流
//   SetPresentCurrent(0);
//   return Error::kOk;
// }
// }  // namespace hortor
