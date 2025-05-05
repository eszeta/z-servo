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
#include "inst_types.h"

namespace hortor_servo {
using Regs = ServoRegs;
using Def = RegsDefaultValues;

// 兼容GD32的编译环境，需要手动定义，否则会链接错误
constexpr Register Regs::kFirmwareMajor;
constexpr Register Regs::kFirmwareMinor;
constexpr Register Regs::kEnd;
constexpr Register Regs::kServoMajor;
constexpr Register Regs::kServoMinor;
constexpr Register Regs::kId;
constexpr Register Regs::kBaudrate;
constexpr Register Regs::kResponseDelay;
constexpr Register Regs::kResponseLevel;
constexpr Register Regs::kMinPositionL;
constexpr Register Regs::kMinPositionH;
constexpr Register Regs::kMaxPositionL;
constexpr Register Regs::kMaxPositionH;
constexpr Register Regs::kMaxTemperature;
constexpr Register Regs::kMaxVoltage;
constexpr Register Regs::kMinVoltage;
constexpr Register Regs::kMaxTorqueL;
constexpr Register Regs::kMaxTorqueH;
constexpr Register Regs::kOption;
constexpr Register Regs::kUnloadCondition;
constexpr Register Regs::kLedAlarmCondition;
constexpr Register Regs::kPosPidKp;
constexpr Register Regs::kPosPidKd;
constexpr Register Regs::kPosPidKi;
constexpr Register Regs::kMinStartupForce;
constexpr Register Regs::kPosPidILimit;
constexpr Register Regs::kCWInsensitiveArea;
constexpr Register Regs::kCCWInsensitiveArea;
constexpr Register Regs::kCurrentProtectionThL;
constexpr Register Regs::kCurrentProtectionThH;
constexpr Register Regs::kAngularResolution;
constexpr Register Regs::kPositionCorrectionL;
constexpr Register Regs::kPositionCorrectionH;
constexpr Register Regs::kMode;
constexpr Register Regs::kTorqueProtectionTh;
constexpr Register Regs::kTorqueProtectionTime;
constexpr Register Regs::kOverloadTorque;
constexpr Register Regs::kVelPidKp;
constexpr Register Regs::kOvercurrentProtectionTime;
constexpr Register Regs::kVelPidKi;
constexpr Register Regs::kTorqueEnable;
constexpr Register Regs::kTargetAcceleration;
constexpr Register Regs::kTargetPositionL;
constexpr Register Regs::kTargetPositionH;
constexpr Register Regs::kTargetTimeL;
constexpr Register Regs::kTargetTimeH;
constexpr Register Regs::kTargetVelocityL;
constexpr Register Regs::kTargetVelocityH;
constexpr Register Regs::kTorqueLimitL;
constexpr Register Regs::kTorqueLimitH;
constexpr Register Regs::kWriteLock;
constexpr Register Regs::kPresentPositionL;
constexpr Register Regs::kPresentPositionH;
constexpr Register Regs::kPresentVelocityL;
constexpr Register Regs::kPresentVelocityH;
constexpr Register Regs::kPresentLoadL;
constexpr Register Regs::kPresentLoadH;
constexpr Register Regs::kPresentVoltage;
constexpr Register Regs::kPresentTemperature;
constexpr Register Regs::kAsyncWriteSt;
constexpr Register Regs::kStatus;
constexpr Register Regs::kMoving;
constexpr Register Regs::kPresentCurrentL;
constexpr Register Regs::kPresentCurrentH;
constexpr Register Regs::kSensorDirection;
constexpr Register Regs::kMotorDirection;
constexpr Register Regs::kAdcShuntResistor;
constexpr Register Regs::kAdcCurrentFactor;
constexpr Register Regs::kPosPidKf;
constexpr Register Regs::kPosFilter;
constexpr Register Regs::kCurrentFilter;
constexpr Register Regs::kVelocityFilter;

constexpr uint8_t Def::kDefFirmwareMajor;
constexpr uint8_t Def::kDefFirmwareMinor;
constexpr uint8_t Def::kDefEnd;
constexpr uint8_t Def::kDefServoMajor;
constexpr uint8_t Def::kDefServoMinor;
constexpr uint8_t Def::kDefId;
constexpr uint8_t Def::kDefBaudrate;
constexpr uint16_t Def::kDefResponseDelay;
constexpr bool Def::kDefResponseLevel;
constexpr uint16_t Def::kDefMinPosition;
constexpr uint16_t Def::kDefMaxPosition;
constexpr uint8_t Def::kDefMaxTemperature;
constexpr float Def::kDefMaxVoltage;
constexpr float Def::kDefMinVoltage;
constexpr uint16_t Def::kDefMaxTorque;
constexpr uint8_t Def::kDefOption;
constexpr uint8_t Def::kDefUnloadCondition;
constexpr uint8_t Def::kDefLedAlarmCondition;
constexpr float Def::kDefPosPidKp;
constexpr float Def::kDefPosPidKd;
constexpr float Def::kDefPosPidKi;
constexpr float Def::kDefMinStartupForce;
constexpr float Def::kDefPosPidILimit;
constexpr uint8_t Def::kDefCWInsensitiveArea;
constexpr uint8_t Def::kDefCCWInsensitiveArea;
constexpr float Def::kDefCurrentProtectionTh;
constexpr uint8_t Def::kDefAngularResolution;
constexpr uint16_t Def::kDefPositionCorrection;
constexpr ServoMode Def::kDefMode;
constexpr float Def::kDefTorqueProtectionTh;
constexpr uint16_t Def::kDefTorqueProtectionTime;
constexpr float Def::kDefOverloadTorque;
constexpr float Def::kDefVelPidKp;
constexpr uint16_t Def::kDefOvercurrentProtectionTime;
constexpr float Def::kDefVelPidKi;
constexpr Direction Def::kDefSensorDirection;
constexpr uint16_t Def::kDefAdcShuntResistor;
constexpr uint16_t Def::kDefAdcCurrentFactor;
constexpr float Def::kDefPosPidFf;
constexpr float Def::kDefPosFilter;
constexpr float Def::kDefCurrentFilter;
constexpr float Def::kDefVelocityFilter;

constexpr RegsBlock RegsBlocks::kEeprom;
constexpr RegsBlock RegsBlocks::kNormalRam;
constexpr RegsBlock RegsBlocks::kReadOnlyRam;
constexpr RegsBlock RegsBlocks::kInternalEeprom;
constexpr RegsBlock RegsBlocks::kAction;
constexpr RegsBlock RegsBlocks::kTotal;

constexpr uint8_t Instruction::kPing;
constexpr uint8_t Instruction::kReadData;
constexpr uint8_t Instruction::kWriteData;
constexpr uint8_t Instruction::kRegWrite;
constexpr uint8_t Instruction::kAction;
constexpr uint8_t Instruction::kSyncWrite;
constexpr uint8_t Instruction::kSyncRead;
constexpr uint8_t Instruction::kRecovery;
constexpr uint8_t Instruction::kReset;

}  // namespace hortor_servo