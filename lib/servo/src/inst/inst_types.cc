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
constexpr Register Regs::kPosProportionalGain;
constexpr Register Regs::kPosDerivativeGain;
constexpr Register Regs::kPosIntegralGain;
constexpr Register Regs::kMinStartupForce;
constexpr Register Regs::kIntegralLimit;
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
constexpr Register Regs::kVelocityProportionalGain;
constexpr Register Regs::kOvercurrentProtectionTime;
constexpr Register Regs::kVelocityIntegralGain;
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
constexpr Register Regs::kAsynchronousWriteSt;
constexpr Register Regs::kStatus;
constexpr Register Regs::kMoving;
constexpr Register Regs::kPresentCurrentL;
constexpr Register Regs::kPresentCurrentH;
constexpr Register Regs::kMotorDirection;
constexpr Register Regs::kSensorDirection;
constexpr Register Regs::kAdcShuntResistor;
constexpr Register Regs::kAdcCurrentFactor;
constexpr Register Regs::kPosFilter;
constexpr Register Regs::kCurrentFilter;
constexpr Register Regs::kVelocityFilter;
constexpr Register Regs::kPosPidFf;
constexpr Register Regs::kPosPidRamp;

constexpr uint8_t RegsDefaultValues::kDefFirmwareMajor;
constexpr uint8_t RegsDefaultValues::kDefFirmwareMinor;
constexpr uint8_t RegsDefaultValues::kDefServoMajor;
constexpr uint8_t RegsDefaultValues::kDefServoMinor;
constexpr uint8_t RegsDefaultValues::kDefId;
constexpr uint8_t RegsDefaultValues::kDefBaudrate;
constexpr uint8_t RegsDefaultValues::kDefResponseDelay;
constexpr uint8_t RegsDefaultValues::kDefResponseStatusLevel;
constexpr uint8_t RegsDefaultValues::kDefMinPositionL;
constexpr uint8_t RegsDefaultValues::kDefMinPositionH;
constexpr uint8_t RegsDefaultValues::kDefMaxPositionL;
constexpr uint8_t RegsDefaultValues::kDefMaxPositionH;
constexpr uint8_t RegsDefaultValues::kDefMaxTemperature;
constexpr uint8_t RegsDefaultValues::kDefMaxVoltage;
constexpr uint8_t RegsDefaultValues::kDefMinVoltage;
constexpr uint8_t RegsDefaultValues::kDefMaxTorqueL;
constexpr uint8_t RegsDefaultValues::kDefMaxTorqueH;
constexpr uint8_t RegsDefaultValues::kDefPosProportionalGain;
constexpr uint8_t RegsDefaultValues::kDefPosDerivativeGain;
constexpr uint8_t RegsDefaultValues::kDefPosIntegralGain;
constexpr uint8_t RegsDefaultValues::kDefMinStartupForce;
constexpr uint8_t RegsDefaultValues::kDefIntegralLimit;
constexpr uint8_t RegsDefaultValues::kDefCWInsensitiveArea;
constexpr uint8_t RegsDefaultValues::kDefCCWInsensitiveArea;
constexpr uint8_t RegsDefaultValues::kDefCurrentProtectionThL;
constexpr uint8_t RegsDefaultValues::kDefCurrentProtectionThH;
constexpr uint8_t RegsDefaultValues::kDefAngularResolution;
constexpr uint8_t RegsDefaultValues::kDefPositionCorrectionL;
constexpr uint8_t RegsDefaultValues::kDefPositionCorrectionH;
constexpr uint8_t RegsDefaultValues::kDefMode;
constexpr uint8_t RegsDefaultValues::kDefTorqueProtectionTh;
constexpr uint8_t RegsDefaultValues::kDefTorqueProtectionTime;
constexpr uint8_t RegsDefaultValues::kDefOverloadTorque;
constexpr uint8_t RegsDefaultValues::kDefVelocityProportionalGain;
constexpr uint8_t RegsDefaultValues::kDefOvercurrentProtectionTime;
constexpr uint8_t RegsDefaultValues::kDefVelocityIntegralGain;
constexpr uint8_t RegsDefaultValues::kDefEnd;
constexpr uint8_t RegsDefaultValues::kDefOption;
constexpr uint8_t RegsDefaultValues::kDefUnloadCondition;
constexpr uint8_t RegsDefaultValues::kDefLedAlarmCondition;
constexpr uint8_t RegsDefaultValues::kDefMotorDirection;
constexpr uint8_t RegsDefaultValues::kDefSensorDirection;
constexpr uint8_t RegsDefaultValues::kDefAdcShuntResistor;
constexpr uint8_t RegsDefaultValues::kDefAdcCurrentFactor;
constexpr uint8_t RegsDefaultValues::kDefPosFilter;
constexpr uint8_t RegsDefaultValues::kDefCurrentFilter;
constexpr uint8_t RegsDefaultValues::kDefVelocityFilter;
constexpr uint8_t RegsDefaultValues::kDefPosPidFf;
constexpr uint8_t RegsDefaultValues::kDefPosPidRamp;

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