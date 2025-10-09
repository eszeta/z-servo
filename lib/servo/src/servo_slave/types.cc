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

#include "types.h"

namespace hortor_servo {

/*-------------------- EEPROM 区（掉电保存） --------------------*/
constexpr ControlTableItem<uint16_t> ControlTable::kModelNumber;
constexpr ControlTableItem<uint8_t> ControlTable::kFirmwareVersion;
constexpr ControlTableItem<uint8_t> ControlTable::kId;
constexpr ControlTableItem<uint8_t> ControlTable::kBaudRate;
constexpr ControlTableItem<uint8_t> ControlTable::kReturnDelayTime;
constexpr ControlTableItem<uint16_t> ControlTable::kCwAngleLimit;
constexpr ControlTableItem<uint16_t> ControlTable::kCcwAngleLimit;
constexpr ControlTableItem<uint16_t> ControlTable::kCcwAngleLimit;
constexpr ControlTableItem<uint8_t> ControlTable::kTemperatureLimit;
constexpr ControlTableItem<uint8_t> ControlTable::kMinVoltageLimit;
constexpr ControlTableItem<uint8_t> ControlTable::kMaxVoltageLimit;
constexpr ControlTableItem<uint16_t> ControlTable::kMaxTorque;
constexpr ControlTableItem<uint8_t> ControlTable::kStatusReturnLevel;
constexpr ControlTableItem<uint8_t> ControlTable::kAlarmLed;
constexpr ControlTableItem<uint8_t> ControlTable::kShutdown;
constexpr ControlTableItem<uint16_t> ControlTable::kMultiTurnOffset;
constexpr ControlTableItem<uint8_t> ControlTable::kResolutionDivider;

/*-------------------- RAM 区（掉电丢失） --------------------*/
constexpr ControlTableItem<uint8_t> ControlTable::kTorqueEnable;
constexpr ControlTableItem<uint8_t> ControlTable::kDxlLed;
constexpr ControlTableItem<uint8_t> ControlTable::kPosDgain;
constexpr ControlTableItem<uint8_t> ControlTable::kPosIgain;
constexpr ControlTableItem<uint8_t> ControlTable::kPosPgain;
constexpr ControlTableItem<uint16_t> ControlTable::kGoalPosition;
constexpr ControlTableItem<uint16_t> ControlTable::kMovingSpeed;
constexpr ControlTableItem<uint16_t> ControlTable::kTorqueLimit;
constexpr ControlTableItem<uint16_t> ControlTable::kPresentPosition;
constexpr ControlTableItem<uint16_t> ControlTable::kPresentSpeed;
constexpr ControlTableItem<uint16_t> ControlTable::kPresentLoad;
constexpr ControlTableItem<uint8_t> ControlTable::kPresentVoltage;
constexpr ControlTableItem<uint8_t> ControlTable::kPresentTemperature;
constexpr ControlTableItem<uint8_t> ControlTable::kRegistered;
constexpr ControlTableItem<uint8_t> ControlTable::kMoving;
constexpr ControlTableItem<uint8_t> ControlTable::kLock;
constexpr ControlTableItem<uint16_t> ControlTable::kPunch;
constexpr ControlTableItem<uint16_t> ControlTable::kRealtimeTick;
constexpr ControlTableItem<uint8_t> ControlTable::kGoalAcceleration;

constexpr ControlTableBlock TableBlocks::kEeprom;
constexpr ControlTableBlock TableBlocks::kRam;
constexpr ControlTableBlock TableBlocks::kTotal;

}  // namespace hortor_servo