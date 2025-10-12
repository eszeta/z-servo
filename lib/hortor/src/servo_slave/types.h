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

#include "protocol/types.h"

namespace hortor::servo_slave {

using protocol::ControlTableBlock;

constexpr uint32_t kBaudrateTable[] = {
    2000000, 1000000, 500000, 250000, 115200, 57600, 38400, 9600};
namespace ControlTable {
// constexpr 工厂函数，用于创建 ControlTableItem
constexpr auto CTI8(uint8_t addr, uint8_t val = 0) {
  return protocol::ControlTableItem<uint8_t>(addr, 0, 8, val);
}
constexpr auto CTI16(uint16_t addr, uint16_t val = 0) {
  return protocol::ControlTableItem<uint16_t>(addr, 0, 16, val);
}
constexpr auto CTI32(uint32_t addr, uint32_t val = 0) {
  return protocol::ControlTableItem<uint32_t>(addr, 0, 32, val);
}
/*-------------------- EEPROM 区（掉电保存） --------------------*/
/** @brief 型号 (只读) */
constexpr auto kModelNumber = CTI16(0, 360);
/** @brief 型号信息 (只读) */
constexpr auto kModelInformation = CTI32(2, 0);
/** @brief 固件版本 (只读) */
constexpr auto kFirmwareVersion = CTI8(6, 0);
/** @brief 舵机ID，范围: 0-252 */
constexpr auto kId = CTI8(7, 0);
/** @brief 波特率索引 */
constexpr auto kBaudRate = CTI8(8, 0);
/** @brief 返回延迟，单位: 2μs */
constexpr auto kReturnDelayTime = CTI8(9, 0);
/** @brief 驱动模式 */
constexpr auto kDriveMode = CTI8(10, 0);
/** @brief 工作模式 */
constexpr auto kOperatingMode = CTI8(11, 0);
/** @brief 副ID，范围: 0-252 */
constexpr auto kSecondaryId = CTI8(12, 0);
/** @brief 协议版本 */
constexpr auto kProtocolVersion = CTI8(13, 0);
/** @brief 归零偏移，单位: pulse */
constexpr auto kHomingOffset = CTI32(20, 0);
/** @brief 运动阈值，单位: rev/min */
constexpr auto kMovingThreshold = CTI32(24, 0);
/** @brief 温度上限，单位: °C */
constexpr auto kTemperatureLimit = CTI8(31, 0);
/** @brief 最高电压限制，单位: 0.1V */
constexpr auto kMaxVoltageLimit = CTI16(32, 0);
/** @brief 最低电压限制，单位: 0.1V */
constexpr auto kMinVoltageLimit = CTI16(34, 0);
/** @brief PWM上限，单位: 0.113% */
constexpr auto kPwmLimit = CTI16(36, 0);
/** @brief 电流上限，单位: mA */
constexpr auto kCurrentLimit = CTI16(38, 0);
/** @brief 速度上限，单位: rev/min */
constexpr auto kVelocityLimit = CTI32(44, 0);
/** @brief 位置上限，单位: pulse */
constexpr auto kMaxPositionLimit = CTI32(48, 0);
/** @brief 位置下限，单位: pulse */
constexpr auto kMinPositionLimit = CTI32(52, 0);
/** @brief 关断条件 */
constexpr auto kShutdown = CTI8(63, 0);

/*-------------------- RAM 区（掉电不保存） --------------------*/
/** @brief 力矩使能 */
constexpr auto kTorqueEnable = CTI8(64, 0);
/** @brief LED开关 */
constexpr auto kDxlLed = CTI8(65, 0);
/** @brief 状态返回级别 */
constexpr auto kStatusReturnLevel = CTI8(68, 0);
/** @brief 已注册指令 (只读) */
constexpr auto kRegisteredInstruction = CTI8(69, 0);
/** @brief 硬件错误状态 (只读) */
constexpr auto kHardwareErrorStatus = CTI8(70, 0);
/** @brief 速度积分增益 */
constexpr auto kVelocityIgain = CTI16(76, 0);
/** @brief 速度比例增益 */
constexpr auto kVelocityPgain = CTI16(78, 0);
/** @brief 位置微分增益 */
constexpr auto kPositionDgain = CTI16(80, 0);
/** @brief 位置积分增益 */
constexpr auto kPositionIgain = CTI16(82, 0);
/** @brief 位置比例增益 */
constexpr auto kPositionPgain = CTI16(84, 0);
/** @brief 前馈二阶增益 */
constexpr auto kFeedforward2ndGain = CTI16(88, 0);
/** @brief 前馈一阶增益 */
constexpr auto kFeedforward1stGain = CTI16(90, 0);
/** @brief 总线看门狗，单位: 20ms */
constexpr auto kBusWatchdog = CTI8(98, 0);
/** @brief 目标PWM，单位: 0.113% */
constexpr auto kGoalPwm = CTI16(100, 0);
/** @brief 目标电流，单位: mA */
constexpr auto kGoalCurrent = CTI16(102, 0);
/** @brief 目标速度，单位: rev/min */
constexpr auto kGoalVelocity = CTI32(104, 0);
/** @brief 轨迹加速度，单位: rev/min² */
constexpr auto kProfileAcceleration = CTI32(108, 0);
/** @brief 轨迹速度，单位: rev/min */
constexpr auto kProfileVelocity = CTI32(112, 0);
/** @brief 目标位置，单位: pulse */
constexpr auto kGoalPosition = CTI32(116, 0);
/** @brief 实时时钟 (只读)，单位: ms */
constexpr auto kRealtimeTick = CTI16(120, 0);
/** @brief 运动状态 (只读) */
constexpr auto kMoving = CTI8(122, 0);
/** @brief 运动详细状态 (只读) */
constexpr auto kMovingStatus = CTI8(123, 0);
/** @brief 当前PWM (只读)，单位: 0.113% */
constexpr auto kPresentPwm = CTI16(124, 0);
/** @brief 当前电流 (只读)，单位: mA */
constexpr auto kPresentCurrent = CTI16(126, 0);
/** @brief 当前速度 (只读)，单位: rev/min */
constexpr auto kPresentVelocity = CTI32(128, 0);
/** @brief 当前位置 (只读)，单位: pulse */
constexpr auto kPresentPosition = CTI32(132, 0);
/** @brief 速度轨迹 (只读)，单位: rev/min */
constexpr auto kVelocityTrajectory = CTI32(136, 0);
/** @brief 位置轨迹 (只读)，单位: pulse */
constexpr auto kPositionTrajectory = CTI32(140, 0);
/** @brief 当前输入电压 (只读)，单位: 0.1V */
constexpr auto kPresentInputVoltage = CTI16(144, 0);
/** @brief 当前温度 (只读)，单位: °C */
constexpr auto kPresentTemperature = CTI8(146, 0);
};  // namespace ControlTable

namespace TableBlocks {
constexpr static ControlTableBlock kEeprom = {
    ControlTable::kModelNumber.reg.address,
    ControlTable::kShutdown.reg.address +
        ControlTable::kShutdown.reg.getSize()};

constexpr static ControlTableBlock kRam = {
    ControlTable::kTorqueEnable.reg.address,
    ControlTable::kPresentTemperature.reg.address +
        ControlTable::kPresentTemperature.reg.getSize()};

constexpr static ControlTableBlock kTotal = {
    ControlTable::kModelNumber.reg.address,
    ControlTable::kPresentTemperature.reg.address +
        ControlTable::kPresentTemperature.reg.getSize()};
};  // namespace TableBlocks
}  // namespace hortor::servo_slave