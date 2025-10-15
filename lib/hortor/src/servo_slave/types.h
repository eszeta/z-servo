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
    9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000};

/**
 * @brief 控制表
 * 仿照XL330的控制表
 */
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
/** @brief 型号编号 | 单位: - | 访问: R | 初始值: 1190 */
constexpr auto kModelNumber = CTI16(0, 1190);
/** @brief 型号信息 | 单位: - | 访问: R | 初始值: 0 */
constexpr auto kModelInformation = CTI32(2, 0);
/** @brief 固件版本 | 单位: - | 访问: R | 初始值: 0 */
constexpr auto kFirmwareVersion = CTI8(6, 0);
/** @brief 舵机ID (0-252) | 单位: - | 访问: RW | 初始值: 1 */
constexpr auto kId = CTI8(7, 1);
/** @brief 波特率索引 (0-7) | 单位: - | 访问: RW | 初始值: 3 */
constexpr auto kBaudRate = CTI8(8, 3);
/** @brief 返回延迟 | 单位: 2μs | 访问: RW | 初始值: 250 */
constexpr auto kReturnDelayTime = CTI8(9, 250);
/** @brief 驱动模式 | 单位: - | 访问: RW | 初始值: 0 */
constexpr auto kDriveMode = CTI8(10, 0);
/** @brief 工作模式 (0,1,3-5,16) | 单位: - | 访问: RW | 初始值: 3 */
constexpr auto kOperatingMode = CTI8(11, 3);
/** @brief 副ID (0-252) | 单位: - | 访问: RW | 初始值: 0 */
constexpr auto kSecondaryId = CTI8(12, 0);
/** @brief 协议版本 | 单位: - | 访问: RW | 初始值: 2 */
constexpr auto kProtocolVersion = CTI8(13, 2);
/** @brief 归零偏移 | 单位: pulse | 访问: RW | 初始值: 0 */
constexpr auto kHomingOffset = CTI32(20, 0);
/** @brief 运动阈值 | 单位: 0.229 rev/min | 访问: RW | 初始值: 10 */
constexpr auto kMovingThreshold = CTI32(24, 10);
/** @brief 温度上限 | 单位: °C | 访问: RW | 初始值: 72 */
constexpr auto kTemperatureLimit = CTI8(31, 72);
/** @brief 最高电压限制 | 单位: 0.1V | 访问: RW | 初始值: 160 */
constexpr auto kMaxVoltageLimit = CTI16(32, 160);
/** @brief 最低电压限制 | 单位: 0.1V | 访问: RW | 初始值: 50 */
constexpr auto kMinVoltageLimit = CTI16(34, 50);
/** @brief PWM上限 | 单位: 0.113% | 访问: RW | 初始值: 885 */
constexpr auto kPwmLimit = CTI16(36, 885);
/** @brief 电流上限 | 单位: mA | 访问: RW | 初始值: 1193 */
constexpr auto kCurrentLimit = CTI16(38, 1193);
/** @brief 速度上限 | 单位: 0.229 rev/min | 访问: RW | 初始值: 2047 */
constexpr auto kVelocityLimit = CTI32(44, 2047);
/** @brief 位置上限 | 单位: pulse | 访问: RW | 初始值: 4095 */
constexpr auto kMaxPositionLimit = CTI32(48, 4095);
/** @brief 位置下限 | 单位: pulse | 访问: RW | 初始值: 0 */
constexpr auto kMinPositionLimit = CTI32(52, 0);
/** @brief 关断条件 | 单位: - | 访问: RW | 初始值: 52 */
constexpr auto kShutdown = CTI8(63, 52);

/*-------------------- RAM 区（掉电不保存） --------------------*/
/** @brief 力矩使能 (0/1) | 单位: - | 访问: RW */
constexpr auto kTorqueEnable = CTI8(64, 0);
/** @brief LED开关 (0/1) | 单位: - | 访问: RW */
constexpr auto kDxlLed = CTI8(65, 0);
/** @brief 状态返回级别 (0-2) | 单位: - | 访问: RW | 初始值: 2 */
constexpr auto kStatusReturnLevel = CTI8(68, 2);
/** @brief 已注册指令 | 单位: - | 访问: R */
constexpr auto kRegisteredInstruction = CTI8(69, 0);
/** @brief 硬件错误状态 | 单位: - | 访问: R */
constexpr auto kHardwareErrorStatus = CTI8(70, 0);
/** @brief 速度积分增益 | 单位: - | 访问: RW | 初始值: 1920 */
constexpr auto kVelocityIgain = CTI16(76, 1920);
/** @brief 速度比例增益 | 单位: - | 访问: RW | 初始值: 100 */
constexpr auto kVelocityPgain = CTI16(78, 100);
/** @brief 位置微分增益 | 单位: - | 访问: RW | 初始值: 4000 (M077: 250.0) */
constexpr auto kPositionDgain = CTI16(80, 4000);
/** @brief 位置积分增益 | 单位: - | 访问: RW | 初始值: 0 */
constexpr auto kPositionIgain = CTI16(82, 0);
/** @brief 位置比例增益 | 单位: - | 访问: RW | 初始值: 800 (M077: 6.25) */
constexpr auto kPositionPgain = CTI16(84, 800);
/** @brief 前馈二阶增益 | 单位: - | 访问: RW */
constexpr auto kFeedforward2ndGain = CTI16(88, 0);
/** @brief 前馈一阶增益 | 单位: - | 访问: RW */
constexpr auto kFeedforward1stGain = CTI16(90, 0);
/** @brief 总线看门狗 | 单位: 20ms | 访问: RW */
constexpr auto kBusWatchdog = CTI8(98, 0);
/** @brief 目标PWM | 单位: 0.113% | 访问: RW */
constexpr auto kGoalPwm = CTI16(100, 0);
/** @brief 目标电流 | 单位: mA | 访问: RW */
constexpr auto kGoalCurrent = CTI16(102, 0);
/** @brief 目标速度 | 单位: 0.229 rev/min | 访问: RW */
constexpr auto kGoalVelocity = CTI32(104, 0);
/** @brief 轨迹加速度 | 单位: Velocity-based: 214.577 rev/min², Time-based: 1ms | 访问: RW */
constexpr auto kProfileAcceleration = CTI32(108, 0);
/** @brief 轨迹速度 | 单位: Velocity-based: 0.229 rev/min, Time-based: 1ms | 访问: RW */
constexpr auto kProfileVelocity = CTI32(112, 0);
/** @brief 目标位置 | 单位: pulse (0-4095=0-360°) | 访问: RW */
constexpr auto kGoalPosition = CTI32(116, 0);
/** @brief 实时时钟 | 单位: ms | 访问: R */
constexpr auto kRealtimeTick = CTI16(120, 0);
/** @brief 运动状态 (0/1) | 单位: - | 访问: R */
constexpr auto kMoving = CTI8(122, 0);
/** @brief 运动详细状态 | 单位: - | 访问: R */
constexpr auto kMovingStatus = CTI8(123, 0);
/** @brief 当前PWM | 单位: 0.113% | 访问: R */
constexpr auto kPresentPwm = CTI16(124, 0);
/** @brief 当前电流 | 单位: mA | 访问: R */
constexpr auto kPresentCurrent = CTI16(126, 0);
/** @brief 当前速度 | 单位: 0.229 rev/min | 访问: R */
constexpr auto kPresentVelocity = CTI32(128, 0);
/** @brief 当前位置 | 单位: pulse (0-4095=0-360°) | 访问: R */
constexpr auto kPresentPosition = CTI32(132, 0);
/** @brief 速度轨迹 | 单位: 0.229 rev/min | 访问: R */
constexpr auto kVelocityTrajectory = CTI32(136, 0);
/** @brief 位置轨迹 | 单位: pulse | 访问: R */
constexpr auto kPositionTrajectory = CTI32(140, 0);
/** @brief 当前输入电压 | 单位: 0.1V | 访问: R */
constexpr auto kPresentInputVoltage = CTI16(144, 0);
/** @brief 当前温度 | 单位: °C | 访问: R */
constexpr auto kPresentTemperature = CTI8(146, 0);

constexpr static size_t kTotalSize = kPresentTemperature.reg.address + kPresentTemperature.reg.kSize;
};  // namespace ControlTable

namespace TableBlocks {
constexpr static ControlTableBlock kEeprom = {
    ControlTable::kModelNumber.reg.address,
    ControlTable::kShutdown.reg.address +
        ControlTable::kShutdown.reg.kSize};

constexpr static ControlTableBlock kRam = {
    ControlTable::kTorqueEnable.reg.address,
    ControlTable::kPresentTemperature.reg.address +
        ControlTable::kPresentTemperature.reg.kSize};

constexpr static ControlTableBlock kTotal = {
    ControlTable::kModelNumber.reg.address,
    ControlTable::kPresentTemperature.reg.address +
        ControlTable::kPresentTemperature.reg.kSize};
};  // namespace TableBlocks
}  // namespace hortor::servo_slave