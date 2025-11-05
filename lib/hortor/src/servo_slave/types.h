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
/* 设备信息组 (0x00-0x0F, 16字节) */
/** @brief 型号编号 | 单位: - | 访问: R */
constexpr auto kModelNumber = CTI16(0x00, 1190);
/** @brief 型号信息 | 单位: - | 访问: R */
constexpr auto kModelInformation = CTI32(0x02, 0);
/** @brief 固件版本 | 单位: - | 访问: R */
constexpr auto kFirmwareVersion = CTI8(0x06, 0);
/** @brief 舵机ID (0-252) | 单位: - | 访问: RW */
constexpr auto kId = CTI8(0x07, 1);
/* 0x08-0x0F: 保留，用于设备信息组扩展 */

/* 通信配置组 (0x10-0x1F, 16字节) */
/** @brief 波特率索引 (0-7) | 单位: - | 访问: RW */
constexpr auto kBaudRate = CTI8(0x10, 3);
/** @brief 返回延迟 | 单位: 2μs | 访问: RW */
constexpr auto kReturnDelayTime = CTI8(0x11, 250);
/** @brief 状态返回级别 (0-2) | 单位: - | 访问: RW */
constexpr auto kStatusReturnLevel = CTI8(0x12, 2);
/* 0x13-0x1F: 保留，用于通信配置组扩展 */

/* 运行模式组 (0x20-0x2F, 16字节) */
/** @brief 驱动模式 | 单位: - | 访问: RW */
constexpr auto kDriveMode = CTI8(0x20, 0);
/** @brief 工作模式 (0,1,3-5,16) | 单位: - | 访问: RW */
constexpr auto kOperatingMode = CTI8(0x21, 3);
/** @brief 关断条件 | 单位: - | 访问: RW */
constexpr auto kShutdown = CTI8(0x22, 52);
/* 0x23-0x2F: 保留，用于运行模式组扩展 */

/* 位置配置组 (0x30-0x3F, 16字节) */
/** @brief 归零偏移 | 单位: pulse | 访问: RW */
constexpr auto kHomingOffset = CTI32(0x30, 0);
/** @brief 运动阈值 | 单位: 0.229 rev/min | 访问: RW */
constexpr auto kMovingThreshold = CTI32(0x34, 10);
/* 0x38-0x3F: 保留，用于位置配置组扩展 */

/* 保护限制组 (0x40-0x5F, 32字节) */
/** @brief 温度上限 | 单位: °C | 访问: RW */
constexpr auto kTemperatureLimit = CTI8(0x40, 72);
/** @brief 最高电压限制 | 单位: 0.1V | 访问: RW */
constexpr auto kMaxVoltageLimit = CTI16(0x41, 160);
/** @brief 最低电压限制 | 单位: 0.1V | 访问: RW */
constexpr auto kMinVoltageLimit = CTI16(0x43, 50);
/** @brief PWM上限 | 单位: 0.113% | 访问: RW */
constexpr auto kPwmLimit = CTI16(0x45, 885);
/** @brief 电流上限 | 单位: 0.001A | 访问: RW */
constexpr auto kCurrentLimit = CTI16(0x47, 3000);
/** @brief 速度上限 | 单位: 0.229 rev/min | 访问: RW */
constexpr auto kVelocityLimit = CTI32(0x49, 2047);
/** @brief 位置上限 | 单位: pulse | 访问: RW */
constexpr auto kMaxPositionLimit = CTI32(0x4D, 4095);
/** @brief 位置下限 | 单位: pulse | 访问: RW */
constexpr auto kMinPositionLimit = CTI32(0x51, 0);
/* 0x55-0x5F: 保留，用于保护限制组扩展 */

/* PID 参数组 (0x60-0x6F, 16字节) */
/** @brief 速度积分增益 | 单位: - | 访问: RW */
constexpr auto kVelocityIgain = CTI16(0x60, 1920);
/** @brief 速度比例增益 | 单位: - | 访问: RW */
constexpr auto kVelocityPgain = CTI16(0x62, 100);
/** @brief 位置微分增益 | 单位: - | 访问: RW */
constexpr auto kPositionDgain = CTI16(0x64, 4000);
/** @brief 位置积分增益 | 单位: - | 访问: RW */
constexpr auto kPositionIgain = CTI16(0x66, 0);
/** @brief 位置比例增益 | 单位: - | 访问: RW */
constexpr auto kPositionPgain = CTI16(0x68, 800);
/** @brief 前馈二阶增益 | 单位: - | 访问: RW */
constexpr auto kFeedforward2ndGain = CTI16(0x6A, 0);
/** @brief 前馈一阶增益 | 单位: - | 访问: RW */
constexpr auto kFeedforward1stGain = CTI16(0x6C, 0);
/* 0x6E-0x6F: 保留，用于PID参数组扩展 */

/* 轨迹配置组 (0x70-0x7F, 16字节) */
/** @brief 轨迹加速度 | 单位: Velocity-based: 214.577 rev/min², Time-based: 1ms
 * | 访问: RW */
constexpr auto kProfileAcceleration = CTI32(0x70, 0);
/** @brief 轨迹速度 | 单位: Velocity-based: 0.229 rev/min, Time-based: 1ms |
 * 访问: RW */
constexpr auto kProfileVelocity = CTI32(0x74, 0);
/* 0x78-0x7F: 保留，用于轨迹配置组扩展 */

/*-------------------- RAM 区（掉电不保存） --------------------*/
/* 控制命令组 (0x80-0x8F, 16字节) */
/** @brief 力矩使能 (0/1) | 单位: - | 访问: RW */
constexpr auto kTorqueEnable = CTI8(0x80, 0);
/** @brief LED开关 (0/1) | 单位: - | 访问: RW */
constexpr auto kDxlLed = CTI8(0x81, 0);
/** @brief 设置为居中位置 | 单位: - | 访问: W */
constexpr auto kSetToCenter = CTI8(0x82, 0);
/** @brief 硬件错误状态 | 单位: - | 访问: R */
constexpr auto kHardwareErrorStatus = CTI8(0x83, 0);
/** @brief 总线看门狗 | 单位: 20ms | 访问: RW */
constexpr auto kBusWatchdog = CTI8(0x84, 0);
/* 0x85-0x8F: 保留，用于控制命令组扩展 */

/* 目标值组 (0x90-0x9F, 16字节) */
/** @brief 目标PWM | 单位: 0.113% | 访问: RW */
constexpr auto kGoalPwm = CTI16(0x90, 0);
/** @brief 目标电流 | 单位: 0.001A | 访问: RW */
constexpr auto kGoalCurrent = CTI16(0x92, 0);
/** @brief 目标速度 | 单位: 0.229 rev/min | 访问: RW */
constexpr auto kGoalVelocity = CTI32(0x94, 0);
/** @brief 目标位置 | 单位: pulse (0-4095=0-360°) | 访问: RW */
constexpr auto kGoalPosition = CTI32(0x98, 0);
/* 0x9C-0x9F: 保留，用于目标值组扩展 */

/* 状态反馈组 (0xA0-0xBF, 32字节) */
/** @brief 实时时钟 | 单位: ms | 访问: R */
constexpr auto kRealtimeTick = CTI16(0xA0, 0);
/** @brief 运动状态 (0/1) | 单位: - | 访问: R */
constexpr auto kMoving = CTI8(0xA2, 0);
/** @brief 运动详细状态 | 单位: - | 访问: R */
constexpr auto kMovingStatus = CTI8(0xA3, 0);
/** @brief 当前PWM | 单位: 0.113% | 访问: R */
constexpr auto kPresentPwm = CTI16(0xA4, 0);
/** @brief 当前电流 | 单位: 0.001A | 访问: R */
constexpr auto kPresentCurrent = CTI16(0xA6, 0);
/** @brief 当前速度 | 单位: 0.229 rev/min | 访问: R */
constexpr auto kPresentVelocity = CTI32(0xA8, 0);
/** @brief 当前位置 | 单位: pulse (0-4095=0-360°) | 访问: R */
constexpr auto kPresentPosition = CTI32(0xAC, 0);
/** @brief 速度轨迹 | 单位: 0.229 rev/min | 访问: R */
constexpr auto kVelocityTrajectory = CTI32(0xB0, 0);
/** @brief 位置轨迹 | 单位: pulse | 访问: R */
constexpr auto kPositionTrajectory = CTI32(0xB4, 0);
/** @brief 当前输入电压 | 单位: 0.1V | 访问: R */
constexpr auto kPresentInputVoltage = CTI16(0xB8, 0);
/** @brief 当前温度 | 单位: °C | 访问: R */
constexpr auto kPresentTemperature = CTI8(0xBA, 0);
/* 0xBB-0xBF: 保留，用于状态反馈组扩展 */

constexpr static size_t kTotalSize =
    kPresentTemperature.reg.address + kPresentTemperature.reg.kSize;
};  // namespace ControlTable

namespace TableBlocks {
constexpr static ControlTableBlock kEeprom = {
    ControlTable::kModelNumber.reg.address,
    ControlTable::kProfileVelocity.reg.address +
        ControlTable::kProfileVelocity.reg.kSize};

constexpr static ControlTableBlock kRam = {
    ControlTable::kTorqueEnable.reg.address,
    ControlTable::kPresentTemperature.reg.address +
        ControlTable::kPresentTemperature.reg.kSize};

constexpr static ControlTableBlock kSetToCenter = {
    ControlTable::kSetToCenter.reg.address,
    ControlTable::kSetToCenter.reg.address +
        ControlTable::kSetToCenter.reg.kSize};
};  // namespace TableBlocks
}  // namespace hortor::servo_slave