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

#include "core/types.h"
#include "protocol/types.h"
#include "register/register_field.h"

namespace hortor_servo {

static constexpr uint32_t kBaudrateTable[] = {
    2000000, 1000000, 500000, 250000, 115200, 57600, 38400, 9600};
struct ControlTable {
  /*-------------------- EEPROM 区（掉电保存） --------------------*/
  CTI_16(kModelNumber, 0, 360);       // 型号固件写死 360（0x0168）
  CTI_08(kFirmwareVersion, 2, 0);     // 固件版本（只读，出厂烧录）
  CTI_08(kId, 3, 1);                  // 总线 ID（1 为出厂值，范围 0-252）
  CTI_08(kBaudRate, 4, 1);            // 波特率
  CTI_08(kReturnDelayTime, 5, 250);   // 返回延时 250×2 µs = 500 µs
  CTI_16(kCwAngleLimit, 6, 0);        // 顺时针角度限位（0°）
  CTI_16(kCcwAngleLimit, 8, 4095);    // 逆时针角度限位（360° → 4095）
  CTI_08(kTemperatureLimit, 11, 70);  // 温度报警阈值 70 ℃
  CTI_08(kMinVoltageLimit, 12, 60);   // 最低输入电压 6.0 V（60×0.1 V）
  CTI_08(kMaxVoltageLimit, 13, 160);  // 最高输入电压 16.0 V
  CTI_16(kMaxTorque, 14, 1023);       // 最大扭矩 1023 ≙ 100 %
  CTI_08(kStatusReturnLevel, 16, 2);  // 状态包返回等级 2（所有指令都返回）
  CTI_08(kAlarmLed, 17, 36);          // 报警时 LED 闪烁掩码 36（0x24）
  CTI_08(kShutdown, 18, 36);          // 报警后关闸掩码 36（同 LED）
  CTI_16(kMultiTurnOffset, 20, 0);    // 多圈偏移 0（仅多圈模式有效）
  CTI_08(kResolutionDivider, 22, 1);  // 分辨率分频 1（1-4 有效）

  /*-------------------- RAM 区（掉电丢失） --------------------*/
  CTI_08(kTorqueEnable, 24, 0);        // 扭矩开关 0=关闭 1=开启
  CTI_08(kDxlLed, 25, 0);              // 用户 LED 0=灭 1=亮
  CTI_08(kPosDgain, 26, 8);            // PID 微分增益 D
  CTI_08(kPosIgain, 27, 0);            // PID 积分增益 I
  CTI_08(kPosPgain, 28, 8);            // PID 比例增益 P
  CTI_16(kGoalPosition, 30, 0);        // 目标位置 0-4095（0.088°/步）
  CTI_16(kMovingSpeed, 32, 0);         // 移动速度 0=最大速度
  CTI_16(kTorqueLimit, 34, 1023);      // 实时扭矩限制 1023≙100 %
  CTI_16(kPresentPosition, 36, 0);     // 当前位置（只读）
  CTI_16(kPresentSpeed, 38, 0);        // 当前速度（只读）
  CTI_16(kPresentLoad, 40, 0);         // 当前负载（只读，0-1023）
  CTI_08(kPresentVoltage, 42, 0);      // 当前电压（只读，值×0.1 V）
  CTI_08(kPresentTemperature, 43, 0);  // 当前温度（只读，单位℃）
  CTI_08(kRegistered, 44, 0);          // REG_WRITE 指令注册标志
  CTI_08(kMoving, 46, 0);              // 是否正在运动 0=静止 1=运动中
  CTI_08(kLock, 47, 0);                // EEPROM 锁 0=解锁 1=锁定
  CTI_16(kPunch, 48, 32);              // 启动电流阈值 32（0x20）
  CTI_16(kRealtimeTick, 50, 0);        // 实时时钟 ms 计数（只读）
  CTI_08(kGoalAcceleration, 73, 0);    // 目标加速度 0=无限制
};  // struct ControlTable

struct TableBlocks {
  constexpr static ControlTableBlock kEeprom = {
      ControlTable::kModelNumber.reg.address,
      ControlTable::kResolutionDivider.reg.address +
          ControlTable::kResolutionDivider.reg.getSize()};

  constexpr static ControlTableBlock kRam = {
      ControlTable::kTorqueEnable.reg.address,
      ControlTable::kGoalAcceleration.reg.address +
          ControlTable::kGoalAcceleration.reg.getSize()};

  constexpr static ControlTableBlock kTotal = {
      ControlTable::kModelNumber.reg.address,
      ControlTable::kGoalAcceleration.reg.address +
          ControlTable::kGoalAcceleration.reg.getSize()};
};
}  // namespace hortor_servo