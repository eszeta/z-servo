// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "protocol/types.h"

namespace hortor::servo_slave {

using protocol::ControlTableBlock;

constexpr uint32_t kBaudRateTable[] = {
    9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000};

//==============================================================================
// 单位转换辅助函数
//==============================================================================
namespace Converters {

// 统一使用语义化 *Converter 命名。
using VoltageCvt = regmap::RatioConverter<float, 1, 10>;       // 0.1V / LSB
using PwmPctCvt = regmap::RatioConverter<float, 113, 1000>;    // 0.113% / LSB
using VelocityCvt = regmap::RatioConverter<float, 229, 1000>;  // 0.229RPM / LSB
using MsCvt = regmap::RatioConverter<uint16_t, 20>;            // 20ms / LSB
using UsCvt = regmap::RatioConverter<uint16_t, 2>;             // 2us / LSB
using CurrentCvt = regmap::RatioConverter<float, 1, 1000>;     // 0.001A / LSB
using FeedforwardGainCvt = regmap::RatioConverter<float, 1, 4>;  // raw / 4
using PidPCvt = regmap::RatioConverter<float, 1, 128>;           // raw / 128
using PidICvt = regmap::RatioConverter<float, 1, 65536>;         // raw / 65536
using PidDCvt = regmap::RatioConverter<float, 1, 16>;            // raw / 16
}  // namespace Converters
/**
 * @brief 控制表
 * 仿照XL330的控制表
 */
struct ControlTable {
  template <uint8_t ADDRESS, uint8_t DEFAULT_VALUE>
  using RegU8 = protocol::RegU8<ADDRESS, DEFAULT_VALUE>;
  template <uint8_t ADDRESS, uint16_t DEFAULT_VALUE>
  using RegU16 = protocol::RegU16<ADDRESS, DEFAULT_VALUE>;
  template <uint8_t ADDRESS, uint32_t DEFAULT_VALUE>
  using RegU32 = protocol::RegU32<ADDRESS, DEFAULT_VALUE>;
  template <uint8_t ADDRESS, int8_t DEFAULT_VALUE>
  using RegS8 = protocol::RegS8<ADDRESS, DEFAULT_VALUE>;
  template <uint8_t ADDRESS, int16_t DEFAULT_VALUE>
  using RegS16 = protocol::RegS16<ADDRESS, DEFAULT_VALUE>;
  template <uint8_t ADDRESS, int32_t DEFAULT_VALUE>
  using RegS32 = protocol::RegS32<ADDRESS, DEFAULT_VALUE>;
  template <uint8_t ADDRESS, bool DEFAULT_VALUE>
  using RegB8 = protocol::RegB8<ADDRESS, DEFAULT_VALUE>;

  // 统一使用语义化 *Converter 命名。
  using VoltageCvt = Converters::VoltageCvt;
  using PwmPctCvt = Converters::PwmPctCvt;
  using VelocityCvt = Converters::VelocityCvt;
  using MsCvt = Converters::MsCvt;
  using UsCvt = Converters::UsCvt;
  using CurrentCvt = Converters::CurrentCvt;
  using FeedforwardGainCvt = Converters::FeedforwardGainCvt;
  using PidPCvt = Converters::PidPCvt;
  using PidICvt = Converters::PidICvt;
  using PidDCvt = Converters::PidDCvt;

  //--------------------------------------------------------------------------
  // EEPROM 区（掉电保存）
  //--------------------------------------------------------------------------

  /* 设备信息组 (0x00-0x0F, 16字节) */
  /** @brief 型号编号 | 单位: - | 访问: R */
  struct kModelNumber : RegU16<0x00, 1190> {};
  /** @brief 型号信息 | 单位: - | 访问: R */
  struct kModelInformation : RegU32<0x02, 0> {};
  /** @brief 固件版本 | 单位: - | 访问: R */
  struct kFirmwareVersion : RegU8<0x06, 0> {};
  /** @brief 舵机ID (0-252) | 单位: - | 访问: RW */
  struct kId : RegU8<0x07, 1> {};
  /* 0x08-0x0F: 保留，用于设备信息组扩展 */

  /* 通信配置组 (0x10-0x1F, 16字节) */
  /** @brief 波特率索引 (0-7) | 单位: - | 访问: RW */
  struct kBaudRate : RegU8<0x10, 3> {};
  /** @brief 返回延迟 | 单位: 2μs | 访问: RW */
  struct kReturnDelayTime : RegU8<0x11, 250> {
    using converter_t = UsCvt;
  };
  /** @brief 状态返回级别 (0-2) | 单位: - | 访问: RW */
  struct kStatusReturnLevel : RegU8<0x12, 2> {};
  /* 0x13-0x1F: 保留，用于通信配置组扩展 */

  /* 运行模式组 (0x20-0x2F, 16字节) */
  /** @brief 驱动模式 | 单位: - | 访问: RW */
  struct kDriveMode : RegU8<0x20, 0> {};
  /** @brief 工作模式 (0,1,3-5,16) | 单位: - | 访问: RW */
  struct kOperatingMode : RegU8<0x21, 3> {};
  /** @brief 关断条件 | 单位: - | 访问: RW */
  struct kShutdown : RegU8<0x22, 52> {};
  /* 0x23-0x2F: 保留，用于运行模式组扩展 */

  /* 位置配置组 (0x30-0x3F, 16字节) */
  /** @brief 归零偏移 | 单位: pulse | 访问: RW */
  struct kHomingOffset : RegS32<0x30, 0> {};
  /** @brief 运动阈值 | 单位: 0.229 rev/min | 访问: RW */
  struct kMovingThreshold : RegU32<0x34, 10> {
    using converter_t = VelocityCvt;
  };
  /* 0x38-0x3F: 保留，用于位置配置组扩展 */

  /* 保护限制组 (0x40-0x5F, 32字节) */
  /** @brief 温度上限 | 单位: °C | 访问: RW */
  struct kTemperatureLimit : RegU8<0x40, 72> {};
  /** @brief 最高电压限制 | 单位: 0.1V | 访问: RW */
  struct kMaxVoltageLimit : RegU16<0x41, 160> {
    using converter_t = VoltageCvt;
  };
  /** @brief 最低电压限制 | 单位: 0.1V | 访问: RW */
  struct kMinVoltageLimit : RegU16<0x43, 50> {
    using converter_t = VoltageCvt;
  };
  /** @brief PWM上限 | 单位: 0.113% | 访问: RW */
  struct kPwmLimit : RegU16<0x45, 885> {
    using converter_t = PwmPctCvt;
  };
  /** @brief 电流上限 | 单位: 0.001A | 访问: RW */
  struct kCurrentLimit : RegU16<0x47, 3000> {
    using converter_t = CurrentCvt;
  };
  /** @brief 速度上限 | 单位: 0.229 rev/min | 访问: RW */
  struct kVelocityLimit : RegU32<0x49, 2047> {
    using converter_t = VelocityCvt;
  };
  /** @brief 位置上限 | 单位: pulse | 访问: RW */
  struct kMaxPositionLimit : RegU32<0x4D, 4095> {};
  /** @brief 位置下限 | 单位: pulse | 访问: RW */
  struct kMinPositionLimit : RegU32<0x51, 0> {};
  /** @brief 过载保护时间 | 单位: 20ms | 访问: RW */
  struct kProtectionTime : RegU8<0x55, 0> {
    using converter_t = MsCvt;
  };
  /* 0x56-0x5F: 保留，用于保护限制组扩展 */

  /* PID 参数组 (0x60-0x6F, 16字节) */
  /** @brief 速度积分增益 | 单位: - | 访问: RW */
  struct kVelocityIGain : RegU16<0x60, 1920> {
    using converter_t = PidICvt;
  };
  /** @brief 速度比例增益 | 单位: - | 访问: RW */
  struct kVelocityPGain : RegU16<0x62, 100> {
    using converter_t = PidPCvt;
  };
  /** @brief 位置微分增益 | 单位: - | 访问: RW */
  struct kPositionDGain : RegU16<0x64, 4000> {
    using converter_t = PidDCvt;
  };
  /** @brief 位置积分增益 | 单位: - | 访问: RW */
  struct kPositionIGain : RegU16<0x66, 0> {
    using converter_t = PidICvt;
  };
  /** @brief 位置比例增益 | 单位: - | 访问: RW */
  struct kPositionPGain : RegU16<0x68, 800> {
    using converter_t = PidPCvt;
  };
  /** @brief 前馈二阶增益 | 单位: - | 访问: RW */
  struct kFeedforward2ndGain : RegU16<0x6A, 0> {
    using converter_t = FeedforwardGainCvt;
  };
  /** @brief 前馈一阶增益 | 单位: - | 访问: RW */
  struct kFeedforward1stGain : RegU16<0x6C, 0> {
    using converter_t = FeedforwardGainCvt;
  };
  /* 0x6E-0x6F: 保留，用于PID参数组扩展 */

  /* 0x70-0x7F: 保留 */

  //--------------------------------------------------------------------------
  // RAM 区（掉电不保存）
  //--------------------------------------------------------------------------
  /* 控制命令组 (0x80-0x8F, 16字节) */
  /** @brief 力矩使能 (0/1) | 单位: - | 访问: RW */
  struct kTorqueEnable : RegU8<0x80, 0> {};
  /** @brief LED开关 (0/1) | 单位: - | 访问: RW */
  struct kDxlLed : RegU8<0x81, 0> {};
  /** @brief 对齐到目标位置 | 单位: pulse | 访问: W */
  struct kAlignToPosition : RegU16<0x82, 0> {};
  /** @brief 硬件错误状态 | 单位: - | 访问: R */
  struct kHardwareErrorStatus : RegU8<0x84, 0> {};
  /** @brief 总线看门狗 | 单位: 20ms | 访问: RW */
  struct kBusWatchdog : RegU8<0x85, 0> {
    using converter_t = MsCvt;
  };
  /* 0x86-0x8F: 保留，用于控制命令组扩展 */

  /* 目标值组 (0x90-0x9F, 16字节) */
  /** @brief 目标PWM | 单位: 0.113% | 访问: RW */
  struct kGoalPwm : RegS16<0x90, 0> {
    using converter_t = PwmPctCvt;
  };
  /** @brief 目标电流 | 单位: 0.001A | 访问: RW */
  struct kGoalCurrent : RegU16<0x92, 0> {
    using converter_t = CurrentCvt;
  };
  /** @brief 目标速度 | 单位: 0.229 rev/min | 访问: RW */
  struct kGoalVelocity : RegU32<0x94, 0> {
    using converter_t = VelocityCvt;
  };
  /** @brief 目标位置 | 单位: pulse (0-4095=0-360°) | 访问: RW */
  struct kGoalPosition : RegS32<0x98, 0> {};
  /* 0x9C-0x9F: 保留，用于目标值组扩展 */

  /* 状态反馈组 (0xA0-0xBF, 32字节) */
  /** @brief 实时时钟 | 单位: ms | 访问: R */
  struct kRealtimeTick : RegU16<0xA0, 0> {};
  /** @brief 运动状态 (0/1) | 单位: - | 访问: R */
  struct kMoving : RegB8<0xA2, 0> {};
  /** @brief 运动详细状态 | 单位: - | 访问: R */
  struct kMovingStatus : RegU8<0xA3, 0> {};
  /** @brief 当前PWM | 单位: 0.113% | 访问: R */
  struct kPresentPwm : RegS16<0xA4, 0> {
    using converter_t = PwmPctCvt;
  };
  /** @brief 当前电流 | 单位: 0.001A | 访问: R */
  struct kPresentCurrent : RegU16<0xA6, 0> {
    using converter_t = CurrentCvt;
  };
  /** @brief 当前速度 | 单位: 0.229 rev/min | 访问: R */
  struct kPresentVelocity : RegU32<0xA8, 0> {
    using converter_t = VelocityCvt;
  };
  /** @brief 当前位置 | 单位: pulse (0-4095=0-360°) | 访问: R */
  struct kPresentPosition : RegS32<0xAC, 0> {};
  /** @brief 速度轨迹 | 单位: 0.229 rev/min | 访问: R */
  struct kVelocityTrajectory : RegU32<0xB0, 0> {
    using converter_t = VelocityCvt;
  };
  /** @brief 位置轨迹 | 单位: pulse | 访问: R */
  struct kPositionTrajectory : RegS32<0xB4, 0> {};
  /** @brief 当前输入电压 | 单位: 0.1V | 访问: R */
  struct kPresentInputVoltage : RegU16<0xB8, 0> {
    using converter_t = VoltageCvt;
  };
  /** @brief 当前温度 | 单位: °C | 访问: R */
  struct kPresentTemperature : RegU8<0xBA, 0> {};
  /* 0xBB-0xBF: 保留，用于状态反馈组扩展 */

  static constexpr size_t kTotalSize = 0xC0;
};  // struct ControlTable

namespace TableBlocks {
constexpr ControlTableBlock kEeprom = {0, ControlTable::kTotalSize};

constexpr ControlTableBlock kRam = {
    ControlTable::kTorqueEnable::kAddress,
    ControlTable::kPresentTemperature::kAddress +
        ControlTable::kPresentTemperature::kSize};

constexpr ControlTableBlock kAlign = {
    ControlTable::kAlignToPosition::kAddress,
    ControlTable::kAlignToPosition::kAddress +
        ControlTable::kAlignToPosition::kSize};
}  // namespace TableBlocks
}  // namespace hortor::servo_slave