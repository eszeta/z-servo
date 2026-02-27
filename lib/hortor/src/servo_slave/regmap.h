// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file regmap.h
 * @brief 伺服从机寄存器映射
 *
 * 参考: DYNAMIXEL XL330 控制表
 * https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/
 */

#pragma once

#include <Arduino.h>

#include "protocol/regmap.h"
#include "regmap/regmap_mmio.h"
#include "types.h"

#ifndef EEPROM_DISABLE
#include <EEPROM.h>
#endif

namespace hortor::servo_slave {

//==============================================================================
// 单位转换辅助函数
//==============================================================================
namespace {

/**
 * @brief 将原始电压值转换为实际电压值
 * @param raw 原始值
 * @return 实际电压值
 * 原始单位：0.1V，转换单位：V
 * 转换公式: voltage = raw × 0.1V
 */
constexpr float VoltageFromRaw(uint16_t raw) { return raw * 0.1f; }

/** @brief 将实际电压值转换为原始值 */
constexpr uint16_t VoltageToRaw(float voltage) {
  return static_cast<uint16_t>(voltage * 10.0f);
}

/**
 * @brief 将原始 PWM 值转换为实际 PWM 百分比
 * @param raw 原始值
 * @return 实际 PWM 百分比 (-100.0% 到 100.0%)
 * 原始单位：0.113%，转换单位：%
 * 转换公式: percent = raw × 0.113%
 */
constexpr float PwmFromRaw(int16_t raw) { return raw * 0.113f; }

/** @brief 将实际 PWM 百分比转换为原始值 */
constexpr int16_t PwmToRaw(float percent) {
  return static_cast<int16_t>(percent / 0.113f);
}

/**
 * @brief 将原始速度值转换为实际速度值
 * @param raw 原始值 
 * @return 实际速度值
 *
 * 转换公式: rpm = raw × 0.229 RPM
 * 原始单位：0.229 RPM，转换单位：RPM
 * @note 负值表示反向旋转
 */
constexpr float VelocityFromRaw(uint32_t raw) { return raw * 0.229f; }

/** @brief 将实际速度值转换为原始值 */
constexpr uint32_t VelocityToRaw(float rpm) {
  return static_cast<uint32_t>(rpm / 0.229f);
}

/**
 * @brief 将原始加速度值转换为实际加速度值
 * @param raw 原始值
 * @return 实际加速度值
 * 原始单位：214.577 rev/min²，转换单位：rev/min²
 * 转换公式: acc = raw × 214.577 rev/min²
 */
constexpr float AccelerationFromRaw(uint32_t raw) { return raw * 214.577f; }

/** @brief 将实际加速度值转换为原始值 */
constexpr uint32_t AccelerationToRaw(float acc) {
  return static_cast<uint32_t>(acc / 214.577f);
}

/**
 * @brief 将原始ms转换为实际秒
 * @param raw 原始值
 * @return 实际秒
 * 原始单位：20ms，转换单位：s
 * 转换公式: seconds = raw × 0.02  
 */
constexpr float MsFromRaw(uint8_t raw) { return raw * 0.02f; }

/** @brief 将实际秒转换为原始值 */
constexpr uint8_t MsToRaw(float seconds) {
  return static_cast<uint8_t>(seconds / 0.02f);
}

/**
 * @brief 将原始 P 增益值转换为实际增益值
 * @param raw 原始值
 * @return 实际 P 增益值
 * 原始单位：128.0，转换单位：-
 * 转换公式: gain = raw / 128.0
 */
constexpr float PidPGainFromRaw(uint16_t raw) { return raw / 128.0f; }

/** @brief 将实际 P 增益值转换为原始值 */
constexpr uint16_t PidPGainToRaw(float gain) {
  return static_cast<uint16_t>(gain * 128.0f);
}

/**
 * @brief 将原始 I 增益值转换为实际增益值
 * @param raw 原始值
 * @return 实际 I 增益值
 * 原始单位：65536.0，转换单位：-
 * 转换公式: gain = raw / 65536.0
 */
constexpr float PidIGainFromRaw(uint16_t raw) { return raw / 65536.0f; }

/** @brief 将实际 I 增益值转换为原始值  */
constexpr uint16_t PidIGainToRaw(float gain) {
  return static_cast<uint16_t>(gain * 65536.0f);
}

/**
 * @brief 将原始 D 增益值转换为实际增益值
 * @param raw 原始值
 * @return 实际 D 增益值
 * 原始单位：16.0，转换单位：-
 * 转换公式: gain = raw / 16.0
 */
constexpr float PidDGainFromRaw(uint16_t raw) { return raw / 16.0f; }

/** @brief 将实际 D 增益值转换为原始值 */
constexpr uint16_t PidDGainToRaw(float gain) {
  return static_cast<uint16_t>(gain * 16.0f);
}

/**
 * @brief 将原始前馈增益值转换为实际增益值
 * @param raw 原始值
 * @return 实际前馈增益值
 * 原始单位：4.0，转换单位：-
 * 转换公式: gain = raw / 4.0
 */
constexpr float FeedforwardGainFromRaw(uint16_t raw) { return raw / 4.0f; }

/** @brief 将实际前馈增益值转换为原始值 */
constexpr uint16_t FeedforwardGainToRaw(float gain) {
  return static_cast<uint16_t>(gain * 4.0f);
}

/**
 * @brief 将原始电流值转换为实际电流值
 * @param raw 原始值 
 * @return 实际电流值 
 * 原始单位：0.001A，转换单位：A
 * 转换公式: current = raw × 0.001A
 */
constexpr float CurrentFromRaw(uint16_t raw) { return raw / 1000.0f; }

/** @brief 将实际电流值转换为原始值 */
constexpr uint16_t CurrentToRaw(float current) {
  return static_cast<uint16_t>(current * 1000.0f);
}

}  // namespace

/**
 * @brief 伺服从机寄存器映射实现（CRTP模式）
 *
 * 继承自 protocol::RegMap，使用 MMIO 总线实现。
 */
class RegMap;
using RegMapBase = protocol::RegMap<RegMap, regmap::RegMapMmio>;
class RegMap : public RegMapBase {
 public:
  Error Init() {
    CHECK(regmap::RegMapMmio::Init(table_, sizeof(table_)));
    CHECK(LoadEeprom());
    if (IsEepromEmpty()) {
      CHECK(RecoveryEeprom());
    }
    return Error::kOk;
  }

  template <typename T>
  constexpr Error RestoreEeprom(
      const hortor::protocol::ControlTableItem<T>& item) {
    CHECK(WriteRegField(item, item.default_value));
    return Error::kOk;
  }

  //==============================================================================
  // 设备信息组 (0x00-0x0F, EEPROM，只读)
  //==============================================================================
#pragma region "设备信息组"
  // 此区域包含设备识别和版本信息，所有寄存器均为只读，出厂预设。

  /**
   * @brief 获取型号编号 (R)
   * @return model_number 型号编号
   */
  uint16_t ReadModelNumber() {
    uint16_t model_number;
    ReadRegField(ControlTable::kModelNumber, model_number);
    return model_number;
  }

  /**
   * @brief 设置型号编号 (R)
   * @param model_number 型号编号
   */
  void WriteModelNumber(const uint16_t model_number) {
    WriteRegField(ControlTable::kModelNumber, model_number);
  }

  /**
   * @brief 获取型号信息 (R)
   * @return model_information 型号信息
   */
  uint32_t ReadModelInformation() {
    uint32_t model_information;
    ReadRegField(ControlTable::kModelInformation, model_information);
    return model_information;
  }

  /**
   * @brief 设置型号信息 (R)
   * @param model_information 型号信息
   */
  void WriteModelInformation(const uint32_t model_information) {
    WriteRegField(ControlTable::kModelInformation, model_information);
  }

  /**
   * @brief 获取固件版本 (R)
   * @return firmware_version 固件版本
   */
  uint8_t ReadFirmwareVersion() {
    uint8_t firmware_version;
    ReadRegField(ControlTable::kFirmwareVersion, firmware_version);
    return firmware_version;
  }

  /**
   * @brief 设置固件版本 (R)
   * @param firmware_version 固件版本
   */
  void WriteFirmwareVersion(const uint8_t firmware_version) {
    WriteRegField(ControlTable::kFirmwareVersion, firmware_version);
  }

  /**
   * @brief 获取舵机 ID (R/W)
   * @return id 舵机 ID
   *
   * 范围: 0-252
   *
   * 【功能说明】
   * - 舵机在总线上的唯一标识符
   * - 用于多舵机系统中的地址识别
   * - 同一总线上不能有重复的 ID
   */
  uint8_t ReadId() {
    uint8_t id;
    ReadRegField(ControlTable::kId, id);
    return id;
  }

  /**
   * @brief 设置舵机 ID (R/W)
   * @param[in] id 舵机 ID (0-252)
   */
  void WriteId(const uint8_t id) { WriteRegField(ControlTable::kId, id); }

#pragma endregion  // "设备信息组"

  //==============================================================================
  // 通信配置组 (0x10-0x1F, EEPROM)
  //==============================================================================
#pragma region "通信配置组"

  /**
   * @brief 获取波特率索引 (R/W)
   * @return baud_rate 波特率索引
   *
   * 范围: 0-7
   *
   * 【功能说明】
   * - 设置串口通信的波特率
   * - 通过索引值选择预定义的波特率
   * - 修改后需要重启生效
   *
   * 【波特率对照表】
   * | 索引 | 波特率 (bps) |
   * |------|--------------|
   * | 0    | 9600         |
   * | 1    | 57600        |
   * | 2    | 115200       |
   * | 3    | 1000000      |
   * | 4    | 2000000      |
   * | 5    | 3000000      |
   * | 6    | 4000000      |
   * | 7    | 4500000      |
   *
   * 【相关寄存器】
   * - Return Delay Time: 返回延迟设置
   */
  uint8_t ReadBaudRate() {
    uint8_t baud_rate;
    ReadRegField(ControlTable::kBaudRate, baud_rate);
    return baud_rate;
  }

  /**
   * @brief 设置波特率索引 (R/W)
   * @param[in] baud_rate 波特率索引 (0-7)
   */
  void WriteBaudRate(const uint8_t baud_rate) {
    WriteRegField(ControlTable::kBaudRate, baud_rate);
  }

  /**
   * @brief 获取返回延迟时间 (R/W)
   * @return microseconds 返回延迟时间（μs）
   *
   * 范围: 0-508 μs
   *
   * 【功能说明】
   * - 设置舵机响应指令后的延迟时间
   * - 用于多舵机串联时避免总线冲突
   * - 延迟时间 = 原始值 × 2μs
   * - 修改后需要重启生效
   *
   * 【典型参数】
   * - 无延迟：0 μs（最快响应）
   * - 默认延迟：500 μs（平衡性能）
   * - 最大延迟：508 μs（最稳定）
   *
   * @note 在多舵机串联时，适当的延迟可避免总线冲突
   * @note 修改后需调用 StoreEeprom() 并重启生效
   */
  uint16_t ReadReturnDelayTime() {
    uint8_t raw;
    ReadRegField(ControlTable::kReturnDelayTime, raw);
    return raw * 2;
  }

  /**
   * @brief 设置返回延迟时间 (R/W)
   * @param[in] microseconds 返回延迟时间（μs）
   *
   * @note 在多舵机串联时，适当的延迟可避免总线冲突
   */
  void WriteReturnDelayTime(const uint16_t microseconds) {
    WriteRegField(ControlTable::kReturnDelayTime, microseconds / 2);
  }

  /**
   * @brief 获取状态返回级别 (R/W)
   * @return status_return_level 状态返回级别
   *
   * 范围: 0-2
   *
   * 【功能说明】
   * - 设置舵机返回状态包的级别
   * - 控制通信流量和响应行为
   * - 影响总线通信效率
   *
   * 【级别说明】
   * - 0: 不返回状态包（仅 PING 指令返回）
   * - 1: 仅读取指令返回状态包
   * - 2: 所有指令都返回状态包（默认）
   *
   */
  uint8_t ReadStatusReturnLevel() {
    uint8_t status_return_level = 0;
    ReadRegField(ControlTable::kStatusReturnLevel, status_return_level);
    return status_return_level;
  }

  /**
   * @brief 设置状态返回级别 (R/W)
   * @param[in] status_return_level 状态返回级别 (0-2)
   */
  void WriteStatusReturnLevel(const uint8_t status_return_level) {
    WriteRegField(ControlTable::kStatusReturnLevel, status_return_level);
  }

#pragma endregion  // "通信配置组"

  //==============================================================================
  // 运行模式组 (0x20-0x2F, EEPROM)
  //==============================================================================
#pragma region "运行模式组"

  /**
   * @brief 获取驱动模式 (R/W)
   * @return drive_mode 驱动模式位域
   *
   * 【功能说明】
   * - 配置舵机的基本驱动行为
   * - 位域寄存器，每个 bit 控制不同功能
   * - 修改后需要重启生效
   *
   * 【位域定义】
   * | Bit | 名称                       | 说明                              |
   * |-----|----------------------------|-----------------------------------|
   * | 0   | Motor Reverse Mode         | 0: Normal, 1: Reverse             |
   * | 1   | Encoder Reverse Mode       | 0: Normal, 1: Reverse             |
   * | 2   | Profile Configuration      | 0: Velocity-based, 1: Time-based  |
   * | 3   | Reserved                   | 保留（未使用）                    |
   * | 4   | Reserved                   | 保留（未使用）                    |
   * | 5   | Reserved                   | 保留（未使用）                    |
   * | 6   | Reserved                   | 保留（未使用）                    |
   * | 7   | Reserved                   | 保留（未使用）                    |
   *
   * 【Profile 配置 (Bit 2)】
   * - 0: Velocity-based Profile - 使用速度和加速度参数
   * - 1: Time-based Profile - 使用时间参数（毫秒）
   * 
   * 【电机反转模式 (Bit 4)】
   * - 电机正反转模式，修正最终输出轴方向不一致问题
   *
   * 【编码器反转模式 (Bit 5)】
   * - 编码器正反转模式，修正最终输出轴方向不一致问题
   *
   * 【相关寄存器】
   * - Profile Velocity: Velocity-based 模式使用
   * - Profile Acceleration: 两种模式都使用（Time-based 模式为加速时间）
   * - Operating Mode: 工作模式设置
   */
  uint8_t ReadDriveMode() {
    uint8_t drive_mode;
    ReadRegField(ControlTable::kDriveMode, drive_mode);
    return drive_mode;
  }

  /**
   * @brief 设置驱动模式 (R/W)
   * @param[in] drive_mode 驱动模式位域
   */
  void WriteDriveMode(const uint8_t drive_mode) {
    WriteRegField(ControlTable::kDriveMode, drive_mode);
  }

  /**
   * @brief 获取工作模式 (R/W)
   * @return operating_mode 工作模式
   *
   * 【功能说明】
   * - 设置舵机的工作模式，决定控制目标和行为特性
   * - 不同模式使用不同的目标寄存器和控制策略
   * - 修改前必须先禁用力矩（Torque Enable = 0）
   * - 修改后需要重启生效
   *
   * 【模式对比表】
   * | 模式             | 控制目标      | Profile     | 位置限位  | 多圈支持| 典型应用       |
   * |------------------|---------------|-------------|-----------|---------|----------------|
   * | 0: Current       | 电流/力矩     | ✗           | ✗         | ✗       | 力控、柔顺     |
   * | 1: Velocity      | 速度          | ~ (仅加速度)| ✗         | ✓       | 轮子、传送带   |
   * | 3: Position      | 位置(单圈)    | ✓           | ✓         | ✗       | 关节、单圈定位 |
   * | 4: Extended      | 位置(多圈)    | ✓           | ✗         | ✓       | 多圈、旋转计数 |
   * | 5: Current-based | 位置+电流限制 | ✓           | ✓         | ✗       | 抓取、防撞     |
   * | 16: PWM          | PWM占空比     | ✗           | ✗         | ✗       | 开环、调试     |
   *
   * 【详细模式说明】
   *
   * 【0: Current Control Mode（电流控制模式）】
   * - 使用 Goal Current 控制输出电流
   * - 输出力矩与电流成正比
   * - 不控制位置和速度
   * - 适用场景：力矩控制、柔顺控制
   *
   * 【1: Velocity Control Mode（速度控制模式）】
   * - 使用 Goal Velocity 控制转速
   * - Profile Acceleration 用于加速度控制
   * - 不控制位置
   * - 适用场景：轮式机器人、传送带
   *
   * 【3: Position Control Mode（位置控制模式，默认）】
   * - 使用 Goal Position 控制位置
   * - 位置范围: 0-4095 (0-360°)
   * - 受 Min/Max Position Limit 限制
   * - Profile Velocity 和 Profile Acceleration 控制运动轨迹
   * - 适用场景：关节控制、单圈定位
   *
   * 【4: Extended Position Control Mode（扩展位置控制模式）】
   * - 支持多圈位置控制
   * - 位置范围: -1,048,575 ~ 1,048,575 (-256 ~ 256 圈)
   * - 不受 Min/Max Position Limit 限制
   * - 支持无限旋转累计
   * - 适用场景：多圈定位、旋转计数
   *
   * 【5: Current-based Position Control Mode（电流型位置控制）】
   * - 位置控制 + 电流限制
   * - Goal Current 作为最大电流限制
   * - 遇到阻力时电流不超过设定值
   * - 适用场景：柔性抓取、防撞控制
   *
   * 【16: PWM Control Mode（PWM 控制模式）】
   * - 直接控制 PWM 输出占空比
   * - 使用 Goal PWM 设置占空比
   * - 不进行位置/速度/电流控制
   * - 适用场景：开环控制、调试测试
   *
   * 【模式切换流程】
   * 1. WriteTorqueEnable(0) - 必须先关闭力矩
   * 2. WriteOperatingMode(new_mode) - 设置新模式
   * 3. 设置该模式的目标值（Goal Position/Velocity/Current/PWM）
   * 4. WriteTorqueEnable(1) - 重新使能力矩
   *
   * 【相关寄存器】
   * - Goal Position: 模式 3,4,5 使用
   * - Goal Velocity: 模式 1 使用
   * - Goal Current: 模式 0,5 使用
   * - Goal PWM: 模式 16 使用
   * - Torque Enable: 切换模式前必须禁用
   *
   * @warning 更改工作模式时，必须先将 Torque Enable 设置为 0
   * @note 不同模式下使用的控制寄存器不同，详见各 Goal 寄存器说明
   */
  uint8_t ReadOperatingMode() {
    uint8_t operating_mode = 0;
    ReadRegField(ControlTable::kOperatingMode, operating_mode);
    return operating_mode;
  }

  /**
   * @brief 设置工作模式 (R/W)
   * @param[in] operating_mode 工作模式
   */
  void WriteOperatingMode(const uint8_t operating_mode) {
    WriteRegField(ControlTable::kOperatingMode, operating_mode);
  }

  /**
   * @brief 获取关断条件 (R/W)
   * @return shutdown 关断条件位域
   *
   * 范围: 0-255 (8位位域)
   *
   * 【功能说明】
   * - 设置舵机自动关断的条件
   * - 当对应错误发生时，舵机会自动关断力矩（Torque Enable = 0）
   * - 错误状态会记录在 Hardware Error Status
   * - 修改后需要重启生效
   *
   * 【位域定义】
   * | Bit | 错误类型               | 说明                          |
   * |-----|------------------------|-------------------------------|
   * | 0   | Input Voltage Error    | 输入电压超出范围              |
   * | 1   | Overheating Error      | 温度超过上限                  |
   * | 2   | Motor Encoder Error    | 编码器故障                    |
   * | 3   | Electrical Shock Error | 电气冲击                      |
   * | 4   | Overload Error         | 过载                          |
   *
   * 【相关寄存器】
   * - Hardware Error Status: 错误状态记录
   * - Torque Enable: 关断后需要重新使能
   * - Temperature Limit: 温度保护阈值
   * - Voltage Limits: 电压保护阈值
   */
  uint8_t ReadShutdown() {
    uint8_t shutdown = 0;
    ReadRegField(ControlTable::kShutdown, shutdown);
    return shutdown;
  }

  /**
   * @brief 设置关断条件 (R/W)
   * @param[in] shutdown 关断条件位域
   */
  void WriteShutdown(const uint8_t shutdown) {
    WriteRegField(ControlTable::kShutdown, shutdown);
  }

#pragma endregion  // "运行模式组"

  //==============================================================================
  // 位置配置组 (0x30-0x3F, EEPROM)
  //==============================================================================
#pragma region "位置配置组"

  /**
   * @brief 获取归零偏移 (R/W)
   * @param[out] homing_offset 归零偏移（pulse）
   *
   * 范围: -1,048,575 ~ 1,048,575 pulse
   *
   * 【功能说明】
   * - 归零偏移用于定义舵机的零位参考点
   * - 影响所有位置指令的计算
   * - 实际目标位置 = Goal Position
   * - 修改后需要重启生效
   *
   * 【使用场景】
   * - 机械校准：补偿机械安装误差
   * - 零点调整：设置用户自定义的零位
   * - 多圈校准：Extended Position Control Mode 下的零点设置
   *
   * 【典型参数】
   * - 无偏移：0（默认）
   * - 小调整：±100 ~ ±1000 pulse（±2.2° ~ ±22°）
   * - 大调整：±10000 ~ ±50000 pulse（±220° ~ ±1100°）
   *
   * 【相关寄存器】
   * - Goal Position: 目标位置会加上此偏移
   * - Present Position: 当前位置会加上此偏移
   * - Min/Max Position Limit: 限位范围不受影响
   * - Operating Mode: 不同模式下的行为差异
   */
  int32_t ReadHomingOffset() {
    uint32_t raw;
    ReadRegField(ControlTable::kHomingOffset, raw);
    return static_cast<int32_t>(raw);
  }

  /**
   * @brief 设置归零偏移 (R/W)
   * @param[in] homing_offset 归零偏移（pulse）
   */
  void WriteHomingOffset(const int32_t homing_offset) {
    WriteRegField(ControlTable::kHomingOffset,
                  static_cast<uint32_t>(homing_offset));
  }

  /**
   * @brief 获取运动阈值 (R/W)
   * @param[out] rpm 运动阈值（RPM）
   *
   * 【功能说明】
   * - 运动阈值用于判断舵机是否在运动
   * - 影响 Moving Status 的更新
   * - 当速度 ≥ 此阈值时，Moving 置 1
   * - 当速度 < 此阈值时，Moving 清 0
   * - 修改后需要重启生效
   *
   * 【典型参数】
   * - 高精度应用: 0.5-1.0 RPM（避免微小振动）
   * - 一般应用: 2-5 RPM（平衡精度和稳定性）
   * - 高速应用: 10-20 RPM（避免高速时的误判）
   *
   * 【相关寄存器】
   * - Moving Status: 基于此阈值更新
   * - Present Velocity: 用于比较的实际速度
   * - Moving: 简化的运动状态指示
   *
   * @note 用于检测运动状态，避免微小振动导致的误判
   */
  float ReadMovingThreshold() {
    uint32_t raw;
    ReadRegField(ControlTable::kMovingThreshold, raw);
    return VelocityFromRaw(raw);
  }

  /**
   * @brief 设置运动阈值 (R/W)
   * @param[in] rpm 运动阈值（RPM）
   */
  void WriteMovingThreshold(const float rpm) {
    WriteRegField(ControlTable::kMovingThreshold, VelocityToRaw(rpm));
  }

#pragma endregion  // "位置配置组"

  //==============================================================================
  // 保护限制组 (0x40-0x5F, EEPROM)
  //==============================================================================
#pragma region "保护限制组"
  /**
   * @brief 获取温度上限 (R/W)
   * @param[out] temperature_limit 温度上限（°C）
   *
   * 范围: 0-100°C
   *
   * 【功能说明】
   * - 设置舵机内部温度保护上限
   * - 当内部温度超过此值时触发保护机制
   * - Hardware Error Status 的 Overheating Error bit 置 1
   * - 如果 Shutdown 对应位启用，舵机会自动关断力矩
   * - 修改后需要重启生效
   *
   * 【典型参数】
   * - 保守设置: 60-70°C（高安全裕度）
   * - 标准设置: 70-80°C（平衡性能和安全性）
   * - 极限设置: 80-90°C（最大性能，需谨慎）
   *
   * 【相关寄存器】
   * - Present Temperature: 当前温度监控
   * - Shutdown: 关断条件设置
   * - Hardware Error Status: 错误状态
   */
  uint8_t ReadTemperatureLimit() {
    uint8_t temperature_limit;
    ReadRegField(ControlTable::kTemperatureLimit, temperature_limit);
    return temperature_limit;
  }

  /**
   * @brief 设置温度上限 (R/W)
   * @param[in] temperature_limit 温度上限（°C）
   */
  void WriteTemperatureLimit(const uint8_t temperature_limit) {
    WriteRegField(ControlTable::kTemperatureLimit, temperature_limit);
  }

  /**
   * @brief 获取最高电压限制 (R/W)
   * @return voltage 最高电压限制（V）
   *
   * 范围: 5.0-16.0V
   *
   * 【功能说明】
   * - 设置输入电压保护上限
   * - 当输入电压超过此值时触发保护机制
   * - Hardware Error Status 的 Input Voltage Error bit 置 1
   * - 如果 Shutdown 对应位启用，舵机会自动关断力矩
   * - 修改后需要重启生效
   *
   * 【典型参数】
   * - 12V 系统: 13-14V（标准12V电源）
   * - 24V 系统: 26-28V（标准24V电源）
   * - 通用设置: 15-16V（兼容多种电源）
   *
   * 【相关寄存器】
   * - Present Input Voltage: 当前电压监控
   * - Min Voltage Limit: 最低电压限制
   * - Shutdown: 关断条件设置
   */
  float ReadMaxVoltageLimit() {
    uint16_t raw;
    ReadRegField(ControlTable::kMaxVoltageLimit, raw);
    return VoltageFromRaw(raw);
  }

  /**
   * @brief 设置最高电压限制 (R/W)
   * @param[in] voltage 最高电压限制（V）
   */
  void WriteMaxVoltageLimit(const float voltage) {
    WriteRegField(ControlTable::kMaxVoltageLimit, VoltageToRaw(voltage));
  }

  /**
   * @brief 获取最低电压限制 (R/W)
   * @return voltage 最低电压限制（V）
   *
   * 范围: 5.0-16.0V
   *
   * 【功能说明】
   * - 设置输入电压保护下限
   * - 当输入电压低于此值时触发保护机制
   * - Hardware Error Status 的 Input Voltage Error bit 置 1
   * - 如果 Shutdown 对应位启用，舵机会自动关断力矩
   * - 修改后需要重启生效
   *
   * 【典型参数】
   * - 12V 系统: 10-11V（标准12V电源）
   * - 24V 系统: 20-22V（标准24V电源）
   * - 通用设置: 6-8V（兼容多种电源）
   *
   * 【相关寄存器】
   * - Present Input Voltage: 当前电压监控
   * - Max Voltage Limit: 最高电压限制
   * - Shutdown: 关断条件设置
   */
  float ReadMinVoltageLimit() {
    uint16_t raw;
    ReadRegField(ControlTable::kMinVoltageLimit, raw);
    return VoltageFromRaw(raw);
  }

  /**
   * @brief 设置最低电压限制 (R/W)
   * @param[in] voltage 最低电压限制（V）
   */
  void WriteMinVoltageLimit(const float voltage) {
    WriteRegField(ControlTable::kMinVoltageLimit, VoltageToRaw(voltage));
  }

  /**
   * @brief 获取 PWM 上限 (R/W)
   * @param[out] percent PWM 上限（%）
   *
   * 范围: 0-100%
   *
   * 【功能说明】
   * - 限制电机驱动器的 PWM 占空比
   * - 间接限制输出力矩和电流
   * - 所有控制模式都受此限制
   * - 修改后需要重启生效
   * 
   * 【典型参数】
   * - 保守设置: 50-70%（安全运行）
   * - 标准设置: 80-90%（平衡性能）
   * - 极限设置: 95-100%（最大性能）
   *
   * 【相关寄存器】
   * - Goal PWM: PWM控制模式的目标值
   * - Present PWM: 实际PWM输出
   * - Current Limit: 电流限制
   *
   * @note Goal PWM 和 Present PWM 不会超过此值
   */
  float ReadPwmLimit() {
    uint16_t raw;
    ReadRegField(ControlTable::kPwmLimit, raw);
    return PwmFromRaw(static_cast<int16_t>(raw));
  }

  /**
   * @brief 设置 PWM 上限 (R/W)
   * @param[in] percent PWM 上限（%）
   */
  void WritePwmLimit(const float percent) {
    WriteRegField(ControlTable::kPwmLimit, PwmToRaw(percent));
  }

  /**
   * @brief 获取电流上限 (R/W)
   * @param[out] current_limit 电流上限（A）
   *
   * 【功能说明】
   * - 限制电机输出电流
   * - 保护电机和驱动电路
   * - Current-based Position Control Mode 的最大电流
   * - 修改后需要重启生效
   *
   * 【相关寄存器】
   * - Goal Current: 电流控制模式的目标值
   * - Present Current: 实际电流输出
   * - PWM Limit: PWM限制
   *
   * @warning 长时间大电流运行会导致过热
   * @note Present Current 不会超过此值
   */
  float ReadCurrentLimit() {
    uint16_t current_limit;
    ReadRegField(ControlTable::kCurrentLimit, current_limit);
    return CurrentFromRaw(current_limit);
  }

  /**
   * @brief 设置电流上限 (R/W)
   * @param[in] current_limit 电流上限（A）
   */
  void WriteCurrentLimit(const uint16_t current_limit) {
    WriteRegField(ControlTable::kCurrentLimit, current_limit);
  }

  /**
   * @brief 获取速度上限 (R/W)
   * @return rpm 速度上限（RPM）
   *
   * 范围: 0-468.763 RPM
   *
   * 【功能说明】
   * - 限制舵机运动的最大速度
   * - 保护机械结构免受高速冲击
   * - 影响所有控制模式的速度限制
   * - 修改后需要重启生效
   *
   * 【典型参数】
   * - 高速应用：400-468 RPM（接近最大值）
   * - 一般应用：200-300 RPM（平衡速度和精度）
   * - 精密应用：50-100 RPM（低速高精度）
   * - 重载应用：100-200 RPM（保护机械结构）
   *
   * 【相关寄存器】
   * - Profile Velocity: 受此限制约束
   * - Goal Velocity: Velocity Mode 下受此限制
   * - Present Velocity: 实际速度不会超过此值
   * - PWM Limit: 间接影响速度上限
   *
   * @note 实际速度还受 PWM Limit 和 Current Limit 限制
   */
  float ReadVelocityLimit() {
    uint32_t raw;
    ReadRegField(ControlTable::kVelocityLimit, raw);
    return VelocityFromRaw(raw);
  }

  /**
   * @brief 设置速度上限 (R/W)
   * @param[in] rpm 速度上限（RPM）
   */
  void WriteVelocityLimit(const float rpm) {
    WriteRegField(ControlTable::kVelocityLimit, VelocityToRaw(rpm));
  }

  /**
   * @brief 获取位置上限 (R/W)
   * @return max_position_limit 位置上限（pulse）
   *
   * 范围: 0-4095 pulse
   *
   * 【功能说明】
   * - 限制 Position Control Mode 下的最大位置
   * - Extended Position Control Mode 不受此限制
   * - Goal Position 会被限制在 [Min Position Limit, Max Position Limit] 范围内
   * - 修改后需要重启生效
   *
   * 【典型参数】
   * - 全范围：4095（0-360°）
   * - 安全范围：3500-4000（留出安全裕度）
   * - 限制范围：2000-3000（特定应用）
   *
   * 【相关寄存器】
   * - Min Position Limit: 位置下限
   * - Goal Position: 目标位置受此限制
   * - Operating Mode: 仅在 Position Control Mode 有效
   *
   * @note 确保 Max Position Limit >= Min Position Limit
   */
  uint32_t ReadMaxPositionLimit() {
    uint32_t max_position_limit;
    ReadRegField(ControlTable::kMaxPositionLimit, max_position_limit);
    return max_position_limit;
  }

  /**
   * @brief 设置位置上限 (R/W)
   * @param[in] max_position_limit 位置上限（pulse）
   *
   * @warning 如果设置不当，可能导致舵机无法移动
   * @note 修改后需调用 StoreEeprom() 并重启生效
   */
  void WriteMaxPositionLimit(const uint32_t max_position_limit) {
    WriteRegField(ControlTable::kMaxPositionLimit, max_position_limit);
  }

  /**
   * @brief 获取位置下限 (R/W)
   * @return min_position_limit 位置下限（pulse）
   *
   * 范围: 0-4095 pulse
   *
   * 【功能说明】
   * - 限制 Position Control Mode 下的最小位置
   * - Extended Position Control Mode 不受此限制
   * - Goal Position 会被限制在 [Min Position Limit, Max Position Limit] 范围内
   * - 修改后需要重启生效
   *
   * 【典型参数】
   * - 全范围：0（0°）
   * - 安全范围：50-100（留出安全裕度）
   * - 限制范围：500-1000（特定应用）
   *
   * 【相关寄存器】
   * - Max Position Limit: 位置上限
   * - Goal Position: 目标位置受此限制
   * - Operating Mode: 仅在 Position Control Mode 有效
   *
   * @note 确保 Max Position Limit >= Min Position Limit
   */
  uint32_t ReadMinPositionLimit() {
    uint32_t min_position_limit = 0;
    ReadRegField(ControlTable::kMinPositionLimit, min_position_limit);
    return min_position_limit;
  }

  /**
   * @brief 设置位置下限 (R/W)
   * @param[in] min_position_limit 位置下限（pulse）
   */
  void WriteMinPositionLimit(const uint32_t min_position_limit) {
    WriteRegField(ControlTable::kMinPositionLimit, min_position_limit);
  }

  /**
   * @brief 获取保护时间 (R/W)
   * @return protection_time 保护时间（ms）
   */
  uint8_t ReadProtectionTime() {
    uint8_t protection_time;
    ReadRegField(ControlTable::kProtectionTime, protection_time);
    return MsFromRaw(protection_time);
  }

  /**
   * @brief 设置保护时间 (R/W)
   * @param[in] protection_time 保护时间（ms）
   */
  void WriteProtectionTime(const uint8_t protection_time) {
    WriteRegField(ControlTable::kProtectionTime, MsToRaw(protection_time));
  }

#pragma endregion  // "保护限制组"

  //==============================================================================
  // PID 参数组 (0x60-0x6F, EEPROM)
  //==============================================================================
#pragma region "PID 参数组"
  /**
   * @brief 获取速度环 I 增益 (R/W)
   * @return velocity_igain I 增益
   *
   * 范围: 0.0-0.25
   *
   * 【功能说明】
   * - 速度环积分增益，用于消除速度稳态误差
   * - 提高恒速控制精度
   * - 仅在 Velocity Control Mode 和 Position Control Mode 有效
   * - 作为内环控制器的积分项
   *
   * 【控制结构】
   * XL330 采用级联控制结构：
   * Goal Position → [Position PID] → Goal Velocity → [Velocity PI] → Goal
   * Current → [Current PI] → PWM
   *
   * 【调试建议】
   * 1. 先调好 P 增益
   * 2. 如果存在稳态误差，缓慢增加 I 增益
   * 3. 观察是否出现低频振荡
   * 4. I 增益过大会导致系统不稳定
   *
   * 【典型参数】
   * - 保守设置: 0.01-0.05（稳定优先）
   * - 标准设置: 0.05-0.15（平衡性能）
   * - 高精度设置: 0.15-0.25（精度优先）
   *
   * 【相关寄存器】
   * - Velocity P Gain: 速度环比例增益
   * - Position PID: 外环位置控制器
   * - Operating Mode: 控制模式设置
   *
   * @note 仅在 Velocity Control Mode 和 Position Control Mode 有效
   */
  float ReadVelocityIgain() {
    uint16_t raw;
    ReadRegField(ControlTable::kVelocityIgain, raw);
    return PidIGainFromRaw(raw);
  }

  /**
   * @brief 设置速度环 I 增益 (R/W)
   * @param[in] velocity_igain I 增益
   */
  void WriteVelocityIgain(const float velocity_igain) {
    WriteRegField(ControlTable::kVelocityIgain, PidIGainToRaw(velocity_igain));
  }

  /**
   * @brief 获取速度环 P 增益 (R/W)
   * @return velocity_pgain P 增益
   *
   * 范围: 0.0-127.99
   *
   * 【功能说明】
   * - 速度环比例增益，决定速度响应速度
   * - 影响速度跟踪精度和稳定性
   * - 仅在 Velocity Control Mode 和 Position Control Mode 有效
   * - 作为内环控制器的主要增益
   *
   * 【调试建议】
   * 1. 从较小值开始调试
   * 2. 逐渐增加直到出现轻微振荡
   * 3. 回退到稳定值
   * 4. P 增益是系统稳定性的基础
   *
   * 【相关寄存器】
   * - Velocity I Gain: 速度环积分增益
   * - Position PID: 外环位置控制器
   * - Operating Mode: 控制模式设置
   *
   * @note 仅在 Velocity Control Mode 和 Position Control Mode 有效
   */
  float ReadVelocityPgain() {
    uint16_t raw;
    ReadRegField(ControlTable::kVelocityPgain, raw);
    return PidPGainFromRaw(raw);
  }

  /**
   * @brief 设置速度环 P 增益 (R/W)
   * @param[in] velocity_pgain P 增益
   */
  void WriteVelocityPgain(const float velocity_pgain) {
    WriteRegField(ControlTable::kVelocityPgain, PidPGainToRaw(velocity_pgain));
  }

  /**
   * @brief 获取位置环 D 增益 (R/W)
   * @return position_dgain D 增益
   *
   * 范围: 0.0-1023.9
   *
   * 【功能说明】
   * - 位置环微分增益，用于抑制超调和振荡
   * - 提高位置控制稳定性
   * - 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * - 作为外环控制器的微分项
   *
   * 【调试建议】
   * 1. 先调好 P 和 I 增益
   * 2. 如果存在超调或振荡，增加 D 增益
   * 3. D 增益过大会导致高频噪声放大
   * 4. 通常 D 增益较小，0.1-1.0 范围
   *
   * 【典型参数】
   * - 保守设置: 50-100（稳定优先）
   * - 标准设置: 100-300（平衡性能）
   * - 高精度设置: 300-500（精度优先）
   *
   * 【相关寄存器】
   * - Position P Gain: 位置环比例增益
   * - Position I Gain: 位置环积分增益
   * - Operating Mode: 控制模式设置
   *
   * @warning D 增益过大会放大高频噪声
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   */
  float ReadPositionDgain() {
    uint16_t raw;
    ReadRegField(ControlTable::kPositionDgain, raw);
    return PidDGainFromRaw(raw);
  }

  /**
   * @brief 设置位置环 D 增益 (R/W)
   * @param[in] position_dgain D 增益
   */
  void WritePositionDgain(const float position_dgain) {
    WriteRegField(ControlTable::kPositionDgain, PidDGainToRaw(position_dgain));
  }

  /**
   * @brief 获取位置环 I 增益 (R/W)
   * @return position_igain I 增益
   *
   * 范围: 0.0-0.25
   *
   * 【功能说明】
   * - 位置环积分增益，用于消除位置稳态误差
   * - 提高位置控制精度
   * - 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * - 作为外环控制器的积分项
   *
   * 【调试建议】
   * 1. 先调好 P 和 D 增益
   * 2. 如果存在稳态误差，缓慢增加 I 增益
   * 3. 观察是否出现低频振荡
   * 4. I 增益过大会导致系统不稳定
   *
   * 【典型参数】
   * - 保守设置: 0.01-0.05（稳定优先）
   * - 标准设置: 0.05-0.15（平衡性能）
   * - 高精度设置: 0.15-0.25（精度优先）
   *
   * 【相关寄存器】
   * - Position P Gain: 位置环比例增益
   * - Position D Gain: 位置环微分增益
   * - Operating Mode: 控制模式设置
   *
   * @warning I 增益过大会导致系统不稳定
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   */
  float ReadPositionIgain() {
    uint16_t raw;
    ReadRegField(ControlTable::kPositionIgain, raw);
    return PidIGainFromRaw(raw);
  }

  /**
   * @brief 设置位置环 I 增益 (R/W)
   * @param[in] position_igain I 增益
   */
  void WritePositionIgain(const float position_igain) {
    WriteRegField(ControlTable::kPositionIgain, PidIGainToRaw(position_igain));
  }

  /**
   * @brief 获取位置环 P 增益 (R/W)
   * @return position_pgain P 增益
   *
   * 范围: 0.0-127.99
   *
   * 【功能说明】
   * - 位置环比例增益，决定位置响应速度
   * - 影响位置跟踪精度和稳定性
   * - 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * - 作为外环控制器的主要增益
   *
   * 【调试建议】
   * 1. 从较小值开始调试
   * 2. 逐渐增加直到出现轻微振荡
   * 3. 回退到稳定值
   * 4. P 增益是系统稳定性的基础
   *
   * 【典型参数】
   * - 保守设置: 1-10（稳定优先）
   * - 标准设置: 10-50（平衡性能）
   * - 高响应设置: 50-127（响应优先）
   *
   * 【相关寄存器】
   * - Position I Gain: 位置环积分增益
   * - Position D Gain: 位置环微分增益
   * - Operating Mode: 控制模式设置
   *
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   */
  float ReadPositionPgain() {
    uint16_t raw;
    ReadRegField(ControlTable::kPositionPgain, raw);
    return PidPGainFromRaw(raw);
  }

  /**
   * @brief 设置位置环 P 增益 (R/W)
   * @param[in] position_pgain P 增益
   */
  void WritePositionPgain(const float position_pgain) {
    WriteRegField(ControlTable::kPositionPgain, PidPGainToRaw(position_pgain));
  }

  /**
   * @brief 获取前馈二阶增益 (R/W)
   * @return feedforward_2nd_gain 前馈二阶增益
   *
   * 范围: 0.0-127.99
   *
   * 【功能说明】
   * - 前馈控制器的二阶增益，用于改善动态响应
   * - 减少位置跟踪误差
   * - 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * - 作为前馈控制器的二阶项
   *
   * 【调试建议】
   * 1. 先调好 PID 参数
   * 2. 如果存在跟踪误差，增加前馈增益
   * 3. 前馈增益过大会导致系统不稳定
   * 4. 通常前馈增益较小，0.1-1.0 范围
   *
   * 【典型参数】
   * - 保守设置: 0.1-0.5（稳定优先）
   * - 标准设置: 0.5-2.0（平衡性能）
   * - 高精度设置: 2.0-5.0（精度优先）
   *
   * 【相关寄存器】
   * - Feedforward 1st Gain: 前馈一阶增益
   * - Position PID: 位置控制器
   * - Operating Mode: 控制模式设置
   *
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   */
  float ReadFeedforward2ndGain() {
    uint16_t raw;
    ReadRegField(ControlTable::kFeedforward2ndGain, raw);
    return FeedforwardGainFromRaw(raw);
  }

  /**
   * @brief 设置前馈二阶增益 (R/W)
   * @param[in] feedforward_2nd_gain 前馈二阶增益
   */
  void WriteFeedforward2ndGain(const float feedforward_2nd_gain) {
    WriteRegField(ControlTable::kFeedforward2ndGain,
                  FeedforwardGainToRaw(feedforward_2nd_gain));
  }

  /**
   * @brief 获取前馈一阶增益 (R/W)
   * @return feedforward_1st_gain 前馈一阶增益
   *
   * 范围: 0.0-127.99
   *
   * 【功能说明】
   * - 前馈控制器的一阶增益，用于改善动态响应
   * - 减少位置跟踪误差
   * - 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * - 作为前馈控制器的一阶项
   *
   * 【调试建议】
   * 1. 先调好 PID 参数
   * 2. 如果存在跟踪误差，增加前馈增益
   * 3. 前馈增益过大会导致系统不稳定
   * 4. 通常前馈增益较小，0.1-1.0 范围
   *
   * 【典型参数】
   * - 保守设置: 0.1-0.5（稳定优先）
   * - 标准设置: 0.5-2.0（平衡性能）
   * - 高精度设置: 2.0-5.0（精度优先）
   *
   * 【相关寄存器】
   * - Feedforward 2nd Gain: 前馈二阶增益
   * - Position PID: 位置控制器
   * - Operating Mode: 控制模式设置
   *
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   */
  float ReadFeedforward1stGain() {
    uint16_t raw;
    ReadRegField(ControlTable::kFeedforward1stGain, raw);
    return FeedforwardGainFromRaw(raw);
  }

  /**
   * @brief 设置前馈一阶增益 (R/W)
   * @param[in] feedforward_1st_gain 前馈一阶增益
   */
  void WriteFeedforward1stGain(const float feedforward_1st_gain) {
    WriteRegField(ControlTable::kFeedforward1stGain,
                  FeedforwardGainToRaw(feedforward_1st_gain));
  }

#pragma endregion  // "PID 参数组"

  //==============================================================================
  // 轨迹配置组 (0x70-0x7F, EEPROM)
  //==============================================================================
#pragma region "轨迹配置组"

  /**
   * @brief 获取轨迹加速度 (RW)
   * @return acceleration 轨迹加速度（rev/min²，Velocity-based模式）
   *
   * 范围: 0-7,032,024.959 rev/min² (Velocity-based模式) | 0-32767 ms
   * (Time-based模式)
   *
   * 【功能说明】
   * Profile Acceleration 在不同 Profile 类型下的作用：
   *
   * 【Velocity-based Profile (Drive Mode Bit 2 = 0)】
   * 单位: rev/min²
   * 作用: 设置加速度
   * - 0: 矩形 Profile（无加速过程）
   * - 非 0: 梯形 Profile（有加速和减速阶段）
   *
   * 【Time-based Profile (Drive Mode Bit 2 = 1)】
   * 单位: ms
   * 作用: 设置加速时间 (t1)
   * - 加速阶段 (0 → t1): 加速
   * - 匀速阶段 (t1 → t3-t1): 匀速
   * - 减速阶段 (t3-t1 → t3): 减速
   * - 系统自动限制: Profile Acceleration 不会超过 Profile Velocity 的 50%
   *   (确保 t1 ≤ t3/2，即加速+减速时间不超过总时间)
   *
   * 【Velocity Control Mode 中的使用】
   * - Velocity Control Mode 只使用 Profile Acceleration(108)
   * - 不使用 Profile Velocity(112)
   * - 支持的 Profile 类型:
   *   * Step: Profile Acceleration = 0（立即达到目标速度）
   *   * Trapezoidal: Profile Acceleration ≠ 0（平滑加速到目标速度）
   * - 加速时间计算同上述 Velocity-based 公式
   *
   * 【典型参数】
   * - 快速运动: 1000-3000 rev/min²
   * - 一般运动: 500-1500 rev/min²
   * - 精密定位: 200-800 rev/min²
   * - 重载应用: 100-500 rev/min²
   *
   * 【相关寄存器】
   * - Profile Velocity: 配合设置速度曲线
   * - Drive Mode: 决定 Profile 类型
   * - Velocity Limit: 限制最大速度
   * - Goal Position: 运动的目标位置
   *
   * @note 在 Velocity Control Mode 中也使用此参数控制加速度
   * @note Time-based模式下单位为ms，当前实现假设Velocity-based模式
   */
  float ReadProfileAcceleration() {
    uint32_t raw;
    ReadRegField(ControlTable::kProfileAcceleration, raw);
    return AccelerationFromRaw(raw);
  }

  /**
   * @brief 设置轨迹加速度
   * @param[in] acceleration 轨迹加速度（rev/min²，Velocity-based模式）
   */
  void WriteProfileAcceleration(const float acceleration) {
    WriteRegField(ControlTable::kProfileAcceleration,
                  AccelerationToRaw(acceleration));
  }

  /**
   * @brief 获取轨迹速度 (RW)
   * @param[out] rpm 轨迹速度（RPM，Velocity-based模式）
   *
   * 范围: 0-7,503.643 RPM (Velocity-based模式) | 0-32767 ms (Time-based模式)
   *
   * 【Profile 处理流程】
   * 根据官方文档，当接收到新的 Goal Position 时：
   *
   * 1. 指令通过 DYNAMIXEL 总线传输，注册到 Goal Position
   * 2. 基于 Profile Velocity 和 Profile Acceleration 计算加速时间 t1
   * 3. 根据以下条件决定 Profile 类型：
   *    - Profile Velocity = 0 → Step (不使用 Profile)
   *    - Profile Velocity ≠ 0, Profile Acceleration = 0 → Rectangular
   *    - Profile Velocity ≠ 0, Profile Acceleration ≠ 0 → Trapezoidal
   *    - 移动距离不足时，Trapezoidal 自动降级为 Triangular
   * 4. 选择的 Profile 类型存储在 Moving Status(123) 的 Bit 4-5
   * 5. Profile Generator 计算期望轨迹:
   *    - Position Trajectory(140): 期望位置轨迹
   *    - Velocity Trajectory: 期望速度轨迹
   * 6. 控制器跟踪期望轨迹驱动电机
   *
   * 【运动中更新 Goal Position】
   * - 如果在运动过程中更新 Goal Position:
   *   * 基于当前运动速度重新生成轨迹
   *   * 速度平滑过渡到新的期望轨迹
   *   * 避免速度突变，保护机械结构
   *
   * 【功能说明】
   * Profile Velocity 在不同 Profile 类型下的作用：
   *
   * 【Velocity-based Profile (Drive Mode Bit 2 = 0)】
   * 单位: RPM
   * 作用: 设置运动的最大速度
   * - 0: 不使用 Profile（Step Instruction）
   *   * 立即以最大速度移动（受 Velocity Limit 限制）
   *   * 无加减速过程，可能对机械产生冲击
   *   * 适用于调试或对响应速度要求极高的场景
   * - 非 0: 使用梯形或矩形速度曲线
   * - 受 Velocity Limit(44) 限制
   *
   * 【Time-based Profile (Drive Mode Bit 2 = 1)】
   * 单位: ms
   * 作用: 设置运动的总时间 (t3)
   * - Profile Acceleration(108) 设置加速时间 (t1)
   * - 运动时间 = Profile Velocity
   * - Profile Acceleration ≤ 50% of Profile Velocity
   *
   * 【Profile 类型选择】
   * | Profile Velocity | Profile Acceleration | Profile 类型      |
   * |------------------|----------------------|-------------------|
   * | 0                | -                    | Step (无 Profile) |
   * | 非 0             | 0                    | Rectangular       |
   * | 非 0             | 非 0                 | Trapezoidal       |
   *
   * 【加速时间计算】
   *
   * Velocity-based Profile:
   *   t1 = 64 × (Profile Velocity / Profile Acceleration)
   *
   *   其中:
   *   - t1: 加速时间（毫秒 ms）
   *   - Profile Velocity: 单位 0.229 RPM/tick
   *   - Profile Acceleration: 单位 214.577 rev/min²/tick
   *   - 系数 64: 单位转换常数
   *
   * Time-based Profile:
   *   t1 = Profile Acceleration(108)
   *
   *   其中:
   *   - t1: 加速时间（毫秒 ms）
   *   - Profile Acceleration 直接设置加速时间
   *   - 自动限制: t1 ≤ 50% × Profile Velocity (即 t1 ≤ t3/2)
   *
   * 【典型参数】
   * - 快速定位: 200-400 RPM
   * - 一般运动: 100-200 RPM
   * - 精密定位: 50-100 RPM
   * - 连续旋转: 设置较大值实现连续运动
   *
   * 【相关寄存器】
   * - Profile Acceleration: 配合设置加速度
   * - Velocity Limit: 限制最大速度
   * - Drive Mode: 决定 Profile 类型
   * - Goal Position: 运动的目标位置
   *
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * @note Time-based模式下单位为ms，当前实现假设Velocity-based模式
   */
  float ReadProfileVelocity() {
    uint32_t raw;
    ReadRegField(ControlTable::kProfileVelocity, raw);
    return VelocityFromRaw(raw);
  }

  /**
   * @brief 设置轨迹速度
   * @param[in] rpm 轨迹速度（RPM，Velocity-based模式）
   */
  void WriteProfileVelocity(const float rpm) {
    WriteRegField(ControlTable::kProfileVelocity, VelocityToRaw(rpm));
  }

#pragma endregion  // "轨迹配置组"

  //==============================================================================
  // 控制命令组 (0x80-0x8F, RAM)
  //==============================================================================
#pragma region "控制命令组"
  /**
   * @brief 获取力矩使能状态 (R/W)
   * @return torque_enable 力矩使能状态
   *
   * 范围: 0-1
   *
   * 【功能说明】
   * - 控制舵机力矩输出的使能状态
   * - 0: 禁用力矩，电机可以自由转动（关断电机驱动器）
   * - 1: 使能力矩，电机根据控制指令运动
   * - 修改 EEPROM 参数（如 Operating Mode）前必须设置为 0
   * - 上电后默认为 0，需要手动使能
   * - 发生 Shutdown 错误时会自动变为 0
   *
   * 【操作流程】
   * 1. 使能力矩：WriteTorqueEnable(1) - 电机会立即执行控制指令
   * 2. 禁用力矩：WriteTorqueEnable(0) - 电机可以自由转动
   * 3. 模式切换：必须先禁用，再切换模式，最后重新使能
   *
   * 【相关寄存器】
   * - Operating Mode: 切换模式前必须禁用
   * - Hardware Error Status: 错误时会自动禁用
   * - Shutdown: 关断条件设置
   *
   * @warning 使能力矩后电机会立即执行控制指令，注意安全
   * @note 禁用力矩时，Present Position 等反馈值仍然有效
   */
  uint8_t ReadTorqueEnable() {
    uint8_t torque_enable = 0;
    ReadRegField(ControlTable::kTorqueEnable, torque_enable);
    return torque_enable;
  }

  /**
   * @brief 设置力矩使能 (R/W)
   * @param[in] torque_enable 力矩使能 (0: 禁用, 1: 使能)
   */
  void WriteTorqueEnable(const uint8_t torque_enable) {
    WriteRegField(ControlTable::kTorqueEnable, torque_enable);
  }

  /**
   * @brief 获取 LED 状态 (R/W)
   * @return dxl_led LED 状态
   *
   * 范围: 0-1
   *
   * 【功能说明】
   * - 控制舵机上的 LED 指示灯
   * - 0: LED 关闭
   * - 1: LED 开启
   * - 用于状态指示和调试
   */
  uint8_t ReadDxlLed() {
    uint8_t dxl_led = 0;
    ReadRegField(ControlTable::kDxlLed, dxl_led);
    return dxl_led;
  }

  /**
   * @brief 设置 LED 开关 (R/W)
   * @param[in] dxl_led LED 状态 (0: 关, 1: 开)
   */
  void WriteDxlLed(const uint8_t dxl_led) {
    WriteRegField(ControlTable::kDxlLed, dxl_led);
  }

  /**
   * @brief 获取对齐到目标位置 (W)
   * @return align_to_position 对齐到目标位置
   */
  uint16_t ReadAlignToPosition() {
    uint16_t align_to_position = 0;
    ReadRegField(ControlTable::kAlignToPosition, align_to_position);
    return align_to_position != 0;
  }

  /**
   * @brief 设置设置为居中位置 (W)
   * @param[in] align_to_position 对齐到目标位置
   * 范围: 0-1
   * 【功能说明】
   * - 调整Homing Offset使Present Position为目标位置
   */
  void WriteAlignToPosition(const uint16_t align_to_position) {
    WriteRegField(ControlTable::kAlignToPosition, align_to_position);
  }

  /**
   * @brief 获取硬件错误状态 (R)
   * @return hardware_error_status 硬件错误状态位域
   *
   * 范围: 0-255 (8位位域)
   *
   * 【功能说明】
   * - 读取舵机硬件错误状态，提供详细的故障诊断信息
   * - 错误状态会持续存在，直到手动清除
   * - 位域寄存器，每个 bit 表示一种错误类型
   *
   * 【硬件错误位域】
   * | Bit | 错误类型               | 说明                          |
   * |-----|------------------------|-------------------------------|
   * | 0   | Input Voltage Error    | 输入电压超出范围              |
   * | 1   | Overheating Error      | 温度超过上限                  |
   * | 2   | Motor Encoder Error    | 编码器故障                    |
   * | 3   | Electrical Shock Error | 电气冲击                      |
   * | 4   | Overload Error         | 过载                          |
   * | 5   | Angle Limit Error      | 角度超出范围                  |
   * | 6   | Range Error            | 范围错误                      |
   *
   * 【相关寄存器】
   * - Shutdown: 决定哪些错误会自动关断力矩
   * - Torque Enable: 错误清除后需要重新使能
   * - Present Temperature: 监控温度状态
   * - Present Input Voltage: 监控电压状态
   * - Present Current: 监控电流状态
   *
   * @note 如果 Shutdown 对应位启用，发生错误会自动关断力矩
   */
  uint8_t ReadHardwareErrorStatus() {
    uint8_t hardware_error_status = 0;
    ReadRegField(ControlTable::kHardwareErrorStatus, hardware_error_status);
    return hardware_error_status;
  }

  /**
   * @brief 设置硬件错误状态 (R)
   * @param[in] hardware_error_status 硬件错误状态位域
   */
  void WriteHardwareErrorStatus(const uint8_t hardware_error_status) {
    WriteRegField(ControlTable::kHardwareErrorStatus, hardware_error_status);
  }

  /**
   * @brief 获取总线看门狗 (R/W)
   * @return milliseconds 看门狗时间（ms）
   *
   * 范围: 0-5080 ms
   *
   * 【功能说明】
   * - 总线看门狗超时时间，用于检测通信故障
   * - 当超过此时间未收到指令时，舵机会自动关断力矩
   * - 0: 禁用看门狗功能
   * - 非0: 启用看门狗功能
   *
   * 【典型参数】
   * - 禁用看门狗: 0（调试时使用）
   * - 快速检测: 100-500 ms（高响应要求）
   * - 标准检测: 1000-2000 ms（一般应用）
   * - 慢速检测: 3000-5080 ms（低频率通信）
   *
   * 【相关寄存器】
   * - Torque Enable: 看门狗触发时会自动禁用
   * - Hardware Error Status: 看门狗超时会记录错误
   *
   * @note 看门狗超时会自动关断力矩，需要重新使能
   */
  uint16_t ReadBusWatchdog() {
    uint8_t raw;
    ReadRegField(ControlTable::kBusWatchdog, raw);
    return MsFromRaw(raw);
  }

  /**
   * @brief 设置总线看门狗 (R/W)
   * @param[in] milliseconds 看门狗时间（ms）
   */
  void WriteBusWatchdog(const uint16_t milliseconds) {
    WriteRegField(ControlTable::kBusWatchdog, MsToRaw(milliseconds));
  }

#pragma endregion  // "控制命令组"

  //==============================================================================
  // 目标值组 (0x90-0x9F, RAM)
  //==============================================================================
#pragma region "目标值组"

  /**
   * @brief 获取目标 PWM (R/W)
   * @return percent 目标 PWM（%）
   *
   * 范围: -100.0 ~ 100.0%
   *
   * 【功能说明】
   * - 设置电机驱动器的 PWM 占空比
   * - 仅在 PWM Control Mode (Operating Mode = 16) 有效
   * - 负值表示反向旋转
   * - 受 PWM Limit 限制
   *
   * 【典型参数】
   * - 停止: 0%（无输出）
   * - 低速: ±10-30%（低速运行）
   * - 中速: ±30-70%（中速运行）
   * - 高速: ±70-100%（高速运行）
   *
   * 【相关寄存器】
   * - Operating Mode: 仅在 PWM Control Mode 有效
   * - PWM Limit: 限制最大 PWM 输出
   * - Present PWM: 实际 PWM 输出
   *
   * @note 仅在 PWM Control Mode 有效
   */
  float ReadGoalPwm() {
    uint16_t raw;
    ReadRegField(ControlTable::kGoalPwm, raw);
    return PwmFromRaw(raw);
  }

  /**
   * @brief 设置目标 PWM (R/W)
   * @param[in] percent 目标 PWM（%）
   */
  void WriteGoalPwm(const float percent) {
    WriteRegField(ControlTable::kGoalPwm, PwmToRaw(percent));
  }

  /**
   * @brief 获取目标电流 (R/W)
   * @return goal_current 目标电流（A）
   *
   * 范围: -65.535 ~ 65.535 A
   *
   * 【功能说明】
   * - 设置电机输出电流目标值
   * - 在 Current Control Mode (Operating Mode = 0) 中作为控制目标
   * - 在 Current-based Position Control Mode (Operating Mode = 5)
   * 中作为最大电流限制
   * - 负值表示反向电流
   * - 受 Current Limit 限制
   *
   * 【相关寄存器】
   * - Operating Mode: 决定电流控制模式
   * - Current Limit: 限制最大电流
   * - Present Current: 实际电流输出
   *
   * @note 在 Current Control Mode 中作为控制目标，在 Current-based Position Control Mode 中作为限制
   */
  uint16_t ReadGoalCurrent() {
    uint16_t goal_current;
    ReadRegField(ControlTable::kGoalCurrent, goal_current);
    return CurrentFromRaw(goal_current);
  }

  /**
   * @brief 设置目标电流 (R/W)
   * @param[in] goal_current 目标电流（0.001A）
   */
  void WriteGoalCurrent(const float goal_current) {
    WriteRegField(ControlTable::kGoalCurrent, CurrentToRaw(goal_current));
  }

  /**
   * @brief 获取目标速度 (R/W)
   * @return rpm 目标速度（RPM）
   *
   * 范围: -468.763 到 468.763 RPM
   *
   * 【功能说明】
   * - 设置电机转速目标值
   * - 在 Velocity Control Mode (Operating Mode = 1) 中作为控制目标
   * - 在 Position Control Mode 和 Extended Position Control Mode
   * 中由位置控制器自动计算
   * - 负值表示反向旋转
   * - 受 Velocity Limit 限制
   *
   * 【典型参数】
   * - 停止: 0 RPM（无旋转）
   * - 低速: ±10-50 RPM（低速运行）
   * - 中速: ±50-200 RPM（中速运行）
   * - 高速: ±200-468 RPM（高速运行）
   *
   * 【相关寄存器】
   * - Operating Mode: 决定速度控制模式
   * - Velocity Limit: 限制最大速度
   * - Present Velocity: 实际速度输出
   *
   * @note 仅在 Velocity Control Mode 中作为控制目标
   */
  float ReadGoalVelocity() {
    uint32_t raw;
    ReadRegField(ControlTable::kGoalVelocity, raw);
    return VelocityFromRaw(raw);
  }

  /**
   * @brief 设置目标速度 (R/W)
   * @param[in] rpm 目标速度（RPM）
   */
  void WriteGoalVelocity(const float rpm) {
    WriteRegField(ControlTable::kGoalVelocity, VelocityToRaw(rpm));
  }

  /**
   * @brief 获取目标位置 (RW)
   * @return goal_position 目标位置（pulse）
   *
   * 范围: 0-4095 (Position Mode) | -1,048,575 到 1,048,575 (Extended Mode)
   *
   * 【功能说明】
   * 设置舵机的目标位置，实际行为取决于当前 Operating Mode：
   * - Position Control Mode: 单圈位置控制，范围 0-4095 (0-360°)
   * - Extended Position Control Mode: 多圈位置控制，支持无限旋转
   *
   * 【典型参数】
   * - Position Mode: 0-4095 (0-360°)
   * - Extended Mode: ±1000000 (约±244圈)
   * - 连续旋转: 设置超出物理范围的值
   *
   * 【相关寄存器】
   * - Operating Mode: 决定位置范围和行为
   * - Homing Offset: 会加到目标位置上
   * - Min/Max Position Limit: 仅在 Position Mode 下有效
   * - Profile Velocity/Acceleration: 控制运动轨迹
   *
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * @note 运动中更新 Goal Position 会平滑过渡到新轨迹
   */
  int32_t ReadGoalPosition() {
    uint32_t raw;
    ReadRegField(ControlTable::kGoalPosition, raw);
    return static_cast<int32_t>(raw);
  }

  /**
   * @brief 设置目标位置 (RW)
   * @param[in] goal_position 目标位置（pulse）
   */
  void WriteGoalPosition(const int32_t goal_position) {
    WriteRegField(ControlTable::kGoalPosition,
                  static_cast<uint32_t>(goal_position));
  }

#pragma endregion  // "目标值组"

  //==============================================================================
  // 状态反馈组 (0xA0-0xBF, RAM，只读)
  //==============================================================================
#pragma region "状态反馈组"
  /**
   * @brief 获取实时时钟 (R)
   * @param[out] realtime_tick 实时时钟（ms）
   *
   * 范围: 0-32767 ms
   *
   * 【功能说明】
   * - 舵机内部实时时钟计数器
   * - 以毫秒为单位递增
   * - 用于时间戳和定时功能
   * - 上电后从 0 开始计数
   * - 达到最大值后重新从 0 开始
   *
   * @note 此寄存器为只读，由系统自动更新
   */
  uint16_t ReadRealtimeTick() {
    uint16_t realtime_tick;
    ReadRegField(ControlTable::kRealtimeTick, realtime_tick);
    return realtime_tick;
  }

  /**
   * @brief 设置实时时钟 (R)
   * @param[in] realtime_tick 实时时钟
   */
  void WriteRealtimeTick(const uint16_t realtime_tick) {
    WriteRegField(ControlTable::kRealtimeTick, realtime_tick);
  }

  /**
   * @brief 获取运动状态 (R)
   * @param[out] moving 运动状态
   *
   * 范围: 0-1
   *
   * 【功能说明】
   * - 简化的运动状态指示
   * - 0: 舵机静止
   * - 1: 舵机正在运动
   * - 基于 Moving Threshold 判断
   * - 实时更新，反映当前运动状态
   *
   * 【判断逻辑】
   * - 当 Present Velocity ≥ Moving Threshold 时，Moving = 1
   * - 当 Present Velocity < Moving Threshold 时，Moving = 0
   *
   * 【相关寄存器】
   * - Moving Threshold: 运动判断阈值
   * - Present Velocity: 实际速度值
   * - Moving Status: 详细的运动状态
   *
   * @note 此寄存器为只读，由系统自动更新
   */
  bool ReadMoving() {
    uint8_t moving;
    ReadRegField(ControlTable::kMoving, moving);
    return moving != 0;
  }

  /**
   * @brief 设置运动状态 (R)
   * @param[in] moving 运动状态
   */
  void WriteMoving(const bool moving) {
    WriteRegField(ControlTable::kMoving, moving);
  }

  /**
   * @brief 获取详细运动状态 (R)
   * @param[out] moving_status 详细运动状态位域
   *
   * 范围: 0-255 (8位位域)
   *
   * 【功能说明】
   * - 提供详细的运动状态信息
   * - 位域寄存器，每个 bit 表示不同的状态
   * - 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * - 实时更新，反映当前运动状态
   *
   * 【位域定义】
   * | Bit | 名称                   | 说明                          |
   * |-----|------------------------|-------------------------------|
   * | 0   | In-Position            | 0: 未到达目标位置, 1: 已到达  |
   * | 1   | Profile Ongoing        | 0: 轨迹已完成, 1: 轨迹进行中  |
   * | 2   | Reserved               | 保留位                        |
   * | 3   | Following Error        | 0: 正在跟随, 1: 未跟随轨迹    |
   * | 4-5 | Velocity Profile       | 00: Step,                     |
   * |     |                        | 01: Rectangular,              |
   * |     |                        | 10: Triangular,               |
   * |     |                        | 11: Trapezoidal               | 
   * | 6-7 | Reserved               | 保留位                        |
   *
   * 【Profile 类型详解】
   *
   * 参数配置（3 种基本类型）：
   * | Profile Velocity | Profile Acceleration | 配置的类型   |
   * |------------------|----------------------|--------------|
   * | 0                | -                    | Step         |
   * | 非 0             | 0                    | Rectangular  |
   * | 非 0             | 非 0                 | Trapezoidal  |
   *
   * 运行时状态（4 种可能状态，存储在 Bit 4-5）：
   * | 编码 | 类型        | 说明                                  |
   * |------|------------|----------------------------------------|
   * | 00   | Step       | 不使用 Profile，立即以最大速度移动     |
   * | 01   | Rectangular| 矩形曲线：匀速运动，无加减速过程       |
   * | 10   | Triangular | 三角形曲线：只有加速和减速，无匀速段   |
   * | 11   | Trapezoidal| 梯形曲线：完整的加速-匀速-减速过程     |
   *
   * 【Triangular vs Trapezoidal】
   * - 当配置为 Trapezoidal 时（Profile Velocity ≠ 0, Profile Acceleration ≠ 0）
   * - 如果移动距离足够长：执行完整梯形曲线 → Moving Status 显示 11
   * (Trapezoidal)
   * - 如果移动距离太短：自动降级为三角形曲线 → Moving Status 显示 10
   * (Triangular)
   * - Triangular 是系统根据实际距离自动选择的，无法直接配置
   *
   * 【状态说明】
   * - In-Position: 当前位置是否在目标位置容差范围内
   * - Profile Ongoing: 是否正在执行运动轨迹
   * - Following Error: 是否出现跟随误差
   * - Velocity Profile: 当前使用的速度曲线类型
   *
   * 【相关寄存器】
   * - Goal Position: 目标位置
   * - Present Position: 当前位置
   * - Profile Velocity: 速度曲线参数
   * - Profile Acceleration: 加速度参数
   *
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   */
  uint8_t ReadMovingStatus() {
    uint8_t moving_status;
    ReadRegField(ControlTable::kMovingStatus, moving_status);
    return moving_status;
  }

  /**
   * @brief 设置详细运动状态 (R)
   * @param[in] moving_status 详细运动状态位域
   */
  void WriteMovingStatus(const uint8_t moving_status) {
    WriteRegField(ControlTable::kMovingStatus, moving_status);
  }

  /**
   * @brief 获取当前 PWM (R)
   * @param[out] percent 当前 PWM（%）
   *
   * 范围: -100.0% 到 100.0%
   *
   * 【功能说明】
   * - 显示实际输出的 PWM 占空比
   * - 正值表示正向旋转
   * - 负值表示反向旋转
   * - 实时更新，反映当前输出状态
   *
   * 【相关寄存器】
   * - Goal PWM: 目标 PWM 值
   * - PWM Limit: PWM 限制值
   * - Torque Enable: 力矩使能状态
   */
  float ReadPresentPwm() {
    int16_t raw;
    ReadRegField(ControlTable::kPresentPwm, raw);
    return PwmFromRaw(raw);
  }

  /**
   * @brief 设置当前 PWM (R)
   * @param[in] present_pwm 当前 PWM
   */
  void WritePresentPwm(const uint16_t present_pwm) {
    WriteRegField(ControlTable::kPresentPwm, present_pwm);
  }

  /**
   * @brief 获取当前电流 (R)
   * @return present_current 当前电流（A）
   *
   * 【功能说明】
   * - 显示当前通过电机的电流
   * - 电流与负载力矩成正比
   * - 实时更新，反映当前电流状态
   * - 用于监控电机负载和功耗
   *
   * 【相关寄存器】
   * - Goal Current: 目标电流值
   * - Current Limit: 电流限制值
   * - Torque Enable: 力矩使能状态
   *
   * @note 此寄存器为只读，由系统自动更新
   */
  float ReadPresentCurrent() {
    uint16_t present_current;
    ReadRegField(ControlTable::kPresentCurrent, present_current);
    return CurrentFromRaw(present_current);
  }

  /**
   * @brief 设置当前电流 (R)
   * @param[in] present_current 当前电流
   */
  void WritePresentCurrent(const float present_current) {
    WriteRegField(ControlTable::kPresentCurrent, CurrentToRaw(present_current));
  }

  /**
   * @brief 获取当前速度 (R)
   * @param[out] rpm 当前速度（RPM）
   *
   * 范围: -468.763 到 468.763 RPM
   *
   * 【功能说明】
   * - 显示当前电机转速
   * - 通过编码器微分计算得到
   * - 正值表示正向旋转
   * - 负值表示反向旋转
   * - 实时更新，反映当前速度状态
   *
   * 【相关寄存器】
   * - Goal Velocity: 目标速度值
   * - Velocity Limit: 速度限制值
   * - Moving Threshold: 运动判断阈值
   * - Moving: 运动状态指示
   *
   * @note 此寄存器为只读，由系统自动更新
   */
  float ReadPresentVelocity() {
    uint32_t raw;
    ReadRegField(ControlTable::kPresentVelocity, raw);
    return VelocityFromRaw(raw);
  }

  /**
   * @brief 设置当前速度 (R)
   * @param[in] present_velocity 当前速度
   */
  void WritePresentVelocity(const uint32_t present_velocity) {
    WriteRegField(ControlTable::kPresentVelocity, present_velocity);
  }

  /**
   * @brief 获取当前位置 (R)
   * @param[out] present_position 当前位置（pulse）
   *
   * 范围: 0-4095 (Position Mode) | -1,048,575 到 1,048,575 (Extended Mode)
   *
   * 【功能说明】
   * - 读取舵机当前的实际位置
   * - 通过磁编码器实时反馈
   * - 位置范围取决于当前 Operating Mode
   * - 实时更新，反映当前位置状态
   *
   * 【模式差异】
   * - Position Control Mode: 0-4095 (单圈)
   * - Extended Position Control Mode: 多圈累计位置
   *
   * 【相关寄存器】
   * - Goal Position: 目标位置对比
   * - Operating Mode: 决定位置范围
   * - Moving Status: 运动状态指示
   * - Homing Offset: 零点偏移
   *
   * @note 此寄存器为只读，由系统自动更新
   */
  int32_t ReadPresentPosition() {
    uint32_t raw;
    ReadRegField(ControlTable::kPresentPosition, raw);
    return static_cast<int32_t>(raw);
  }

  /**
   * @brief 设置当前位置 (R)
   * @param[in] present_position 当前位置
   */
  void WritePresentPosition(const int32_t present_position) {
    WriteRegField(ControlTable::kPresentPosition,
                  static_cast<uint32_t>(present_position));
  }

  /**
   * @brief 获取速度轨迹 (R)
   * @param[out] rpm 速度轨迹（RPM）
   *
   * 范围: -468.763 到 468.763 RPM
   *
   * 【功能说明】
   * - Profile 生成的期望速度轨迹值
   * - 存储在 Velocity Trajectory 寄存器中
   * - 作为速度控制器的参考输入
   * - 实时更新，反映 Profile 计算的期望速度
   * - 与 Present Velocity 对比可得速度跟踪误差
   *
   * 【控制流程】
   * Goal Position → [Profile Generator] → Velocity Trajectory → [Velocity
   * PID] → PWM
   * - Profile Generator 根据 Goal Position 生成期望速度轨迹
   * - 期望速度轨迹存储在 Velocity Trajectory 中
   * - Velocity PID 控制器跟踪 Velocity Trajectory
   * - 输出 PWM 驱动电机
   *
   * 【相关寄存器】
   * - Profile Velocity: 速度曲线参数
   * - Profile Acceleration: 加速度参数
   * - Present Velocity: 实际速度值
   * - Position Trajectory: 位置轨迹
   *
   * @note 此寄存器为只读，由系统自动更新
   */
  float ReadVelocityTrajectory() {
    uint32_t raw;
    ReadRegField(ControlTable::kVelocityTrajectory, raw);
    return VelocityFromRaw(raw);
  }

  /**
   * @brief 设置速度轨迹 (R)
   * @param[in] velocity_trajectory 速度轨迹
   */
  void WriteVelocityTrajectory(const uint32_t velocity_trajectory) {
    WriteRegField(ControlTable::kVelocityTrajectory, velocity_trajectory);
  }

  /**
   * @brief 获取位置轨迹 (R)
   * @param[out] position_trajectory 位置轨迹（pulse）
   *
   * 范围: 0-4095 (Position Mode) | -1,048,575 到 1,048,575 (Extended Mode)
   *
   * 【功能说明】
   * - 读取 Profile 生成的期望位置轨迹
   * - 位置 PID 控制器的参考输入
   * - 与 Present Position 的区别：期望位置 vs 实际位置
   * - 实时更新，反映期望位置轨迹
   *
   * 【相关寄存器】
   * - Goal Position: 轨迹的最终目标
   * - Present Position: 实际位置对比
   * - Profile Velocity: 速度曲线参数
   * - Profile Acceleration: 加速度参数
   * - Moving Status: 轨迹状态指示
   *
   * @note 此寄存器为只读，由系统自动更新
   */
  int32_t ReadPositionTrajectory() {
    uint32_t raw;
    ReadRegField(ControlTable::kPositionTrajectory, raw);
    return static_cast<int32_t>(raw);
  }

  /**
   * @brief 设置位置轨迹 (R)
   * @param[in] position_trajectory 位置轨迹
   */
  void WritePositionTrajectory(const int32_t position_trajectory) {
    WriteRegField(ControlTable::kPositionTrajectory,
                  static_cast<uint32_t>(position_trajectory));
  }

  /**
   * @brief 获取当前输入电压 (R)
   * @return voltage 当前输入电压（V）
   *
   * 范围: 0-16.0 V
   *
   * 【功能说明】
   * - 显示当前输入电压
   * - 用于监控电源状态
   * - 实时更新，反映当前电压
   * - 电压异常会触发保护
   *
   * 【相关寄存器】
   * - Min Voltage Limit: 最低电压限制
   * - Max Voltage Limit: 最高电压限制
   * - Hardware Error Status: 电压错误标志
   * - Shutdown: 电压错误保护配置
   *
   * @note 此寄存器为只读，由系统自动更新
   */
  float ReadPresentInputVoltage() {
    uint16_t raw;
    ReadRegField(ControlTable::kPresentInputVoltage, raw);
    return VoltageFromRaw(raw);
  }

  /**
   * @brief 设置当前输入电压 (R)
   * @param present_input_voltage 当前输入电压
   */
  void WritePresentInputVoltage(const uint16_t present_input_voltage) {
    WriteRegField(ControlTable::kPresentInputVoltage, present_input_voltage);
  }

  /**
   * @brief 获取当前温度 (R)
   * @return present_temperature 当前温度（°C）
   *
   * 范围: 0-100 °C
   *
   * 【功能说明】
   * - 显示当前电机温度
   * - 温度传感器位于电机内部
   * - 实时更新，反映当前温度
   * - 温度过高会触发保护
   */
  uint8_t ReadPresentTemperature() {
    uint8_t present_temperature;
    ReadRegField(ControlTable::kPresentTemperature, present_temperature);
    return present_temperature;
  }

  /**
   * @brief 设置当前温度 (R)
   * @param present_temperature 当前温度
   */
  void WritePresentTemperature(const uint8_t present_temperature) {
    WriteRegField(ControlTable::kPresentTemperature, present_temperature);
  }

#pragma endregion  // "状态反馈组"

  /**
   * @brief 恢复 EEPROM 为默认值
   * @return 错误码
   */
  Error RecoveryEeprom() {
    RestoreEeprom(ControlTable::kModelNumber);
    RestoreEeprom(ControlTable::kModelInformation);
    RestoreEeprom(ControlTable::kFirmwareVersion);
    RestoreEeprom(ControlTable::kId);
    RestoreEeprom(ControlTable::kBaudRate);
    RestoreEeprom(ControlTable::kReturnDelayTime);
    RestoreEeprom(ControlTable::kStatusReturnLevel);
    RestoreEeprom(ControlTable::kDriveMode);
    RestoreEeprom(ControlTable::kOperatingMode);
    RestoreEeprom(ControlTable::kShutdown);
    RestoreEeprom(ControlTable::kHomingOffset);
    RestoreEeprom(ControlTable::kMovingThreshold);
    RestoreEeprom(ControlTable::kTemperatureLimit);
    RestoreEeprom(ControlTable::kMaxVoltageLimit);
    RestoreEeprom(ControlTable::kMinVoltageLimit);
    RestoreEeprom(ControlTable::kPwmLimit);
    RestoreEeprom(ControlTable::kCurrentLimit);
    RestoreEeprom(ControlTable::kVelocityLimit);
    RestoreEeprom(ControlTable::kMaxPositionLimit);
    RestoreEeprom(ControlTable::kMinPositionLimit);
    RestoreEeprom(ControlTable::kProtectionTime);
    RestoreEeprom(ControlTable::kVelocityIgain);
    RestoreEeprom(ControlTable::kVelocityPgain);
    RestoreEeprom(ControlTable::kPositionDgain);
    RestoreEeprom(ControlTable::kPositionIgain);
    RestoreEeprom(ControlTable::kPositionPgain);
    RestoreEeprom(ControlTable::kFeedforward2ndGain);
    RestoreEeprom(ControlTable::kFeedforward1stGain);
    RestoreEeprom(ControlTable::kProfileAcceleration);
    RestoreEeprom(ControlTable::kProfileVelocity);
    CHECK(StoreEeprom());
    return Error::kOk;
  }

  /**
   * @brief 加载 EEPROM
   * @return 错误码
   */
  Error LoadEeprom() {
#ifndef EEPROM_DISABLE
    int pos = 0;
    for (uint8_t address = TableBlocks::kEeprom.begin;
         address < TableBlocks::kEeprom.end;
         address++) {
      regs_[address] = EEPROM.read(pos++);
    }
#endif
    return Error::kOk;
  }

  /**
   * @brief 存储 EEPROM
   * @return 错误码
   */
  Error StoreEeprom() {
#ifndef EEPROM_DISABLE
    int pos = 0;
    for (uint8_t address = TableBlocks::kEeprom.begin;
         address < TableBlocks::kEeprom.end;
         address++) {
      EEPROM.update(pos++, regs_[address]);
    }
#endif
    return Error::kOk;
  }

  /**
   * @brief 检测 EEPROM 存档是否为空（未初始化）
   * @return true 若 EEPROM 区全为 0xFF 或 0x00
   *
   * 擦除后的 Flash/EEPROM 通常为 0xFF；EEPROM_DISABLE 时 table_ 初始化为 0。
   */
  bool IsEepromEmpty() const {
    const auto first = regs_[TableBlocks::kEeprom.begin];
    const auto begin = TableBlocks::kEeprom.begin;
    const auto end = TableBlocks::kEeprom.end;
    for (uint8_t address = begin; address < end; address++) {
      if (regs_[address] != first) {
        return false;
      }
    }
    return first == 0x00 || first == 0xFF;
  }

  uint8_t table_[ControlTable::kTotalSize] = {};
};

}  // namespace hortor::servo_slave
