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

namespace hortor::servo_slave {

//==============================================================================
// 单位转换辅助函数
//==============================================================================
namespace {

/**
 * @brief 将原始电压值转换为实际电压值
 * @param raw 原始值 (0-160)
 * @return 实际电压值 (0.0-16.0V)
 *
 * 转换公式: voltage = raw × 0.1V
 * 范围: 0 × 0.1 = 0.0V 到 160 × 0.1 = 16.0V
 */
constexpr float VoltageFromRaw(uint16_t raw) { return raw * 0.1f; }

/**
 * @brief 将实际电压值转换为原始值
 * @param voltage 实际电压值 (0.0-16.0V)
 * @return 原始值 (0-160)
 */
constexpr uint16_t VoltageToRaw(float voltage) {
  return static_cast<uint16_t>(voltage * 10.0f);
}

/**
 * @brief 将原始 PWM 值转换为实际 PWM 百分比
 * @param raw 原始值 (0-885)
 * @return 实际 PWM 百分比 (-100.0% 到 100.0%)
 *
 * 转换公式: percent = raw × 0.113%
 * 范围: 0 × 0.113 = 0% 到 885 × 0.113 = 100%
 *
 * @note 负值表示反向旋转
 */
constexpr float PwmFromRaw(uint16_t raw) { return raw * 0.113f; }

/**
 * @brief 将实际 PWM 百分比转换为原始值
 * @param percent 实际 PWM 百分比 (-100.0% 到 100.0%)
 * @return 原始值 (0-885)
 */
constexpr uint16_t PwmToRaw(float percent) {
  return static_cast<uint16_t>(percent / 0.113f);
}

/**
 * @brief 将原始速度值转换为实际速度值
 * @param raw 原始值 (0-2047)
 * @return 实际速度值 (-468.763 到 468.763 RPM)
 *
 * 转换公式: rpm = raw × 0.229 RPM
 * 范围: 0 × 0.229 = 0 RPM 到 2047 × 0.229 = 468.763 RPM
 *
 * @note 负值表示反向旋转
 */
constexpr float VelocityFromRaw(uint32_t raw) { return raw * 0.229f; }

/**
 * @brief 将实际速度值转换为原始值
 * @param rpm 实际速度值 (-468.763 到 468.763 RPM)
 * @return 原始值 (0-2047)
 */
constexpr uint32_t VelocityToRaw(float rpm) {
  return static_cast<uint32_t>(rpm / 0.229f);
}

/**
 * @brief 将原始加速度值转换为实际加速度值
 * @param raw 原始值 (0-32767)
 * @return 实际加速度值 (0-7,032,024.959 rev/min²)
 *
 * 转换公式: acc = raw × 214.577 rev/min²
 * 范围: 0 × 214.577 = 0 到 32767 × 214.577 = 7,032,024.959 rev/min²
 *
 * @note 仅在 Velocity-based Profile 模式下使用
 */
constexpr float AccelerationFromRaw(uint32_t raw) { return raw * 214.577f; }

/**
 * @brief 将实际加速度值转换为原始值
 * @param acc 实际加速度值 (0-7,032,024.959 rev/min²)
 * @return 原始值 (0-32767)
 */
constexpr uint32_t AccelerationToRaw(float acc) {
  return static_cast<uint32_t>(acc / 214.577f);
}

/**
 * @brief 将原始延迟值转换为实际延迟时间
 * @param raw 原始值 (0-254)
 * @return 实际延迟时间 (0-508 μs)
 *
 * 转换公式: delay = raw × 2μs
 * 范围: 0 × 2 = 0μs 到 254 × 2 = 508μs
 */
constexpr uint16_t DelayFromRaw(uint8_t raw) { return raw * 2; }

/**
 * @brief 将实际延迟时间转换为原始值
 * @param us 实际延迟时间 (0-508 μs)
 * @return 原始值 (0-254)
 */
constexpr uint8_t DelayToRaw(uint16_t us) {
  return static_cast<uint8_t>(us / 2);
}

/**
 * @brief 将原始看门狗值转换为实际时间
 * @param raw 原始值 (0-254)
 * @return 实际看门狗时间 (0-5080 ms)
 *
 * 转换公式: time = raw × 20ms
 * 范围: 0 × 20 = 0ms 到 254 × 20 = 5080ms
 *
 * @note 0 表示禁用看门狗功能
 */
constexpr uint16_t WatchdogFromRaw(uint8_t raw) { return raw * 20; }

/**
 * @brief 将实际看门狗时间转换为原始值
 * @param ms 实际看门狗时间 (0-5080 ms)
 * @return 原始值 (0-254)
 */
constexpr uint8_t WatchdogToRaw(uint16_t ms) {
  return static_cast<uint8_t>(ms / 20);
}

/**
 * @brief 将原始 P 增益值转换为实际增益值
 * @param raw 原始值 (0-16383)
 * @return 实际 P 增益值 (0.0-127.99)
 *
 * 转换公式: gain = raw / 128.0
 * 范围: 0 / 128 = 0.0 到 16383 / 128 = 127.99
 */
constexpr float PidPGainFromRaw(uint16_t raw) { return raw / 128.0f; }

/**
 * @brief 将实际 P 增益值转换为原始值
 * @param gain 实际 P 增益值 (0.0-127.99)
 * @return 原始值 (0-16383)
 */
constexpr uint16_t PidPGainToRaw(float gain) {
  return static_cast<uint16_t>(gain * 128.0f);
}

/**
 * @brief 将原始 I 增益值转换为实际增益值
 * @param raw 原始值 (0-16383)
 * @return 实际 I 增益值 (0.0-0.25)
 *
 * 转换公式: gain = raw / 65536.0
 * 范围: 0 / 65536 = 0.0 到 16383 / 65536 = 0.25
 */
constexpr float PidIGainFromRaw(uint16_t raw) { return raw / 65536.0f; }

/**
 * @brief 将实际 I 增益值转换为原始值
 * @param gain 实际 I 增益值 (0.0-0.25)
 * @return 原始值 (0-16383)
 */
constexpr uint16_t PidIGainToRaw(float gain) {
  return static_cast<uint16_t>(gain * 65536.0f);
}

/**
 * @brief 将原始 D 增益值转换为实际增益值
 * @param raw 原始值 (0-16383)
 * @return 实际 D 增益值 (0.0-1023.9)
 *
 * 转换公式: gain = raw / 16.0
 * 范围: 0 / 16 = 0.0 到 16383 / 16 = 1023.9
 */
constexpr float PidDGainFromRaw(uint16_t raw) { return raw / 16.0f; }

/**
 * @brief 将实际 D 增益值转换为原始值
 * @param gain 实际 D 增益值 (0.0-1023.9)
 * @return 原始值 (0-16383)
 */
constexpr uint16_t PidDGainToRaw(float gain) {
  return static_cast<uint16_t>(gain * 16.0f);
}

/**
 * @brief 将原始前馈增益值转换为实际增益值
 * @param raw 原始值 (0-16383)
 * @return 实际前馈增益值 (0.0-4095.75)
 *
 * 转换公式: gain = raw / 4.0
 * 范围: 0 / 4 = 0.0 到 16383 / 4 = 4095.75
 *
 * @note 实际使用中通常限制在 0.0-127.99 范围内
 */
constexpr float FeedforwardGainFromRaw(uint16_t raw) { return raw / 4.0f; }

/**
 * @brief 将实际前馈增益值转换为原始值
 * @param gain 实际前馈增益值 (0.0-4095.75)
 * @return 原始值 (0-16383)
 */
constexpr uint16_t FeedforwardGainToRaw(float gain) {
  return static_cast<uint16_t>(gain * 4.0f);
}

}  // namespace

/**
 * @brief 伺服从机寄存器映射实现（CRTP模式）
 *
 * 继承自 protocol::RegMap，使用 MMIO 总线实现。
 */
class RegMap : public protocol::RegMap<RegMap, regmap::RegMapMmio> {
 public:
  Error Init();

  //==============================================================================
  // 设备信息区（EEPROM，只读）
  //==============================================================================
  /**
   * @name 设备信息
   * 此区域包含设备识别和版本信息，所有寄存器均为只读，出厂预设。
   * @{
   */

  /**
   * @brief 获取型号编号 (R)
   * @param[out] model_number 型号编号
   * @return Error::kOk 成功
   *
   * 范围: 1190 (XL330-M077)
   *
   * 【功能说明】
   * - 设备型号的唯一标识符
   * - 用于设备识别和兼容性检查
   * - 出厂时预设，不可修改
   *
   * @note 此值用于区分不同的 DYNAMIXEL 型号
   * @see GetModelInformation, GetFirmwareVersion
   */
  Error GetModelNumber(uint16_t& model_number) {
    return ReadRegField(ControlTable::kModelNumber, model_number);
  }

  /**
   * @brief 设置型号编号 (R)
   * @param[in] model_number 型号编号
   * @return Error::kOk 成功
   *
   * @see GetModelNumber 查看详细说明
   */
  Error SetModelNumber(const uint16_t model_number) {
    return WriteRegField(ControlTable::kModelNumber, model_number);
  }

  /**
   * @brief 获取型号信息 (R)
   * @param[out] model_information 型号信息
   * @return Error::kOk 成功
   *
   * 范围: 0-4294967295
   *
   * 【功能说明】
   * - 提供额外的型号相关信息
   * - 可能包含子型号、变体或特殊配置信息
   * - 出厂时预设，不可修改
   *
   * @note 具体含义取决于厂商定义
   * @see GetModelNumber, GetFirmwareVersion
   */
  Error GetModelInformation(uint32_t& model_information) {
    return ReadRegField(ControlTable::kModelInformation, model_information);
  }

  /**
   * @brief 设置型号信息 (R)
   * @param[in] model_information 型号信息
   * @return Error::kOk 成功
   *
   * @see GetModelInformation 查看详细说明
   */
  Error SetModelInformation(const uint32_t model_information) {
    return WriteRegField(ControlTable::kModelInformation, model_information);
  }

  /**
   * @brief 获取固件版本 (R)
   * @param[out] firmware_version 固件版本
   * @return Error::kOk 成功
   *
   * 范围: 0-255
   *
   * 【功能说明】
   * - 固件版本号，用于版本兼容性检查
   * - 固件更新时会改变此值
   * - 出厂时预设，用户不可直接修改
   *
   * @note 用于判断固件功能和兼容性
   * @see GetModelNumber, GetModelInformation
   */
  Error GetFirmwareVersion(uint8_t& firmware_version) {
    return ReadRegField(ControlTable::kFirmwareVersion, firmware_version);
  }

  /**
   * @brief 设置固件版本 (R)
   * @param[in] firmware_version 固件版本
   * @return Error::kOk 成功
   *
   * @see GetFirmwareVersion 查看详细说明
   */
  Error SetFirmwareVersion(const uint8_t firmware_version) {
    return WriteRegField(ControlTable::kFirmwareVersion, firmware_version);
  }

  /** @} */  // 设备信息组结束

  //==============================================================================
  // 通信配置区（EEPROM）
  //==============================================================================
  /**
   * @name 通信配置
   * 配置 DYNAMIXEL 总线通信参数。
   * 修改后需要调用 StoreEeprom() 保存到 Flash，重启后生效。
   * @{
   */

  /**
   * @brief 获取舵机 ID (R/W)
   * @param[out] id 舵机 ID
   * @return Error::kOk 成功
   *
   * 范围: 0-252
   *
   * 【功能说明】
   * - 舵机在总线上的唯一标识符
   * - 用于多舵机系统中的地址识别
   * - 同一总线上不能有重复的 ID
   * - 修改后需要重启生效
   *
   * 【使用场景】
   * - 多舵机系统：为每个舵机分配唯一 ID
   * - 总线管理：避免通信冲突
   * - 设备识别：快速定位特定舵机
   *
   * 【相关寄存器】
   * - Secondary ID: 副 ID 功能
   * - Protocol Version: 协议版本兼容性
   *
   * @warning 修改 ID 后需要使用新 ID 进行通信
   * @warning 同一总线上不应有重复 ID，否则会导致通信冲突
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see SetSecondaryId, GetProtocolVersion
   */
  Error GetId(uint8_t& id) { return ReadRegField(ControlTable::kId, id); }

  /**
   * @brief 设置舵机 ID (R/W)
   * @param[in] id 舵机 ID (0-252)
   * @return Error::kOk 成功，Error::kInvalidParameter ID 超出范围
   *
   * @warning 修改 ID 后需要使用新 ID 进行通信
   * @warning 同一总线上不应有重复 ID，否则会导致通信冲突
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetId 查看详细说明
   */
  Error SetId(const uint8_t id) { return WriteRegField(ControlTable::kId, id); }

  /**
   * @brief 获取波特率索引 (R/W)
   * @param[out] baud_rate 波特率索引
   * @return Error::kOk 成功
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
   * 【使用场景】
   * - 通信速度优化：根据应用需求选择合适波特率
   * - 多设备兼容：确保主机和舵机波特率一致
   * - 性能调优：高速应用使用高波特率
   *
   * 【相关寄存器】
   * - Return Delay Time: 返回延迟设置
   * - Protocol Version: 协议版本
   *
   * @warning 确保主机端波特率与此设置一致，否则无法通信
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see SetReturnDelayTime, GetProtocolVersion
   */
  Error GetBaudRate(uint8_t& baud_rate) {
    return ReadRegField(ControlTable::kBaudRate, baud_rate);
  }

  /**
   * @brief 设置波特率索引 (R/W)
   * @param[in] baud_rate 波特率索引 (0-7)
   * @return Error::kOk 成功
   *
   * @warning 确保主机端波特率与此设置一致，否则无法通信
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetBaudRate 查看详细说明
   */
  Error SetBaudRate(const uint8_t baud_rate) {
    return WriteRegField(ControlTable::kBaudRate, baud_rate);
  }

  /**
   * @brief 获取返回延迟时间 (R/W)
   * @param[out] microseconds 返回延迟时间（μs）
   * @return Error::kOk 成功
   *
   * 范围: 0-508 μs
   *
   * 【功能说明】
   * - 设置舵机响应指令后的延迟时间
   * - 用于多舵机串联时避免总线冲突
   * - 延迟时间 = 原始值 × 2μs
   * - 修改后需要重启生效
   *
   * 【使用场景】
   * - 多舵机串联：避免总线冲突
   * - 通信优化：平衡响应速度和稳定性
   * - 长距离通信：增加延迟提高可靠性
   *
   * 【典型参数】
   * - 无延迟：0 μs（最快响应）
   * - 默认延迟：500 μs（平衡性能）
   * - 最大延迟：508 μs（最稳定）
   *
   * 【相关寄存器】
   * - Baud Rate: 通信速度设置
   * - Status Return Level: 状态返回级别
   *
   * @note 在多舵机串联时，适当的延迟可避免总线冲突
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see SetBaudRate, GetStatusReturnLevel
   */
  Error GetReturnDelayTime(uint16_t& microseconds) {
    uint8_t raw;
    CHECK(ReadRegField(ControlTable::kReturnDelayTime, raw));
    microseconds = DelayFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置返回延迟时间 (R/W)
   * @param[in] microseconds 返回延迟时间（μs）
   * @return Error::kOk 成功
   *
   * @note 在多舵机串联时，适当的延迟可避免总线冲突
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetReturnDelayTime 查看详细说明
   */
  Error SetReturnDelayTime(const uint16_t microseconds) {
    return WriteRegField(ControlTable::kReturnDelayTime,
                         DelayToRaw(microseconds));
  }

  /**
   * @brief 获取驱动模式 (R/W)
   * @param[out] drive_mode 驱动模式位域
   * @return Error::kOk 成功
   *
   * 范围: 0-255 (8位位域)
   *
   * 【功能说明】
   * - 配置舵机的基本驱动行为
   * - 位域寄存器，每个 bit 控制不同功能
   * - 修改后需要重启生效
   *
  * 【位域定义】
  * | Bit | 名称                        | 说明 |
  * |-----|----------------------------|----------------------------------------------------------------------|
  * | 0   | Reverse Mode               | 0: Normal, 1: Reverse |
  * | 1   | -                          | 保留（未使用） |
  * | 2   | Profile Configuration      | 0: Velocity-based, 1: Time-based |
  * | 3   | Torque On by Goal Update   | 0: 仅在 Torque Enable(64)=1
  时执行命令；1: 忽略 Torque Enable 状态，
  * |     |                            |    若 Torque Enable(64)=0
  且收到命令则自动置 1 并执行               |
  * | 4-7 | -                          | 保留（未使用） |
   *
   * 【反转模式 (Bit 0)】
   * - [0] Normal Mode: CCW(正), CW(反)
   * - [1] Reverse Mode: CCW(反), CW(正)
   *
  * 【Profile 配置 (Bit 2)】
   * - 0: Velocity-based Profile - 使用速度和加速度参数
   * - 1: Time-based Profile - 使用时间参数（毫秒）

  * 【扭矩联动 (Bit 3: Torque On by Goal Update)】
  * - [0] 标准：仅当 Torque Enable(64)=1 时执行目标命令
  * - [1] 联动：忽略当前 Torque Enable(64) 值；若为 0 且接收到命令，
  *             将自动置 Torque Enable(64)=1 并执行命令
   *
  * 【使用场景】
   * - 机械适配：反转模式适应安装方向
  * - 轨迹控制：选择 Profile 类型
  * - 扭矩管理：按需启用 Bit3 以简化上层控制
  * - 系统集成：统一多舵机行为
   *
   * 【相关寄存器】
   * - Profile Velocity: Velocity-based 模式使用
  * - Profile Acceleration: 两种模式都使用（Time-based 模式为加速时间）
   * - Operating Mode: 工作模式设置
  * - Torque Enable: 扭矩使能控制（与 Bit3 行为相关）
   *
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see SetProfileVelocity, SetProfileAcceleration, GetOperatingMode
   */
  Error GetDriveMode(uint8_t& drive_mode) {
    return ReadRegField(ControlTable::kDriveMode, drive_mode);
  }

  /**
   * @brief 设置驱动模式 (R/W)
   * @param[in] drive_mode 驱动模式位域
   * @return Error::kOk 成功
   *
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetDriveMode 查看位域定义
   */
  Error SetDriveMode(const uint8_t drive_mode) {
    return WriteRegField(ControlTable::kDriveMode, drive_mode);
  }

  /**
   * @brief 获取工作模式 (R/W)
   * @param[out] operating_mode 工作模式
   * @return Error::kOk 成功
   *
   * 范围: 0, 1, 3-5, 16
   *
   * 【功能说明】
   * - 设置舵机的工作模式，决定控制目标和行为特性
   * - 不同模式使用不同的目标寄存器和控制策略
   * - 修改前必须先禁用力矩（Torque Enable = 0）
   * - 修改后需要重启生效
   *
   * 【模式对比表】
   * | 模式 | 控制目标 | Profile | 位置限位 | 多圈支持 | 典型应用 |
   * |------|---------|---------|---------|---------|---------|
   * | 0: Current | 电流/力矩 | ❌ | ❌ | ❌ | 力控、柔顺 |
   * | 1: Velocity | 速度 | ⚠️ 仅加速度 | ❌ | ✅ | 轮子、传送带 |
   * | 3: Position | 位置(单圈) | ✅ | ✅ | ❌ | 关节、单圈定位 |
   * | 4: Extended | 位置(多圈) | ✅ | ❌ | ✅ | 多圈、旋转计数 |
   * | 5: Current-based | 位置+电流限制 | ✅ | ✅ | ❌ | 抓取、防撞 |
   * | 16: PWM | PWM占空比 | ❌ | ❌ | ❌ | 开环、调试 |
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
   * 1. SetTorqueEnable(0) - 必须先关闭力矩
   * 2. SetOperatingMode(new_mode) - 设置新模式
   * 3. 设置该模式的目标值（Goal Position/Velocity/Current/PWM）
   * 4. SetTorqueEnable(1) - 重新使能力矩
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
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see SetGoalPosition, SetGoalVelocity, SetGoalCurrent, SetGoalPwm,
   * SetTorqueEnable
   */
  Error GetOperatingMode(uint8_t& operating_mode) {
    return ReadRegField(ControlTable::kOperatingMode, operating_mode);
  }

  /**
   * @brief 设置工作模式 (R/W)
   * @param[in] operating_mode 工作模式 (0, 1, 3-5, 16)
   * @return Error::kOk 成功
   *
   * @warning 更改工作模式前必须先 Torque Disable
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetOperatingMode 查看各模式详细说明
   */
  Error SetOperatingMode(const uint8_t operating_mode) {
    return WriteRegField(ControlTable::kOperatingMode, operating_mode);
  }

  /**
   * @brief 获取副ID (R/W)
   * @param[out] secondary_id 副ID
   * @return Error::kOk 成功
   *
   * 范围: 0-252
   *
   * 【功能说明】
   * - 副ID 用于额外的舵机识别功能
   * - 0: 禁用副ID功能
   * - 非0: 启用副ID功能
   * - 修改后需要重启生效
   *
   * 【使用场景】
   * - 分组控制：多个舵机使用相同副ID
   * - 广播指令：向特定副ID组发送指令
   * - 系统管理：简化多舵机系统控制
   *
   * 【相关寄存器】
   * - ID: 主ID设置
   * - Protocol Version: 协议版本
   *
   * @note 副ID 功能的具体用途取决于应用实现
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see SetId, GetProtocolVersion
   */
  Error GetSecondaryId(uint8_t& secondary_id) {
    return ReadRegField(ControlTable::kSecondaryId, secondary_id);
  }

  /**
   * @brief 设置副ID (R/W)
   * @param[in] secondary_id 副ID (0-252)
   * @return Error::kOk 成功
   *
   * @note 副ID 功能的具体用途取决于应用实现
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetSecondaryId 查看详细说明
   */
  Error SetSecondaryId(const uint8_t secondary_id) {
    return WriteRegField(ControlTable::kSecondaryId, secondary_id);
  }

  /**
   * @brief 获取协议版本 (R/W)
   * @param[out] protocol_version 协议版本
   * @return Error::kOk 成功
   *
   * 范围: 1-2
   *
   * 【功能说明】
   * - 设置 DYNAMIXEL 通信协议版本
   * - 决定指令格式和功能支持
   * - 修改后需要重启生效
   *
   * 【协议版本】
   * - 1: DYNAMIXEL Protocol 1.0
   * - 2: DYNAMIXEL Protocol 2.0（默认）
   *
   * 【使用场景】
   * - 兼容性：与旧版本设备通信
   * - 功能选择：Protocol 2.0 支持更多功能
   * - 系统集成：统一协议版本
   *
   * 【相关寄存器】
   * - ID: 设备ID设置
   * - Baud Rate: 通信速度
   *
   * @note XL330 支持 Protocol 2.0
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see SetId, GetBaudRate
   */
  Error GetProtocolVersion(uint8_t& protocol_version) {
    return ReadRegField(ControlTable::kProtocolVersion, protocol_version);
  }

  /**
   * @brief 设置协议版本 (R/W)
   * @param[in] protocol_version 协议版本 (1-2)
   * @return Error::kOk 成功
   *
   * @note XL330 支持 Protocol 2.0
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetProtocolVersion 查看详细说明
   */
  Error SetProtocolVersion(const uint8_t protocol_version) {
    return WriteRegField(ControlTable::kProtocolVersion, protocol_version);
  }

  /** @} */  // 通信配置组结束

  //==============================================================================
  // 限位与保护设置区（EEPROM）
  //==============================================================================
  /**
   * @name 限位与保护设置
   * 配置运动限位、温度/电压/电流保护阈值，防止舵机损坏。
   * @{
   */

  /**
   * @brief 获取归零偏移 (R/W)
   * @param[out] homing_offset 归零偏移（pulse）
   * @return Error::kOk 成功
   *
   * 范围: -1,048,575 ~ 1,048,575 pulse
   *
   * 【功能说明】
   * - 归零偏移用于定义舵机的零位参考点
   * - 影响所有位置指令的计算
   * - 实际目标位置 = Goal Position + Homing Offset
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
   * - Present Position: 当前位置显示不受影响
   * - Min/Max Position Limit: 限位范围不受影响
   * - Operating Mode: 不同模式下的行为差异
   *
   * @note 偏移值会加到所有位置指令上，包括 Goal Position
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see SetGoalPosition, GetPresentPosition, SetOperatingMode
   */
  Error GetHomingOffset(int32_t& homing_offset) {
    uint32_t raw;
    CHECK(ReadRegField(ControlTable::kHomingOffset, raw));
    homing_offset = static_cast<int32_t>(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置归零偏移 (R/W)
   * @param[in] homing_offset 归零偏移（pulse）
   * @return Error::kOk 成功
   *
   * @note 偏移值会加到所有位置指令上，包括 Goal Position
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetHomingOffset 查看详细说明
   */
  Error SetHomingOffset(const int32_t homing_offset) {
    return WriteRegField(ControlTable::kHomingOffset,
                         static_cast<uint32_t>(homing_offset));
  }

  /**
   * @brief 获取运动阈值 (R/W)
   * @param[out] rpm 运动阈值（RPM）
   * @return Error::kOk 成功
   *
   * 范围: 0-468.763 RPM
   *
   * 【功能说明】
   * - 运动阈值用于判断舵机是否在运动
   * - 影响 Moving Status(123) 的更新
   * - 当速度 ≥ 此阈值时，Moving 置 1
   * - 当速度 < 此阈值时，Moving 清 0
   * - 修改后需要重启生效
   *
   * 【使用场景】
   * - 运动检测：判断舵机是否在运动
   * - 到达检测：判断是否到达目标位置
   * - 状态监控：避免微小振动导致的误判
   * - 节能控制：静止时降低控制频率
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
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetMovingStatus, GetPresentVelocity, SetMovingThreshold
   */
  Error GetMovingThreshold(float& rpm) {
    uint32_t raw;
    CHECK(ReadRegField(ControlTable::kMovingThreshold, raw));
    rpm = VelocityFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置运动阈值 (R/W)
   * @param[in] rpm 运动阈值（RPM）
   * @return Error::kOk 成功
   *
   * @note 用于检测运动状态，避免微小振动导致的误判
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetMovingThreshold 查看详细说明
   */
  Error SetMovingThreshold(const float rpm) {
    return WriteRegField(ControlTable::kMovingThreshold, VelocityToRaw(rpm));
  }

  /**
   * @brief 获取温度上限 (R/W)
   * @param[out] temperature_limit 温度上限（°C）
   * @return Error::kOk 成功
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
   * 【使用场景】
   * - 过热保护：防止电机因过热损坏
   * - 环境适应：根据工作环境调整温度限制
   * - 安全控制：确保系统安全运行
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
   *
   * @warning 温度过高会损坏电机，建议不超过 80°C
   * @note XL330 工作温度范围: -5°C ~ 72°C
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetPresentTemperature, GetShutdown, GetHardwareErrorStatus
   */
  Error GetTemperatureLimit(uint8_t& temperature_limit) {
    return ReadRegField(ControlTable::kTemperatureLimit, temperature_limit);
  }

  /**
   * @brief 设置温度上限 (R/W)
   * @param[in] temperature_limit 温度上限（°C）
   * @return Error::kOk 成功
   *
   * @warning 温度过高会损坏电机，建议不超过 80°C
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetTemperatureLimit 查看详细说明
   */
  Error SetTemperatureLimit(const uint8_t temperature_limit) {
    return WriteRegField(ControlTable::kTemperatureLimit, temperature_limit);
  }

  /**
   * @brief 获取最高电压限制 (R/W)
   * @param[out] voltage 最高电压限制（V）
   * @return Error::kOk 成功
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
   * 【使用场景】
   * - 过压保护：防止电源电压过高损坏设备
   * - 电源管理：适应不同电源规格
   * - 安全控制：确保电源安全
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
   *
   * @note XL330 工作电压范围: 5.0V ~ 12.0V
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetPresentInputVoltage, GetMinVoltageLimit, GetShutdown
   */
  Error GetMaxVoltageLimit(float& voltage) {
    uint16_t raw;
    CHECK(ReadRegField(ControlTable::kMaxVoltageLimit, raw));
    voltage = VoltageFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置最高电压限制 (R/W)
   * @param[in] voltage 最高电压限制（V）
   * @return Error::kOk 成功
   *
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetMaxVoltageLimit 查看详细说明
   */
  Error SetMaxVoltageLimit(const float voltage) {
    return WriteRegField(ControlTable::kMaxVoltageLimit, VoltageToRaw(voltage));
  }

  /**
   * @brief 获取最低电压限制 (R/W)
   * @param[out] voltage 最低电压限制（V）
   * @return Error::kOk 成功
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
   * 【使用场景】
   * - 欠压保护：防止电源电压过低影响性能
   * - 电源管理：确保电源质量
   * - 安全控制：防止低电压运行
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
   *
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetMaxVoltageLimit 查看详细说明
   */
  Error GetMinVoltageLimit(float& voltage) {
    uint16_t raw;
    CHECK(ReadRegField(ControlTable::kMinVoltageLimit, raw));
    voltage = VoltageFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置最低电压限制 (R/W)
   * @param[in] voltage 最低电压限制（V）
   * @return Error::kOk 成功
   *
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetMinVoltageLimit 查看详细说明
   */
  Error SetMinVoltageLimit(const float voltage) {
    return WriteRegField(ControlTable::kMinVoltageLimit, VoltageToRaw(voltage));
  }

  /**
   * @brief 获取 PWM 上限 (R/W)
   * @param[out] percent PWM 上限（%）
   * @return Error::kOk 成功
   *
   * 范围: 0-100%
   *
   * 【功能说明】
   * - 限制电机驱动器的 PWM 占空比
   * - 间接限制输出力矩和电流
   * - 所有控制模式都受此限制
   * - 修改后需要重启生效
   *
   * 【使用场景】
   * - 力矩限制：通过限制PWM间接限制力矩
   * - 安全控制：防止过大的输出功率
   * - 节能管理：限制功耗
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
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see SetGoalPwm, GetPresentPwm, SetCurrentLimit
   */
  Error GetPwmLimit(float& percent) {
    uint16_t raw;
    CHECK(ReadRegField(ControlTable::kPwmLimit, raw));
    percent = PwmFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置 PWM 上限 (R/W)
   * @param[in] percent PWM 上限（%）
   * @return Error::kOk 成功
   *
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetPwmLimit 查看详细说明
   */
  Error SetPwmLimit(const float percent) {
    return WriteRegField(ControlTable::kPwmLimit, PwmToRaw(percent));
  }

  /**
   * @brief 获取电流上限 (R/W)
   * @param[out] current_limit 电流上限（mA）
   * @return Error::kOk 成功
   *
   * 范围: 0-1193 mA
   *
   * 【功能说明】
   * - 限制电机输出电流
   * - 保护电机和驱动电路
   * - Current-based Position Control Mode 的最大电流
   * - 修改后需要重启生效
   *
   * 【使用场景】
   * - 过流保护：防止电流过大损坏设备
   * - 力矩控制：通过电流限制控制输出力矩
   * - 安全运行：确保系统安全
   *
   * 【典型参数】
   * - 轻载应用: 200-500 mA
   * - 一般应用: 500-800 mA
   * - 重载应用: 800-1193 mA
   *
   * 【相关寄存器】
   * - Goal Current: 电流控制模式的目标值
   * - Present Current: 实际电流输出
   * - PWM Limit: PWM限制
   *
   * @warning 长时间大电流运行会导致过热
   * @note Present Current 不会超过此值
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see SetGoalCurrent, GetPresentCurrent, SetPwmLimit
   */
  Error GetCurrentLimit(uint16_t& current_limit) {
    return ReadRegField(ControlTable::kCurrentLimit, current_limit);
  }

  /**
   * @brief 设置电流上限 (R/W)
   * @param[in] current_limit 电流上限（mA）
   * @return Error::kOk 成功
   *
   * @warning 长时间大电流运行会导致过热
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetCurrentLimit 查看详细说明
   */
  Error SetCurrentLimit(const uint16_t current_limit) {
    return WriteRegField(ControlTable::kCurrentLimit, current_limit);
  }

  /**
   * @brief 获取速度上限 (R/W)
   * @param[out] rpm 速度上限（RPM）
   * @return Error::kOk 成功
   *
   * 范围: 0-468.763 RPM
   *
   * 【功能说明】
   * - 限制舵机运动的最大速度
   * - 保护机械结构免受高速冲击
   * - 影响所有控制模式的速度限制
   * - 修改后需要重启生效
   *
   * 【使用场景】
   * - 机械保护：限制最大速度，防止机械损坏
   * - 安全控制：在不确定负载时设置保守速度
   * - 性能优化：根据应用需求调整速度上限
   * - 多模式兼容：统一的速度限制策略
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
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see SetProfileVelocity, SetGoalVelocity, GetPresentVelocity
   */
  Error GetVelocityLimit(float& rpm) {
    uint32_t raw;
    CHECK(ReadRegField(ControlTable::kVelocityLimit, raw));
    rpm = VelocityFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置速度上限 (R/W)
   * @param[in] rpm 速度上限（RPM）
   * @return Error::kOk 成功
   *
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetVelocityLimit 查看详细说明
   */
  Error SetVelocityLimit(const float rpm) {
    return WriteRegField(ControlTable::kVelocityLimit, VelocityToRaw(rpm));
  }

  /**
   * @brief 获取位置上限 (R/W)
   * @param[out] max_position_limit 位置上限（pulse）
   * @return Error::kOk 成功
   *
   * 范围: 0-4095 pulse
   *
   * 【功能说明】
   * - 限制 Position Control Mode 下的最大位置
   * - Extended Position Control Mode 不受此限制
   * - Goal Position 会被限制在 [Min Position Limit, Max Position Limit] 范围内
   * - 修改后需要重启生效
   *
   * 【使用场景】
   * - 机械限位：防止机械结构损坏
   * - 安全控制：限制运动范围
   * - 系统保护：避免超出安全范围
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
   * @warning 如果设置不当，可能导致舵机无法移动
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetMinPositionLimit, SetGoalPosition, GetOperatingMode
   */
  Error GetMaxPositionLimit(uint32_t& max_position_limit) {
    return ReadRegField(ControlTable::kMaxPositionLimit, max_position_limit);
  }

  /**
   * @brief 设置位置上限 (R/W)
   * @param[in] max_position_limit 位置上限（pulse）
   * @return Error::kOk 成功
   *
   * @warning 如果设置不当，可能导致舵机无法移动
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetMaxPositionLimit 查看详细说明
   */
  Error SetMaxPositionLimit(const uint32_t max_position_limit) {
    return WriteRegField(ControlTable::kMaxPositionLimit, max_position_limit);
  }

  /**
   * @brief 获取位置下限 (R/W)
   * @param[out] min_position_limit 位置下限（pulse）
   * @return Error::kOk 成功
   *
   * 范围: 0-4095 pulse
   *
   * 【功能说明】
   * - 限制 Position Control Mode 下的最小位置
   * - Extended Position Control Mode 不受此限制
   * - Goal Position 会被限制在 [Min Position Limit, Max Position Limit] 范围内
   * - 修改后需要重启生效
   *
   * 【使用场景】
   * - 机械限位：防止机械结构损坏
   * - 安全控制：限制运动范围
   * - 系统保护：避免超出安全范围
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
   * @warning 如果设置不当，可能导致舵机无法移动
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetMaxPositionLimit, SetGoalPosition, GetOperatingMode
   */
  Error GetMinPositionLimit(uint32_t& min_position_limit) {
    return ReadRegField(ControlTable::kMinPositionLimit, min_position_limit);
  }

  /**
   * @brief 设置位置下限 (R/W)
   * @param[in] min_position_limit 位置下限（pulse）
   * @return Error::kOk 成功
   *
   * @warning 如果设置不当，可能导致舵机无法移动
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetMinPositionLimit 查看详细说明
   */
  Error SetMinPositionLimit(const uint32_t min_position_limit) {
    return WriteRegField(ControlTable::kMinPositionLimit, min_position_limit);
  }

  /**
   * @brief 获取关断条件 (R/W)
   * @param[out] shutdown 关断条件位域
   * @return Error::kOk 成功
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
   * | 2   | Overheating Error      | 温度超过上限                  |
   * | 3   | Motor Encoder Error    | 编码器故障                    |
   * | 4   | Electrical Shock Error | 电气冲击                      |
   * | 5   | Overload Error         | 过载（电流持续超限）          |
   *
   * 【使用场景】
   * - 安全保护：自动关断防止损坏
   * - 故障处理：快速响应错误状态
   * - 系统保护：确保系统安全运行
   *
   * 【典型设置】
   * - 全保护：0b00111111 (63) - 所有错误都关断
   * - 标准保护：0b00110100 (52) - 过热+电气冲击+过载
   * - 最小保护：0b00100000 (32) - 仅过热保护
   *
   * 【错误处理流程】
   * 1. 检测错误：调用 GetHardwareErrorStatus() 获取错误状态
   * 2. 分析错误类型：检查各个错误位（Bit 0,2,3,4,5）
   * 3. 排除故障原因：根据错误类型采取相应措施
   * 4. 清除错误：调用 SetHardwareErrorStatus(0) 清除错误状态
   * 5. 重新使能：调用 SetTorqueEnable(1) 重新使能力矩
   *
   * 【相关寄存器】
   * - Hardware Error Status: 错误状态记录
   * - Torque Enable: 关断后需要重新使能
   * - Temperature Limit: 温度保护阈值
   * - Voltage Limits: 电压保护阈值
   *
   * @note 关断后需要手动清除错误并重新使能力矩
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetHardwareErrorStatus, SetTorqueEnable, GetTemperatureLimit
   */
  Error GetShutdown(uint8_t& shutdown) {
    return ReadRegField(ControlTable::kShutdown, shutdown);
  }

  /**
   * @brief 设置关断条件 (R/W)
   * @param[in] shutdown 关断条件位域
   * @return Error::kOk 成功
   *
   * @note 关断后需要手动清除错误并重新使能力矩
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @see GetShutdown 查看位域定义
   */
  Error SetShutdown(const uint8_t shutdown) {
    return WriteRegField(ControlTable::kShutdown, shutdown);
  }

  /** @} */  // 限位与保护设置组结束

  //==============================================================================
  // 使能与状态区（RAM）
  //==============================================================================
  /**
   * @name 使能与状态
   * @{
   */

  /**
   * @brief 获取力矩使能状态 (R/W)
   * @param[out] torque_enable 力矩使能状态
   * @return Error::kOk 成功
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
   * 【使用场景】
   * - 安全控制：紧急情况下禁用力矩
   * - 模式切换：切换工作模式前必须禁用
   * - 系统初始化：上电后需要手动使能
   * - 错误恢复：清除错误后重新使能
   *
   * 【操作流程】
   * 1. 使能力矩：SetTorqueEnable(1) - 电机会立即执行控制指令
   * 2. 禁用力矩：SetTorqueEnable(0) - 电机可以自由转动
   * 3. 模式切换：必须先禁用，再切换模式，最后重新使能
   *
   * 【相关寄存器】
   * - Operating Mode: 切换模式前必须禁用
   * - Hardware Error Status: 错误时会自动禁用
   * - Shutdown: 关断条件设置
   *
   * @warning 使能力矩后电机会立即执行控制指令，注意安全
   * @note 禁用力矩时，Present Position 等反馈值仍然有效
   * @see GetOperatingMode, GetHardwareErrorStatus, SetShutdown
   */
  Error GetTorqueEnable(uint8_t& torque_enable) {
    return ReadRegField(ControlTable::kTorqueEnable, torque_enable);
  }

  /**
   * @brief 设置力矩使能 (R/W)
   * @param[in] torque_enable 力矩使能 (0: 禁用, 1: 使能)
   * @return Error::kOk 成功
   *
   * @warning 使能力矩后电机会立即执行控制指令，注意安全
   * @see GetTorqueEnable 查看详细说明
   */
  Error SetTorqueEnable(const uint8_t torque_enable) {
    return WriteRegField(ControlTable::kTorqueEnable, torque_enable);
  }

  /**
   * @brief 获取 LED 状态 (R/W)
   * @param[out] dxl_led LED 状态
   * @return Error::kOk 成功
   *
   * 范围: 0-1
   *
   * 【功能说明】
   * - 控制舵机上的 LED 指示灯
   * - 0: LED 关闭
   * - 1: LED 开启
   * - 用于状态指示和调试
   *
   * 【使用场景】
   * - 状态指示：显示舵机工作状态
   * - 调试辅助：快速识别舵机
   * - 系统监控：视觉反馈
   *
   * @note 可用于状态指示和调试
   * @see SetDxlLed
   */
  Error GetDxlLed(uint8_t& dxl_led) {
    return ReadRegField(ControlTable::kDxlLed, dxl_led);
  }

  /**
   * @brief 设置 LED 开关 (R/W)
   * @param[in] dxl_led LED 状态 (0: 关, 1: 开)
   * @return Error::kOk 成功
   *
   * @note 可用于状态指示和调试
   * @see GetDxlLed 查看详细说明
   */
  Error SetDxlLed(const uint8_t dxl_led) {
    return WriteRegField(ControlTable::kDxlLed, dxl_led);
  }

  /**
   * @brief 获取状态返回级别 (R/W)
   * @param[out] status_return_level 状态返回级别
   * @return Error::kOk 成功
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
   * 【使用场景】
   * - 通信优化：降低返回级别减少总线流量
   * - 调试模式：提高返回级别便于调试
   * - 系统集成：根据应用需求调整
   *
   * @note 降低返回级别可以减少总线流量，提高通信效率
   * @see GetStatusReturnLevel
   */
  Error GetStatusReturnLevel(uint8_t& status_return_level) {
    return ReadRegField(ControlTable::kStatusReturnLevel, status_return_level);
  }

  /**
   * @brief 设置状态返回级别 (R/W)
   * @param[in] status_return_level 状态返回级别 (0-2)
   * @return Error::kOk 成功
   *
   * @note 降低返回级别可以减少总线流量，提高通信效率
   * @see GetStatusReturnLevel 查看详细说明
   */
  Error SetStatusReturnLevel(const uint8_t status_return_level) {
    return WriteRegField(ControlTable::kStatusReturnLevel, status_return_level);
  }

  /**
   * @brief 获取已注册指令 (R)
   * @param[out] registered_instruction 已注册指令
   * @return Error::kOk 成功
   *
   * 范围: 0-255
   *
   * 【功能说明】
   * - 显示当前已注册的指令类型
   * - 用于指令状态跟踪和调试
   * - 只读寄存器，由系统自动更新
   *
   * 【使用场景】
   * - 指令跟踪：监控指令执行状态
   * - 调试辅助：了解当前指令类型
   * - 系统监控：指令执行分析
   *
   * @note 此寄存器为只读，由系统自动更新
   * @see SetRegisteredInstruction
   */
  Error GetRegisteredInstruction(uint8_t& registered_instruction) {
    return ReadRegField(ControlTable::kRegisteredInstruction,
                        registered_instruction);
  }

  /**
   * @brief 设置已注册指令 (R)
   * @param[in] registered_instruction 已注册指令
   * @return Error::kOk 成功
   *
   * @see GetRegisteredInstruction 查看详细说明
   */
  Error SetRegisteredInstruction(const uint8_t registered_instruction) {
    return WriteRegField(ControlTable::kRegisteredInstruction,
                         registered_instruction);
  }

  /**
   * @brief 获取硬件错误状态 (R)
   * @param[out] hardware_error_status 硬件错误状态位域
   * @return Error::kOk 成功
   *
   * 范围: 0-255 (8位位域)
   *
   * 【功能说明】
   * - 读取舵机硬件错误状态，提供详细的故障诊断信息
   * - 错误状态会持续存在，直到手动清除
   * - 位域寄存器，每个 bit 表示一种错误类型
   *
   * 【硬件错误位域】
   * | Bit | 错误类型                 | 说明                            |
   * |-----|------------------------|-------------------------------|
   * | 0   | Input Voltage Error    | 输入电压超出范围                 |
   * | 2   | Overheating Error      | 温度超过上限                    |
   * | 3   | Motor Encoder Error    | 编码器故障                    |
   * | 4   | Electrical Shock Error | 电气冲击                      |
   * | 5   | Overload Error         | 过载                          |
   *
   * 【错误处理标准流程】
   * 1. 检测错误：调用 GetHardwareErrorStatus() 获取错误状态
   * 2. 分析错误类型：检查各个错误位（Bit 0,2,3,4,5）
   * 3. 排除故障原因：根据错误类型采取相应措施
   * 4. 清除错误：调用 SetHardwareErrorStatus(0) 清除错误状态
   * 5. 重新使能：调用 SetTorqueEnable(1) 重新使能力矩
   *
   * 【常见错误及解决方法】
   *
   * Input Voltage Error (Bit 0):
   *   原因：电源电压超出 [Min Voltage Limit, Max Voltage Limit]
   *   解决：1) 检查电源稳定性 2) 调整电压限制范围
   *
   * Overheating Error (Bit 2):
   *   原因：温度超过 Temperature Limit
   *   解决：1) 降低负载 2) 改善散热 3) 降低速度/电流限制
   *
   * Motor Encoder Error (Bit 3):
   *   原因：磁编码器故障或磁铁脱落
   *   解决：1) 检查磁铁安装 2) 硬件故障需返修
   *
   * Electrical Shock Error (Bit 4):
   *   原因：电路瞬态冲击或电磁干扰
   *   解决：1) 改善布线 2) 添加滤波电容
   *
   * Overload Error (Bit 5):
   *   原因：负载过大或堵转，电流持续超限
   *   解决：1) 减小负载 2) 增加 Current Limit 3) 检查机械卡死
   *
   * 【相关寄存器】
   * - Shutdown: 决定哪些错误会自动关断力矩
   * - Torque Enable: 错误清除后需要重新使能
   * - Present Temperature: 监控温度状态
   * - Present Input Voltage: 监控电压状态
   * - Present Current: 监控电流状态
   *
   * @note 如果 Shutdown 对应位启用，发生错误会自动关断力矩
   * @see GetShutdown, SetTorqueEnable, SetHardwareErrorStatus
   */
  Error GetHardwareErrorStatus(uint8_t& hardware_error_status) {
    return ReadRegField(ControlTable::kHardwareErrorStatus,
                        hardware_error_status);
  }

  /**
   * @brief 设置硬件错误状态 (R)
   * @param[in] hardware_error_status 硬件错误状态位域
   * @return Error::kOk 成功
   *
   * @note 写入 0 可清除所有错误状态
   * @see GetHardwareErrorStatus 查看位域定义
   */
  Error SetHardwareErrorStatus(const uint8_t hardware_error_status) {
    return WriteRegField(ControlTable::kHardwareErrorStatus,
                         hardware_error_status);
  }

  /** @} */  // 使能与状态组结束

  //==============================================================================
  // PID 和前馈控制参数区（RAM）
  //==============================================================================
  /**
   * @name PID 和前馈控制参数
   * 配置位置和速度控制器的 PID 增益和前馈增益。
   * 参数范围: 0-16383，增益值越大响应越快但可能产生振荡。
   *
   * XL330 控制结构：
   *   Goal Position → [Position PID] → Goal Velocity → [Velocity PI] → Goal
   * Current → [Current PI] → PWM
   * @{
   */

  /**
   * @brief 获取速度环 I 增益 (R/W)
   * @param[out] velocity_igain I 增益
   * @return Error::kOk 成功
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
   * @warning I 增益过大会导致系统不稳定
   * @note 仅在 Velocity Control Mode 和 Position Control Mode 有效
   * @see SetVelocityPgain, GetPositionPgain, GetOperatingMode
   */
  Error GetVelocityIgain(float& velocity_igain) {
    uint16_t raw;
    CHECK(ReadRegField(ControlTable::kVelocityIgain, raw));
    velocity_igain = PidIGainFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置速度环 I 增益 (R/W)
   * @param[in] velocity_igain I 增益
   * @return Error::kOk 成功
   *
   * @warning I 增益过大会导致系统不稳定
   * @see GetVelocityIgain 查看详细说明
   */
  Error SetVelocityIgain(const float velocity_igain) {
    return WriteRegField(ControlTable::kVelocityIgain,
                         PidIGainToRaw(velocity_igain));
  }

  /**
   * @brief 获取速度环 P 增益 (R/W)
   * @param[out] velocity_pgain P 增益
   * @return Error::kOk 成功
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
   * 【典型参数】
   * - 保守设置: 1-10（稳定优先）
   * - 标准设置: 10-50（平衡性能）
   * - 高响应设置: 50-127（响应优先）
   *
   * 【相关寄存器】
   * - Velocity I Gain: 速度环积分增益
   * - Position PID: 外环位置控制器
   * - Operating Mode: 控制模式设置
   *
   * @note 仅在 Velocity Control Mode 和 Position Control Mode 有效
   * @see SetVelocityIgain, GetPositionPgain, GetOperatingMode
   */
  Error GetVelocityPgain(float& velocity_pgain) {
    uint16_t raw;
    CHECK(ReadRegField(ControlTable::kVelocityPgain, raw));
    velocity_pgain = PidPGainFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置速度环 P 增益 (R/W)
   * @param[in] velocity_pgain P 增益
   * @return Error::kOk 成功
   *
   * @see GetVelocityPgain 查看详细说明
   */
  Error SetVelocityPgain(const float velocity_pgain) {
    return WriteRegField(ControlTable::kVelocityPgain,
                         PidPGainToRaw(velocity_pgain));
  }

  /**
   * @brief 获取位置环 D 增益 (R/W)
   * @param[out] position_dgain D 增益
   * @return Error::kOk 成功
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
   * @see SetPositionPgain, SetPositionIgain, GetOperatingMode
   */
  Error GetPositionDgain(float& position_dgain) {
    uint16_t raw;
    CHECK(ReadRegField(ControlTable::kPositionDgain, raw));
    position_dgain = PidDGainFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置位置环 D 增益 (R/W)
   * @param[in] position_dgain D 增益
   * @return Error::kOk 成功
   *
   * @warning D 增益过大会放大高频噪声
   * @see GetPositionDgain 查看详细说明
   */
  Error SetPositionDgain(const float position_dgain) {
    return WriteRegField(ControlTable::kPositionDgain,
                         PidDGainToRaw(position_dgain));
  }

  /**
   * @brief 获取位置环 I 增益 (R/W)
   * @param[out] position_igain I 增益
   * @return Error::kOk 成功
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
   * @see SetPositionPgain, SetPositionDgain, GetOperatingMode
   */
  Error GetPositionIgain(float& position_igain) {
    uint16_t raw;
    CHECK(ReadRegField(ControlTable::kPositionIgain, raw));
    position_igain = PidIGainFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置位置环 I 增益 (R/W)
   * @param[in] position_igain I 增益
   * @return Error::kOk 成功
   *
   * @warning I 增益过大会导致系统不稳定
   * @see GetPositionIgain 查看详细说明
   */
  Error SetPositionIgain(const float position_igain) {
    return WriteRegField(ControlTable::kPositionIgain,
                         PidIGainToRaw(position_igain));
  }

  /**
   * @brief 获取位置环 P 增益 (R/W)
   * @param[out] position_pgain P 增益
   * @return Error::kOk 成功
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
   * @see SetPositionIgain, SetPositionDgain, GetOperatingMode
   */
  Error GetPositionPgain(float& position_pgain) {
    uint16_t raw;
    CHECK(ReadRegField(ControlTable::kPositionPgain, raw));
    position_pgain = PidPGainFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置位置环 P 增益 (R/W)
   * @param[in] position_pgain P 增益
   * @return Error::kOk 成功
   *
   * @see GetPositionPgain 查看详细说明
   */
  Error SetPositionPgain(const float position_pgain) {
    return WriteRegField(ControlTable::kPositionPgain,
                         PidPGainToRaw(position_pgain));
  }

  /**
   * @brief 获取前馈二阶增益 (R/W)
   * @param[out] feedforward_2nd_gain 前馈二阶增益
   * @return Error::kOk 成功
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
   * @see SetFeedforward1stGain, GetPositionPgain, GetOperatingMode
   */
  Error GetFeedforward2ndGain(float& feedforward_2nd_gain) {
    uint16_t raw;
    CHECK(ReadRegField(ControlTable::kFeedforward2ndGain, raw));
    feedforward_2nd_gain = FeedforwardGainFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置前馈二阶增益 (R/W)
   * @param[in] feedforward_2nd_gain 前馈二阶增益
   * @return Error::kOk 成功
   *
   * @see GetFeedforward2ndGain 查看详细说明
   */
  Error SetFeedforward2ndGain(const float feedforward_2nd_gain) {
    return WriteRegField(ControlTable::kFeedforward2ndGain,
                         FeedforwardGainToRaw(feedforward_2nd_gain));
  }

  /**
   * @brief 获取前馈一阶增益 (R/W)
   * @param[out] feedforward_1st_gain 前馈一阶增益
   * @return Error::kOk 成功
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
   * @see SetFeedforward2ndGain, GetPositionPgain, GetOperatingMode
   */
  Error GetFeedforward1stGain(float& feedforward_1st_gain) {
    uint16_t raw;
    CHECK(ReadRegField(ControlTable::kFeedforward1stGain, raw));
    feedforward_1st_gain = FeedforwardGainFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置前馈一阶增益 (R/W)
   * @param[in] feedforward_1st_gain 前馈一阶增益
   * @return Error::kOk 成功
   *
   * @see GetFeedforward1stGain 查看详细说明
   */
  Error SetFeedforward1stGain(const float feedforward_1st_gain) {
    return WriteRegField(ControlTable::kFeedforward1stGain,
                         FeedforwardGainToRaw(feedforward_1st_gain));
  }

  /**
   * @brief 获取总线看门狗 (R/W)
   * @param[out] milliseconds 看门狗时间（ms）
   * @return Error::kOk 成功
   *
   * 范围: 0-5080 ms
   *
   * 【功能说明】
   * - 总线看门狗超时时间，用于检测通信故障
   * - 当超过此时间未收到指令时，舵机会自动关断力矩
   * - 0: 禁用看门狗功能
   * - 非0: 启用看门狗功能
   *
   * 【使用场景】
   * - 通信故障检测：自动检测通信中断
   * - 安全保护：通信中断时自动关断力矩
   * - 系统监控：确保通信正常
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
   * @see SetTorqueEnable, GetHardwareErrorStatus
   */
  Error GetBusWatchdog(uint16_t& milliseconds) {
    uint8_t raw;
    CHECK(ReadRegField(ControlTable::kBusWatchdog, raw));
    milliseconds = WatchdogFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置总线看门狗 (R/W)
   * @param[in] milliseconds 看门狗时间（ms）
   * @return Error::kOk 成功
   *
   * @note 看门狗超时会自动关断力矩，需要重新使能
   * @see GetBusWatchdog 查看详细说明
   */
  Error SetBusWatchdog(const uint16_t milliseconds) {
    return WriteRegField(ControlTable::kBusWatchdog,
                         WatchdogToRaw(milliseconds));
  }

  /** @} */  // PID 控制参数组结束

  //==============================================================================
  // 目标值设置区（RAM）
  //==============================================================================
  /**
   * @name 目标值设置
   * 设置各控制模式下的目标值。
   * 实际使用的目标值取决于当前的 Operating Mode(11)。
   * @{
   */

  /**
   * @brief 获取目标 PWM (R/W)
   * @param[out] percent 目标 PWM（%）
   * @return Error::kOk 成功
   *
   * 范围: -100.0 ~ 100.0%
   *
   * 【功能说明】
   * - 设置电机驱动器的 PWM 占空比
   * - 仅在 PWM Control Mode (Operating Mode = 16) 有效
   * - 负值表示反向旋转
   * - 受 PWM Limit 限制
   *
   * 【使用场景】
   * - 开环控制：直接控制电机输出
   * - 调试测试：验证电机基本功能
   * - 紧急控制：在控制系统故障时使用
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
   * @see SetOperatingMode, GetPwmLimit, GetPresentPwm
   */
  Error GetGoalPwm(float& percent) {
    uint16_t raw;
    CHECK(ReadRegField(ControlTable::kGoalPwm, raw));
    percent = PwmFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置目标 PWM (R/W)
   * @param[in] percent 目标 PWM（%）
   * @return Error::kOk 成功
   *
   * @warning 开环控制可能导致失速或过载
   * @note 仅在 PWM Control Mode 有效
   * @see GetGoalPwm 查看详细说明
   */
  Error SetGoalPwm(const float percent) {
    return WriteRegField(ControlTable::kGoalPwm, PwmToRaw(percent));
  }

  /**
   * @brief 获取目标电流 (R/W)
   * @param[out] goal_current 目标电流（mA）
   * @return Error::kOk 成功
   *
   * 范围: -1193 ~ 1193 mA
   *
   * 【功能说明】
   * - 设置电机输出电流目标值
   * - 在 Current Control Mode (Operating Mode = 0) 中作为控制目标
   * - 在 Current-based Position Control Mode (Operating Mode = 5)
   * 中作为最大电流限制
   * - 负值表示反向电流
   * - 受 Current Limit 限制
   *
   * 【使用场景】
   * - 力矩控制：直接控制输出力矩
   * - 力控应用：柔顺控制和力反馈
   * - 安全限制：防止过载
   *
   * 【典型参数】
   * - 停止: 0 mA（无输出）
   * - 轻载: ±100-300 mA（轻负载）
   * - 中载: ±300-600 mA（中等负载）
   * - 重载: ±600-1193 mA（重负载）
   *
   * 【相关寄存器】
   * - Operating Mode: 决定电流控制模式
   * - Current Limit: 限制最大电流
   * - Present Current: 实际电流输出
   *
   * @note 在 Current Control Mode 中作为控制目标，在 Current-based Position
   * Control Mode 中作为限制
   * @see SetOperatingMode, GetCurrentLimit, GetPresentCurrent
   */
  Error GetGoalCurrent(uint16_t& goal_current) {
    return ReadRegField(ControlTable::kGoalCurrent, goal_current);
  }

  /**
   * @brief 设置目标电流 (R/W)
   * @param[in] goal_current 目标电流（mA）
   * @return Error::kOk 成功
   *
   * @note 在 Current Control Mode 中作为控制目标，在 Current-based Position
   * Control Mode 中作为限制
   * @see GetGoalCurrent 查看详细说明
   */
  Error SetGoalCurrent(const uint16_t goal_current) {
    return WriteRegField(ControlTable::kGoalCurrent, goal_current);
  }

  /**
   * @brief 获取目标速度 (R/W)
   * @param[out] rpm 目标速度（RPM）
   * @return Error::kOk 成功
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
   * 【使用场景】
   * - 速度控制：直接控制电机转速
   * - 轮式机器人：控制轮子转速
   * - 传送带控制：控制传送带速度
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
   * @see SetOperatingMode, GetVelocityLimit, GetPresentVelocity
   */
  Error GetGoalVelocity(float& rpm) {
    uint32_t raw;
    CHECK(ReadRegField(ControlTable::kGoalVelocity, raw));
    rpm = VelocityFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置目标速度 (R/W)
   * @param[in] rpm 目标速度（RPM）
   * @return Error::kOk 成功
   *
   * @note 仅在 Velocity Control Mode 中作为控制目标
   * @see GetGoalVelocity 查看详细说明
   */
  Error SetGoalVelocity(const float rpm) {
    return WriteRegField(ControlTable::kGoalVelocity, VelocityToRaw(rpm));
  }

  /**
   * @brief 获取轨迹加速度 (RW)
   * @param[out] acceleration 轨迹加速度（rev/min²，Velocity-based模式）
   * @return Error::kOk 成功
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
   * 【使用场景】
   * - 平滑启动：避免突然加速对机械的冲击
   * - 精确定位：控制加减速过程，提高定位精度
   * - 保护机械：限制加速度，延长机械寿命
   * - 优化轨迹：根据负载特性调整加速度
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
   * @see SetProfileVelocity, GetDriveMode, SetGoalPosition
   */
  Error GetProfileAcceleration(float& acceleration) {
    uint32_t raw;
    CHECK(ReadRegField(ControlTable::kProfileAcceleration, raw));
    acceleration = AccelerationFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置轨迹加速度
   * @param[in] acceleration 轨迹加速度（rev/min²，Velocity-based模式）
   * @return Error::kOk 成功
   *
   * @note Time-based模式下单位为ms，当前实现假设Velocity-based模式
   * @see GetProfileAcceleration 查看详细说明
   */
  Error SetProfileAcceleration(const float acceleration) {
    return WriteRegField(ControlTable::kProfileAcceleration,
                         AccelerationToRaw(acceleration));
  }

  /**
   * @brief 获取轨迹速度 (RW)
   * @param[out] rpm 轨迹速度（RPM，Velocity-based模式）
   * @return Error::kOk 成功
   *
   * 范围: 0-7,503.643 RPM (Velocity-based模式) | 0-32767 ms (Time-based模式)
   *
   * 【Profile 处理流程】
   * 根据官方文档，当接收到新的 Goal Position 时：
   *
   * 1. 指令通过 DYNAMIXEL 总线传输，注册到 Goal Position(116)
   * 2. 基于 Profile Velocity(112) 和 Profile Acceleration(108) 计算加速时间 t1
   * 3. 根据以下条件决定 Profile 类型：
   *    - Profile Velocity = 0 → Step (不使用 Profile)
   *    - Profile Velocity ≠ 0, Profile Acceleration = 0 → Rectangular
   *    - Profile Velocity ≠ 0, Profile Acceleration ≠ 0 → Trapezoidal
   *    - 移动距离不足时，Trapezoidal 自动降级为 Triangular
   * 4. 选择的 Profile 类型存储在 Moving Status(123) 的 Bit 4-5
   * 5. Profile Generator 计算期望轨迹:
   *    - Position Trajectory(140): 期望位置轨迹
   *    - Velocity Trajectory(136): 期望速度轨迹
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
   * | Profile Velocity | Profile Acceleration | Profile 类型    |
   * |------------------|----------------------|----------------|
   * | 0                | -                    | Step (无 Profile) |
   * | 非 0             | 0                    | Rectangular    |
   * | 非 0             | 非 0                 | Trapezoidal    |
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
   * 【使用场景】
   * - Position Control Mode: 控制位置运动的速度曲线
   * - Extended Position Control Mode: 多圈位置运动的速度控制
   * - 平滑运动: 避免突然启停，保护机械结构
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
   * @see SetProfileAcceleration, SetGoalPosition, GetDriveMode
   */
  Error GetProfileVelocity(float& rpm) {
    uint32_t raw;
    CHECK(ReadRegField(ControlTable::kProfileVelocity, raw));
    rpm = VelocityFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置轨迹速度
   * @param[in] rpm 轨迹速度（RPM，Velocity-based模式）
   * @return Error::kOk 成功
   *
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * @note Time-based模式下单位为ms，当前实现假设Velocity-based模式
   * @see GetProfileVelocity 查看详细说明
   */
  Error SetProfileVelocity(const float rpm) {
    return WriteRegField(ControlTable::kProfileVelocity, VelocityToRaw(rpm));
  }

  /**
   * @brief 获取目标位置 (RW)
   * @param[out] goal_position 目标位置（pulse）
   * @return Error::kOk 成功
   *
   * 范围: 0-4095 (Position Mode) | -1,048,575 到 1,048,575 (Extended Mode)
   *
   * 【功能说明】
   * 设置舵机的目标位置，实际行为取决于当前 Operating Mode：
   * - Position Control Mode: 单圈位置控制，范围 0-4095 (0-360°)
   * - Extended Position Control Mode: 多圈位置控制，支持无限旋转
   *
   * 【使用场景】
   * - Position Control Mode: 关节控制、单圈定位
   * - Extended Position Control Mode: 多圈定位、旋转计数、连续旋转
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
   * @see SetOperatingMode, GetPresentPosition, SetProfileVelocity
   */
  Error GetGoalPosition(int32_t& goal_position) {
    uint32_t raw;
    CHECK(ReadRegField(ControlTable::kGoalPosition, raw));
    goal_position = static_cast<int32_t>(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置目标位置 (RW)
   * @param[in] goal_position 目标位置（pulse）
   * @return Error::kOk 成功
   *
   * 【在 Position Control Mode 中的行为】
   * - 自动限制在 [Min Position Limit, Max Position Limit] 范围内
   * - 触发 Profile 重新生成（如果启用 Profile）
   * - 平滑更新轨迹（不会突变）
   *
   * 【运动中更新 Goal Position】
   * - 旧轨迹被中断
   * - 基于当前速度生成新轨迹
   * - 速度平滑过渡到新的期望轨迹
   *
   * @note 其他工作模式下此寄存器无效
   * @see GetGoalPosition, GetMinPositionLimit, GetMaxPositionLimit
   */
  Error SetGoalPosition(const int32_t goal_position) {
    return WriteRegField(ControlTable::kGoalPosition,
                         static_cast<uint32_t>(goal_position));
  }

  /** @} */  // 目标值设置组结束

  //==============================================================================
  // 状态反馈区（RAM，只读）
  //==============================================================================
  /**
   * @name 状态反馈
   * 读取舵机当前运行状态和传感器数据。
   * 所有寄存器为只读，由舵机内部自动更新。
   * @{
   */

  /**
   * @brief 获取实时时钟 (R)
   * @param[out] realtime_tick 实时时钟（ms）
   * @return Error::kOk 成功
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
   * 【使用场景】
   * - 时间戳：记录事件发生时间
   * - 定时功能：实现延时和超时检测
   * - 调试辅助：分析时序问题
   *
   * @note 此寄存器为只读，由系统自动更新
   * @see SetRealtimeTick
   */
  Error GetRealtimeTick(uint16_t& realtime_tick) {
    return ReadRegField(ControlTable::kRealtimeTick, realtime_tick);
  }

  /**
   * @brief 设置实时时钟 (R)
   * @param[in] realtime_tick 实时时钟
   * @return Error::kOk 成功
   *
   * @see GetRealtimeTick 查看详细说明
   */
  Error SetRealtimeTick(const uint16_t realtime_tick) {
    return WriteRegField(ControlTable::kRealtimeTick, realtime_tick);
  }

  /**
   * @brief 获取运动状态 (R)
   * @param[out] moving 运动状态
   * @return Error::kOk 成功
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
   * 【使用场景】
   * - 运动检测：快速判断舵机是否在运动
   * - 到达检测：判断是否到达目标位置
   * - 状态监控：实时监控运动状态
   *
   * 【相关寄存器】
   * - Moving Threshold: 运动判断阈值
   * - Present Velocity: 实际速度值
   * - Moving Status: 详细的运动状态
   *
   * @note 此寄存器为只读，由系统自动更新
   * @see GetMovingThreshold, GetPresentVelocity, GetMovingStatus
   */
  Error GetMoving(uint8_t& moving) {
    return ReadRegField(ControlTable::kMoving, moving);
  }

  /**
   * @brief 设置运动状态 (R)
   * @param[in] moving 运动状态
   * @return Error::kOk 成功
   *
   * @see GetMoving 查看详细说明
   */
  Error SetMoving(const uint8_t moving) {
    return WriteRegField(ControlTable::kMoving, moving);
  }

  /**
   * @brief 获取详细运动状态 (R)
   * @param[out] moving_status 详细运动状态位域
   * @return Error::kOk 成功
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
   * | Bit | 名称                    | 说明                          |
   * |-----|------------------------|-------------------------------|
   * | 0   | In-Position            | 0: 未到达目标位置, 1: 已到达 |
   * | 1   | Profile Ongoing        | 0: 轨迹已完成, 1: 轨迹进行中 |
   * | 2   | Reserved               | 保留位                        |
   * | 3   | Following Error        | 0: 正在跟随, 1: 未跟随轨迹    |
   * | 4-5 | Velocity Profile       | 00: Step, 01: Rectangular, 10: Triangular,
   * 11: Trapezoidal | | 6-7 | Reserved               | 保留位 |
   *
   * 【Profile 类型详解】
   *
   * 参数配置（3 种基本类型）：
   * | Profile Velocity | Profile Acceleration | 配置的类型    |
   * |------------------|----------------------|--------------|
   * | 0                | -                    | Step         |
   * | 非 0             | 0                    | Rectangular  |
   * | 非 0             | 非 0                 | Trapezoidal  |
   *
   * 运行时状态（4 种可能状态，存储在 Bit 4-5）：
   * | 编码 | 类型        | 说明                                    |
   * |------|------------|----------------------------------------|
   * | 00   | Step       | 不使用 Profile，立即以最大速度移动      |
   * | 01   | Rectangular| 矩形曲线：匀速运动，无加减速过程        |
   * | 10   | Triangular | 三角形曲线：只有加速和减速，无匀速段    |
   * | 11   | Trapezoidal| 梯形曲线：完整的加速-匀速-减速过程      |
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
   * 【使用场景】
   * - 位置控制：监控位置到达状态
   * - 轨迹跟踪：检查轨迹执行状态
   * - 错误诊断：检测跟随误差
   * - 调试分析：了解运动曲线类型
   *
   * 【相关寄存器】
   * - Goal Position: 目标位置
   * - Present Position: 当前位置
   * - Profile Velocity: 速度曲线参数
   * - Profile Acceleration: 加速度参数
   *
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * @see GetGoalPosition, GetPresentPosition, GetProfileVelocity
   */
  Error GetMovingStatus(uint8_t& moving_status) {
    return ReadRegField(ControlTable::kMovingStatus, moving_status);
  }

  /**
   * @brief 设置详细运动状态 (R)
   * @param[in] moving_status 详细运动状态位域
   * @return Error::kOk 成功
   *
   * @see GetMovingStatus 查看详细说明
   */
  Error SetMovingStatus(const uint8_t moving_status) {
    return WriteRegField(ControlTable::kMovingStatus, moving_status);
  }

  /**
   * @brief 获取当前 PWM (R)
   * @param[out] percent 当前 PWM（%）
   * @return Error::kOk 成功
   *
   * 范围: -100.0% 到 100.0%
   *
   * 【功能说明】
   * - 显示实际输出的 PWM 占空比
   * - 正值表示正向旋转
   * - 负值表示反向旋转
   * - 实时更新，反映当前输出状态
   *
   * 【使用场景】
   * - 输出监控：监控实际 PWM 输出
   * - 调试分析：分析 PWM 控制效果
   * - 状态反馈：获取当前输出状态
   *
   * 【相关寄存器】
   * - Goal PWM: 目标 PWM 值
   * - PWM Limit: PWM 限制值
   * - Torque Enable: 力矩使能状态
   *
   * @note 此寄存器为只读，由系统自动更新
   * @see GetGoalPwm, GetPwmLimit, GetTorqueEnable
   */
  Error GetPresentPwm(float& percent) {
    uint16_t raw;
    CHECK(ReadRegField(ControlTable::kPresentPwm, raw));
    percent = PwmFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置当前 PWM (R)
   * @param[in] present_pwm 当前 PWM
   * @return Error::kOk 成功
   *
   * @see GetPresentPwm 查看详细说明
   */
  Error SetPresentPwm(const uint16_t present_pwm) {
    return WriteRegField(ControlTable::kPresentPwm, present_pwm);
  }

  /**
   * @brief 获取当前电流 (R)
   * @param[out] present_current 当前电流（mA）
   * @return Error::kOk 成功
   *
   * 范围: 0-1193 mA
   *
   * 【功能说明】
   * - 显示当前通过电机的电流
   * - 电流与负载力矩成正比
   * - 实时更新，反映当前电流状态
   * - 用于监控电机负载和功耗
   *
   * 【使用场景】
   * - 负载监控：监控电机负载状态
   * - 功耗分析：分析电机功耗
   * - 过载保护：检测过载情况
   * - 调试分析：分析电流控制效果
   *
   * 【相关寄存器】
   * - Goal Current: 目标电流值
   * - Current Limit: 电流限制值
   * - Torque Enable: 力矩使能状态
   *
   * @note 此寄存器为只读，由系统自动更新
   * @see GetGoalCurrent, GetCurrentLimit, GetTorqueEnable
   */
  Error GetPresentCurrent(uint16_t& present_current) {
    return ReadRegField(ControlTable::kPresentCurrent, present_current);
  }

  /**
   * @brief 设置当前电流 (R)
   * @param[in] present_current 当前电流
   * @return Error::kOk 成功
   *
   * @see GetPresentCurrent 查看详细说明
   */
  Error SetPresentCurrent(const uint16_t present_current) {
    return WriteRegField(ControlTable::kPresentCurrent, present_current);
  }

  /**
   * @brief 获取当前速度 (R)
   * @param[out] rpm 当前速度（RPM）
   * @return Error::kOk 成功
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
   * 【使用场景】
   * - 速度监控：监控电机转速
   * - 运动控制：用于速度控制模式
   * - 调试分析：分析速度控制效果
   * - 状态反馈：获取当前运动状态
   *
   * 【相关寄存器】
   * - Goal Velocity: 目标速度值
   * - Velocity Limit: 速度限制值
   * - Moving Threshold: 运动判断阈值
   * - Moving: 运动状态指示
   *
   * @note 此寄存器为只读，由系统自动更新
   * @see GetGoalVelocity, GetVelocityLimit, GetMovingThreshold
   */
  Error GetPresentVelocity(float& rpm) {
    uint32_t raw;
    CHECK(ReadRegField(ControlTable::kPresentVelocity, raw));
    rpm = VelocityFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置当前速度 (R)
   * @param[in] present_velocity 当前速度
   * @return Error::kOk 成功
   *
   * @see GetPresentVelocity 查看详细说明
   */
  Error SetPresentVelocity(const uint32_t present_velocity) {
    return WriteRegField(ControlTable::kPresentVelocity, present_velocity);
  }

  /**
   * @brief 获取当前位置 (R)
   * @param[out] present_position 当前位置（pulse）
   * @return Error::kOk 成功
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
   * 【使用场景】
   * - 位置监控：实时跟踪舵机位置
   * - 闭环控制：作为位置反馈信号
   * - 状态检测：判断是否到达目标位置
   * - 调试分析：观察运动轨迹
   *
   * 【相关寄存器】
   * - Goal Position: 目标位置对比
   * - Operating Mode: 决定位置范围
   * - Moving Status: 运动状态指示
   * - Homing Offset: 零点偏移
   *
   * @note 此寄存器为只读，由系统自动更新
   * @see GetGoalPosition, GetOperatingMode, GetMovingStatus
   */
  Error GetPresentPosition(int32_t& present_position) {
    uint32_t raw;
    CHECK(ReadRegField(ControlTable::kPresentPosition, raw));
    present_position = static_cast<int32_t>(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置当前位置 (R)
   * @param[in] present_position 当前位置
   * @return Error::kOk 成功
   *
   * @see GetPresentPosition 查看详细说明
   */
  Error SetPresentPosition(const int32_t present_position) {
    return WriteRegField(ControlTable::kPresentPosition,
                         static_cast<uint32_t>(present_position));
  }

  /**
   * @brief 获取速度轨迹 (R)
   * @param[out] rpm 速度轨迹（RPM）
   * @return Error::kOk 成功
   *
   * 范围: -468.763 到 468.763 RPM
   *
   * 【功能说明】
   * - Profile 生成的期望速度轨迹值
   * - 存储在 Velocity Trajectory(136) 寄存器中
   * - 作为速度控制器的参考输入
   * - 实时更新，反映 Profile 计算的期望速度
   * - 与 Present Velocity 对比可得速度跟踪误差
   *
   * 【控制流程】
   * Goal Position → [Profile Generator] → Velocity Trajectory(136) → [Velocity
   * PID] → PWM
   * - Profile Generator 根据 Goal Position 生成期望速度轨迹
   * - 期望速度轨迹存储在 Velocity Trajectory(136) 中
   * - Velocity PID 控制器跟踪 Velocity Trajectory
   * - 输出 PWM 驱动电机
   *
   * 【使用场景】
   * - 轨迹监控：观察期望速度轨迹
   * - 调试分析：分析 Profile 效果
   * - 控制验证：验证级联控制结构
   * - 性能优化：优化控制参数
   *
   * 【相关寄存器】
   * - Profile Velocity: 速度曲线参数
   * - Profile Acceleration: 加速度参数
   * - Present Velocity: 实际速度值
   * - Position Trajectory: 位置轨迹
   *
   * @note 此寄存器为只读，由系统自动更新
   * @see GetProfileVelocity, GetPresentVelocity, GetPositionTrajectory
   */
  Error GetVelocityTrajectory(float& rpm) {
    uint32_t raw;
    CHECK(ReadRegField(ControlTable::kVelocityTrajectory, raw));
    rpm = VelocityFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置速度轨迹 (R)
   * @param[in] velocity_trajectory 速度轨迹
   * @return Error::kOk 成功
   *
   * @see GetVelocityTrajectory 查看详细说明
   */
  Error SetVelocityTrajectory(const uint32_t velocity_trajectory) {
    return WriteRegField(ControlTable::kVelocityTrajectory,
                         velocity_trajectory);
  }

  /**
   * @brief 获取位置轨迹 (R)
   * @param[out] position_trajectory 位置轨迹（pulse）
   * @return Error::kOk 成功
   *
   * 范围: 0-4095 (Position Mode) | -1,048,575 到 1,048,575 (Extended Mode)
   *
   * 【功能说明】
   * - 读取 Profile 生成的期望位置轨迹
   * - 位置 PID 控制器的参考输入
   * - 与 Present Position 的区别：期望位置 vs 实际位置
   * - 实时更新，反映期望位置轨迹
   *
   * 【使用场景】
   * - Profile 调试：观察期望轨迹是否合理
   * - 控制分析：对比期望位置与实际位置
   * - 性能评估：分析跟踪误差
   * - 轨迹验证：验证 Profile 生成效果
   *
   * 【相关寄存器】
   * - Goal Position: 轨迹的最终目标
   * - Present Position: 实际位置对比
   * - Profile Velocity: 速度曲线参数
   * - Profile Acceleration: 加速度参数
   * - Moving Status: 轨迹状态指示
   *
   * @note 此寄存器为只读，由系统自动更新
   * @see GetGoalPosition, GetPresentPosition, GetProfileVelocity
   */
  Error GetPositionTrajectory(int32_t& position_trajectory) {
    uint32_t raw;
    CHECK(ReadRegField(ControlTable::kPositionTrajectory, raw));
    position_trajectory = static_cast<int32_t>(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置位置轨迹 (R)
   * @param[in] position_trajectory 位置轨迹
   * @return Error::kOk 成功
   *
   * @see GetPositionTrajectory 查看详细说明
   */
  Error SetPositionTrajectory(const int32_t position_trajectory) {
    return WriteRegField(ControlTable::kPositionTrajectory,
                         static_cast<uint32_t>(position_trajectory));
  }

  /**
   * @brief 获取当前输入电压 (R)
   * @param[out] voltage 当前输入电压（V）
   * @return Error::kOk 成功
   *
   * 范围: 0-16.0 V
   *
   * 【功能说明】
   * - 显示当前输入电压
   * - 用于监控电源状态
   * - 实时更新，反映当前电压
   * - 电压异常会触发保护
   *
   * 【使用场景】
   * - 电源监控：监控电源电压
   * - 保护检测：检测电压异常
   * - 调试分析：分析电源问题
   * - 故障诊断：判断电源故障
   *
   * 【相关寄存器】
   * - Min Voltage Limit: 最低电压限制
   * - Max Voltage Limit: 最高电压限制
   * - Hardware Error Status: 电压错误标志
   * - Shutdown: 电压错误保护配置
   *
   * @note 此寄存器为只读，由系统自动更新
   * @see GetMinVoltageLimit, GetMaxVoltageLimit, GetHardwareErrorStatus
   */
  Error GetPresentInputVoltage(float& voltage) {
    uint16_t raw;
    CHECK(ReadRegField(ControlTable::kPresentInputVoltage, raw));
    voltage = VoltageFromRaw(raw);
    return Error::kOk;
  }

  /**
   * @brief 设置当前输入电压 (R)
   * @param[in] present_input_voltage 当前输入电压
   * @return Error::kOk 成功
   *
   * @see GetPresentInputVoltage 查看详细说明
   */
  Error SetPresentInputVoltage(const uint16_t present_input_voltage) {
    return WriteRegField(ControlTable::kPresentInputVoltage,
                         present_input_voltage);
  }

  /**
   * @brief 获取当前温度 (R)
   * @param[out] present_temperature 当前温度（°C）
   * @return Error::kOk 成功
   *
   * 范围: 0-100 °C
   *
   * 【功能说明】
   * - 显示当前电机温度
   * - 温度传感器位于电机内部
   * - 实时更新，反映当前温度
   * - 温度过高会触发保护
   *
   * 【使用场景】
   * - 温度监控：监控电机温度
   * - 过热保护：检测过热情况
   * - 调试分析：分析热量问题
   * - 故障诊断：判断过热故障
   *
   * 【相关寄存器】
   * - Temperature Limit: 温度限制值
   * - Hardware Error Status: 温度错误标志
   * - Shutdown: 温度错误保护配置
   *
   * @note 此寄存器为只读，由系统自动更新
   * @warning 温度过高（>80°C）会损坏电机
   * @see GetTemperatureLimit, GetHardwareErrorStatus
   */
  Error GetPresentTemperature(uint8_t& present_temperature) {
    return ReadRegField(ControlTable::kPresentTemperature, present_temperature);
  }

  /**
   * @brief 设置当前温度 (R)
   * @param[in] present_temperature 当前温度
   * @return Error::kOk 成功
   *
   * @see GetPresentTemperature 查看详细说明
   */
  Error SetPresentTemperature(const uint8_t present_temperature) {
    return WriteRegField(ControlTable::kPresentTemperature,
                         present_temperature);
  }

  /** @} */  // 状态反馈组结束

  Error RecoveryEeprom();
  Error LoadEeprom();
  Error StoreEeprom();
  Error StoreEeprom(const uint8_t address, const uint8_t size);

 private:
  uint8_t table_[TableBlocks::kTotal.size()] = {};
};

}  // namespace hortor::servo_slave
