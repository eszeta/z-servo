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
 *
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │ EEPROM 区 (0-63) - 掉电保存                                          │
 * ├──────────────────────────┬──────────────────────────────────────────┤
 * │ 设备信息                 │ 0-6                                       │
 * │ 通信配置                 │ 7-13                                      │
 * │ 工作模式                 │ 10-11                                     │
 * │ 限位与保护               │ 20-63                                     │
 * ├─────────────────────────────────────────────────────────────────────┤
 * │ RAM 区 (64-146) - 掉电丢失                                           │
 * ├──────────────────────────┬──────────────────────────────────────────┤
 * │ 使能与状态               │ 64-70                                     │
 * │ PID 控制参数             │ 76-98                                     │
 * │ 目标值设置               │ 100-119                                   │
 * │ 状态反馈                 │ 120-146                                   │
 * └──────────────────────────┴──────────────────────────────────────────┘
 */

#pragma once

#include <Arduino.h>

#include "protocol/regmap.h"
#include "regmap/regmap_mmio.h"
#include "types.h"

namespace hortor::servo_slave {

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
   */
  Error GetModelNumber(uint16_t& model_number) {
    return ReadRegField(ControlTable::kModelNumber, model_number);
  }

  /**
   * @brief 设置型号编号 (R)
   * @param[in] model_number 型号编号
   * @return Error::kOk 成功
   */
  Error SetModelNumber(const uint16_t model_number) {
    return WriteRegField(ControlTable::kModelNumber, model_number);
  }

  /**
   * @brief 获取型号信息 (R)
   * @param[out] model_information 型号信息
   * @return Error::kOk 成功
   */
  Error GetModelInformation(uint32_t& model_information) {
    return ReadRegField(ControlTable::kModelInformation, model_information);
  }

  /**
   * @brief 设置型号信息 (R)
   * @param[in] model_information 型号信息
   * @return Error::kOk 成功
   */
  Error SetModelInformation(const uint32_t model_information) {
    return WriteRegField(ControlTable::kModelInformation, model_information);
  }

  /**
   * @brief 获取固件版本 (R)
   * @param[out] firmware_version 固件版本
   * @return Error::kOk 成功
   */
  Error GetFirmwareVersion(uint8_t& firmware_version) {
    return ReadRegField(ControlTable::kFirmwareVersion, firmware_version);
  }

  /**
   * @brief 设置固件版本 (R)
   * @param[in] firmware_version 固件版本
   * @return Error::kOk 成功
   */
  Error SetFirmwareVersion(const uint8_t firmware_version) {
    return WriteRegField(ControlTable::kFirmwareVersion, firmware_version);
  }

  /** @} */ // 设备信息组结束

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
   */
  Error SetId(const uint8_t id) { return WriteRegField(ControlTable::kId, id); }

  /**
   * @brief 获取波特率索引
   * @param[out] baud_rate 波特率索引
   * @return Error::kOk 成功
   *
   * 波特率索引对应表：
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
   * @note 修改后需调用 StoreEeprom() 并重启生效
   * @warning 确保主机端波特率与此设置一致，否则无法通信
   */
  Error GetBaudRate(uint8_t& baud_rate) {
    return ReadRegField(ControlTable::kBaudRate, baud_rate);
  }

  /**
   * @brief 设置波特率索引
   * @param[in] baud_rate 波特率索引 (0-7)
   * @return Error::kOk 成功
   *
   * @warning 修改后需调用 StoreEeprom() 并重启生效
   * @warning 确保主机端波特率与此设置一致，否则无法通信
   */
  Error SetBaudRate(const uint8_t baud_rate) {
    return WriteRegField(ControlTable::kBaudRate, baud_rate);
  }

  /**
   * @brief 获取返回延迟时间
   * @param[out] return_delay_time 返回延迟时间
   * @return Error::kOk 成功
   *
   * 单位: 2 μs
   * 返回状态包前的延迟时间 = return_delay_time × 2 μs
   *
   * 示例：
   * - 0: 无延迟
   * - 250: 500 μs 延迟（默认）
   * - 254: 508 μs 延迟（最大）
   *
   * @note 在多舵机串联时，适当的延迟可避免总线冲突
   */
  Error GetReturnDelayTime(uint8_t& return_delay_time) {
    return ReadRegField(ControlTable::kReturnDelayTime, return_delay_time);
  }

  /**
   * @brief 设置返回延迟时间
   * @param[in] return_delay_time 返回延迟时间 (0-254)
   * @return Error::kOk 成功
   *
   * @note 在多舵机串联时，适当的延迟可避免总线冲突
   */
  Error SetReturnDelayTime(const uint8_t return_delay_time) {
    return WriteRegField(ControlTable::kReturnDelayTime, return_delay_time);
  }

  /**
   * @brief 获取驱动模式
   * @param[out] drive_mode 驱动模式位域
   * @return Error::kOk 成功
   *
   * 驱动模式位域定义：
   * | Bit | 名称                    | 说明                          |
   * |-----|------------------------|-------------------------------|
   * | 0   | Normal/Reverse Mode    | 0: 正转, 1: 反转              |
   * | 1   | -                      | 未使用                        |
   * | 2   | Profile Configuration  | 0: Velocity-based, 1: Time-based |
   * | 3-7 | -                      | 未使用                        |
   *
   * 反转模式 (Bit 0 = 1):
   * - 正向指令导致反向旋转
   * - Present Position 值反向
   *
   * Profile 配置 (Bit 2):
   * - 0: Velocity-based Profile - 使用速度和加速度参数
   * - 1: Time-based Profile - 使用时间参数（毫秒）
   */
  Error GetDriveMode(uint8_t& drive_mode) {
    return ReadRegField(ControlTable::kDriveMode, drive_mode);
  }

  /**
   * @brief 设置驱动模式
   * @param[in] drive_mode 驱动模式位域
   * @return Error::kOk 成功
   *
   * @see GetDriveMode 查看位域定义
   */
  Error SetDriveMode(const uint8_t drive_mode) {
    return WriteRegField(ControlTable::kDriveMode, drive_mode);
  }

  /**
   * @brief 获取工作模式 (R/W)
   * @param[out] operating_mode 工作模式
   * @return Error::kOk 成功
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
   * @see GetOperatingMode 查看各模式详细说明
   */
  Error SetOperatingMode(const uint8_t operating_mode) {
    return WriteRegField(ControlTable::kOperatingMode, operating_mode);
  }

  /**
   * @brief 获取副ID
   * @param[out] secondary_id 副ID
   * @return Error::kOk 成功
   *
   * 副ID 用于额外的舵机识别：
   * - 范围: 0-252
   * - 0: 禁用副ID
   * - 非0: 启用副ID功能
   *
   * @note 副ID 功能的具体用途取决于应用实现
   */
  Error GetSecondaryId(uint8_t& secondary_id) {
    return ReadRegField(ControlTable::kSecondaryId, secondary_id);
  }

  /**
   * @brief 设置副ID
   * @param[in] secondary_id 副ID (0-252)
   * @return Error::kOk 成功
   *
   * @note 副ID 功能的具体用途取决于应用实现
   */
  Error SetSecondaryId(const uint8_t secondary_id) {
    return WriteRegField(ControlTable::kSecondaryId, secondary_id);
  }

  /**
   * @brief 获取协议版本
   * @param[out] protocol_version 协议版本
   * @return Error::kOk 成功
   *
   * 协议版本：
   * - 1: DYNAMIXEL Protocol 1.0
   * - 2: DYNAMIXEL Protocol 2.0（默认）
   *
   * @note XL330 支持 Protocol 2.0
   */
  Error GetProtocolVersion(uint8_t& protocol_version) {
    return ReadRegField(ControlTable::kProtocolVersion, protocol_version);
  }

  /**
   * @brief 设置协议版本
   * @param[in] protocol_version 协议版本 (1-2)
   * @return Error::kOk 成功
   *
   * @note XL330 支持 Protocol 2.0
   */
  Error SetProtocolVersion(const uint8_t protocol_version) {
    return WriteRegField(ControlTable::kProtocolVersion, protocol_version);
  }

  /** @} */ // 通信配置组结束

  //==============================================================================
  // 限位与保护设置区（EEPROM）
  //==============================================================================
  /**
   * @name 限位与保护设置
   * 配置运动限位、温度/电压/电流保护阈值，防止舵机损坏。
   * @{
   */

  /**
   * @brief 获取归零偏移
   * @param[out] homing_offset 归零偏移
   * @return Error::kOk 成功
   *
   * 单位: pulse | 范围: -1,048,575 ~ 1,048,575
   *
   * 归零偏移用于定义舵机的零位：
   * - 0: 无偏移
   * - 正值: 顺时针偏移
   * - 负值: 逆时针偏移
   *
   * @note 偏移值会加到所有位置指令上
   */
  Error GetHomingOffset(uint32_t& homing_offset) {
    return ReadRegField(ControlTable::kHomingOffset, homing_offset);
  }

  /**
   * @brief 设置归零偏移
   * @param[in] homing_offset 归零偏移
   * @return Error::kOk 成功
   *
   * @see GetHomingOffset 查看详细说明
   */
  Error SetHomingOffset(const uint32_t homing_offset) {
    return WriteRegField(ControlTable::kHomingOffset, homing_offset);
  }

  /**
   * @brief 获取运动阈值
   * @param[out] moving_threshold 运动阈值
   * @return Error::kOk 成功
   *
   * 单位: 0.229 rev/min | 范围: 0-1023
   *
   * 运动阈值用于判断舵机是否在运动：
   * - 当速度超过此阈值时，Moving(122) 置 1
   * - 当速度低于此阈值时，Moving(122) 清 0
   *
   * @note 用于检测运动状态，避免微小振动导致的误判
   */
  Error GetMovingThreshold(uint32_t& moving_threshold) {
    return ReadRegField(ControlTable::kMovingThreshold, moving_threshold);
  }

  /**
   * @brief 设置运动阈值
   * @param[in] moving_threshold 运动阈值
   * @return Error::kOk 成功
   *
   * @see GetMovingThreshold 查看详细说明
   */
  Error SetMovingThreshold(const uint32_t moving_threshold) {
    return WriteRegField(ControlTable::kMovingThreshold, moving_threshold);
  }

  /**
   * @brief 获取温度上限
   * @param[out] temperature_limit 温度上限
   * @return Error::kOk 成功
   *
   * 单位: °C | 范围: 0-100 | 初始值: 72
   *
   * 当内部温度超过此值时：
   * - Hardware Error Status(70) 的 Overheating Error bit 置 1
   * - 如果 Shutdown(63) 对应位启用，舵机会自动关断力矩
   *
   * @warning 温度过高会损坏电机，建议不超过 80°C
   * @note XL330 工作温度范围: -5°C ~ 72°C
   * @see GetPresentTemperature, GetShutdown
   */
  Error GetTemperatureLimit(uint8_t& temperature_limit) {
    return ReadRegField(ControlTable::kTemperatureLimit, temperature_limit);
  }

  /**
   * @brief 设置温度上限
   * @param[in] temperature_limit 温度上限
   * @return Error::kOk 成功
   *
   * @warning 温度过高会损坏电机，建议不超过 80°C
   * @see GetTemperatureLimit 查看详细说明
   */
  Error SetTemperatureLimit(const uint8_t temperature_limit) {
    return WriteRegField(ControlTable::kTemperatureLimit, temperature_limit);
  }

  /**
   * @brief 获取最高电压限制
   * @param[out] max_voltage_limit 最高电压限制
   * @return Error::kOk 成功
   *
   * 单位: 0.1V | 范围: 50-160 | 初始值: 160
   *
   * 当输入电压超过此值时：
   * - Hardware Error Status(70) 的 Input Voltage Error bit 置 1
   * - 如果 Shutdown(63) 对应位启用，舵机会自动关断力矩
   *
   * @note XL330 工作电压范围: 5.0V ~ 12.0V
   * @see GetPresentInputVoltage, GetMinVoltageLimit
   */
  Error GetMaxVoltageLimit(uint16_t& max_voltage_limit) {
    return ReadRegField(ControlTable::kMaxVoltageLimit, max_voltage_limit);
  }

  /**
   * @brief 设置最高电压限制
   * @param[in] max_voltage_limit 最高电压限制
   * @return Error::kOk 成功
   *
   * @see GetMaxVoltageLimit 查看详细说明
   */
  Error SetMaxVoltageLimit(const uint16_t max_voltage_limit) {
    return WriteRegField(ControlTable::kMaxVoltageLimit, max_voltage_limit);
  }

  /**
   * @brief 获取最低电压限制
   * @param[out] min_voltage_limit 最低电压限制
   * @return Error::kOk 成功
   *
   * 单位: 0.1V | 范围: 50-160 | 初始值: 50
   *
   * 当输入电压低于此值时：
   * - Hardware Error Status(70) 的 Input Voltage Error bit 置 1
   * - 如果 Shutdown(63) 对应位启用，舵机会自动关断力矩
   *
   * @see GetMaxVoltageLimit 查看详细说明
   */
  Error GetMinVoltageLimit(uint16_t& min_voltage_limit) {
    return ReadRegField(ControlTable::kMinVoltageLimit, min_voltage_limit);
  }

  /**
   * @brief 设置最低电压限制
   * @param[in] min_voltage_limit 最低电压限制
   * @return Error::kOk 成功
   *
   * @see GetMinVoltageLimit 查看详细说明
   */
  Error SetMinVoltageLimit(const uint16_t min_voltage_limit) {
    return WriteRegField(ControlTable::kMinVoltageLimit, min_voltage_limit);
  }

  /**
   * @brief 获取 PWM 上限
   * @param[out] pwm_limit PWM 上限
   * @return Error::kOk 成功
   *
   * 单位: 0.113% | 范围: 0-885 | 初始值: 885
   *
   * PWM 上限作用：
   * - 限制电机驱动器的 PWM 占空比
   * - 间接限制输出力矩和电流
   * - 所有控制模式都受此限制
   *
   * 换算示例：
   * - 885: 100% (默认)
   * - 443: 50%
   * - 177: 20%
   *
   * @note Goal PWM(100) 和 Present PWM(124) 不会超过此值
   */
  Error GetPwmLimit(uint16_t& pwm_limit) {
    return ReadRegField(ControlTable::kPwmLimit, pwm_limit);
  }

  /**
   * @brief 设置 PWM 上限
   * @param[in] pwm_limit PWM 上限
   * @return Error::kOk 成功
   *
   * @see GetPwmLimit 查看详细说明
   */
  Error SetPwmLimit(const uint16_t pwm_limit) {
    return WriteRegField(ControlTable::kPwmLimit, pwm_limit);
  }

  /**
   * @brief 获取电流上限
   * @param[out] current_limit 电流上限
   * @return Error::kOk 成功
   *
   * 单位: mA | 范围: 0-1193 | 初始值: 1193
   *
   * 电流上限作用：
   * - 限制电机输出力矩
   * - 保护电机和驱动电路
   * - Current-based Position Control Mode 的最大电流
   *
   * @warning 长时间大电流运行会导致过热
   * @note Present Current(126) 不会超过此值
   * @see GetGoalCurrent, GetPresentCurrent
   */
  Error GetCurrentLimit(uint16_t& current_limit) {
    return ReadRegField(ControlTable::kCurrentLimit, current_limit);
  }

  /**
   * @brief 设置电流上限
   * @param[in] current_limit 电流上限
   * @return Error::kOk 成功
   *
   * @warning 长时间大电流运行会导致过热
   * @see GetCurrentLimit 查看详细说明
   */
  Error SetCurrentLimit(const uint16_t current_limit) {
    return WriteRegField(ControlTable::kCurrentLimit, current_limit);
  }

  /**
   * @brief 获取速度上限
   * @param[out] velocity_limit 速度上限
   * @return Error::kOk 成功
   *
   * 单位: 0.229 rev/min | 范围: 0-2047 | 初始值: 2047
   *
   * 速度上限作用：
   * - 限制舵机运动速度
   * - Velocity Control Mode 的最大速度
   * - Position Control Mode 中 Profile Velocity 的最大值
   *
   * @note Present Velocity(128) 不会超过此值
   * @see SetGoalVelocity, SetProfileVelocity
   */
  Error GetVelocityLimit(uint32_t& velocity_limit) {
    return ReadRegField(ControlTable::kVelocityLimit, velocity_limit);
  }

  /**
   * @brief 设置速度上限
   * @param[in] velocity_limit 速度上限
   * @return Error::kOk 成功
   *
   * @see GetVelocityLimit 查看详细说明
   */
  Error SetVelocityLimit(const uint32_t velocity_limit) {
    return WriteRegField(ControlTable::kVelocityLimit, velocity_limit);
  }

  /**
   * @brief 获取位置上限
   * @param[out] max_position_limit 位置上限
   * @return Error::kOk 成功
   *
   * 单位: pulse | 范围: 0-4095 | 初始值: 4095
   *
   * 位置限制作用：
   * - 仅在 Position Control Mode(3) 有效
   * - Extended Position Control Mode(4) 不受此限制
   * - Goal Position(116) 会被限制在 [Min Position Limit, Max Position Limit] 范围内
   *
   * @note 确保 Max Position Limit >= Min Position Limit
   * @warning 如果设置不当，可能导致舵机无法移动
   * @see GetMinPositionLimit, SetGoalPosition
   */
  Error GetMaxPositionLimit(uint32_t& max_position_limit) {
    return ReadRegField(ControlTable::kMaxPositionLimit, max_position_limit);
  }

  /**
   * @brief 设置位置上限
   * @param[in] max_position_limit 位置上限
   * @return Error::kOk 成功
   *
   * @warning 如果设置不当，可能导致舵机无法移动
   * @see GetMaxPositionLimit 查看详细说明
   */
  Error SetMaxPositionLimit(const uint32_t max_position_limit) {
    return WriteRegField(ControlTable::kMaxPositionLimit, max_position_limit);
  }

  /**
   * @brief 获取位置下限
   * @param[out] min_position_limit 位置下限
   * @return Error::kOk 成功
   *
   * 单位: pulse | 范围: 0-4095 | 初始值: 0
   *
   * @see GetMaxPositionLimit 查看详细说明
   */
  Error GetMinPositionLimit(uint32_t& min_position_limit) {
    return ReadRegField(ControlTable::kMinPositionLimit, min_position_limit);
  }

  /**
   * @brief 设置位置下限
   * @param[in] min_position_limit 位置下限
   * @return Error::kOk 成功
   *
   * @see GetMinPositionLimit 查看详细说明
   */
  Error SetMinPositionLimit(const uint32_t min_position_limit) {
    return WriteRegField(ControlTable::kMinPositionLimit, min_position_limit);
  }

  /**
   * @brief 获取关断条件
   * @param[out] shutdown 关断条件位域
   * @return Error::kOk 成功
   *
   * 关断条件位域定义：
   * | Bit | 错误类型               | 说明                          |
   * |-----|------------------------|-------------------------------|
   * | 0   | Input Voltage Error    | 输入电压超出范围              |
   * | 2   | Overheating Error      | 温度超过上限                  |
   * | 3   | Motor Encoder Error    | 编码器故障                    |
   * | 4   | Electrical Shock Error | 电气冲击                      |
   * | 5   | Overload Error         | 过载（电流持续超限）          |
   *
   * 当对应位为 1 时，发生该错误会自动关断力矩（Torque Enable = 0）
   * 错误状态会记录在 Hardware Error Status(70)
   *
   * 默认值 52 (0b00110100) = Overheating + Electrical Shock + Overload
   *
   * @note 关断后需要手动清除错误并重新使能力矩
   * @see GetHardwareErrorStatus, SetTorqueEnable
   */
  Error GetShutdown(uint8_t& shutdown) {
    return ReadRegField(ControlTable::kShutdown, shutdown);
  }

  /**
   * @brief 设置关断条件
   * @param[in] shutdown 关断条件位域
   * @return Error::kOk 成功
   *
   * @note 关断后需要手动清除错误并重新使能力矩
   * @see GetShutdown 查看位域定义
   */
  Error SetShutdown(const uint8_t shutdown) {
    return WriteRegField(ControlTable::kShutdown, shutdown);
  }

  /** @} */ // 限位与保护设置组结束

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
   * @brief 获取 LED 状态
   * @param[out] dxl_led LED 状态
   * @return Error::kOk 成功
   *
   * LED 控制：
   * - 0: LED 关闭
   * - 1: LED 开启
   *
   * @note 可用于状态指示和调试
   */
  Error GetDxlLed(uint8_t& dxl_led) {
    return ReadRegField(ControlTable::kDxlLed, dxl_led);
  }

  /**
   * @brief 设置 LED 开关
   * @param[in] dxl_led LED 状态 (0: 关, 1: 开)
   * @return Error::kOk 成功
   *
   * @note 可用于状态指示和调试
   */
  Error SetDxlLed(const uint8_t dxl_led) {
    return WriteRegField(ControlTable::kDxlLed, dxl_led);
  }

  /**
   * @brief 获取状态返回级别
   * @param[out] status_return_level 状态返回级别
   * @return Error::kOk 成功
   *
   * 状态返回级别：
   * - 0: 不返回状态包（仅 PING 指令返回）
   * - 1: 仅读取指令返回状态包
   * - 2: 所有指令都返回状态包（默认）
   *
   * @note 降低返回级别可以减少总线流量，提高通信效率
   */
  Error GetStatusReturnLevel(uint8_t& status_return_level) {
    return ReadRegField(ControlTable::kStatusReturnLevel, status_return_level);
  }

  /**
   * @brief 设置状态返回级别
   * @param[in] status_return_level 状态返回级别 (0-2)
   * @return Error::kOk 成功
   *
   * @note 降低返回级别可以减少总线流量，提高通信效率
   */
  Error SetStatusReturnLevel(const uint8_t status_return_level) {
    return WriteRegField(ControlTable::kStatusReturnLevel, status_return_level);
  }

  /**
   * @brief 获取已注册指令 (R)
   * @param[out] registered_instruction 已注册指令
   * @return Error::kOk 成功
   */
  Error GetRegisteredInstruction(uint8_t& registered_instruction) {
    return ReadRegField(ControlTable::kRegisteredInstruction,
                        registered_instruction);
  }

  /**
   * @brief 设置已注册指令 (R)
   * @param[in] registered_instruction 已注册指令
   * @return Error::kOk 成功
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
   * 硬件错误位域：
   * | Bit | 错误类型               | 说明                          |
   * |-----|------------------------|-------------------------------|
   * | 0   | Input Voltage Error    | 输入电压超出范围              |
   * | 2   | Overheating Error      | 温度超过上限                  |
   * | 3   | Motor Encoder Error    | 编码器故障                    |
   * | 4   | Electrical Shock Error | 电气冲击                      |
   * | 5   | Overload Error         | 过载                          |
   *
   * 错误处理流程：
   * 1. 检查此寄存器获取错误类型
   * 2. 排除错误原因
   * 3. 写入 0 清除错误状态
   * 4. 重新使能力矩
   *
   * @note 如果 Shutdown(63) 对应位启用，发生错误会自动关断力矩
   * @see GetShutdown, SetTorqueEnable
   */
  Error GetHardwareErrorStatus(uint8_t& hardware_error_status) {
    return ReadRegField(ControlTable::kHardwareErrorStatus,
                        hardware_error_status);
  }

  /**
   * @brief 设置硬件错误状态
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

  /** @} */ // 使能与状态组结束

  //==============================================================================
  // PID 和前馈控制参数区（RAM）
  //==============================================================================
  /**
   * @name PID 和前馈控制参数
   * 配置位置和速度控制器的 PID 增益和前馈增益。
   * 参数范围: 0-16383，增益值越大响应越快但可能产生振荡。
   *
   * XL330 控制结构：
   *   Goal Position → [Position PID] → Goal Velocity → [Velocity PI] → Goal Current → [Current PI] → PWM
   * @{
   */

  /**
   * @brief 获取速度环 I 增益
   * @param[out] velocity_igain I 增益
   * @return Error::kOk 成功
   *
   * 范围: 0-16383 | 初始值: 1920
   *
   * 速度环 I 增益作用：
   * - 消除速度稳态误差
   * - 提高恒速控制精度
   *
   * @note 仅在 Velocity Control Mode 和 Position Control Mode 有效
   */
  Error GetVelocityIgain(uint16_t& velocity_igain) {
    return ReadRegField(ControlTable::kVelocityIgain, velocity_igain);
  }

  /**
   * @brief 设置速度环 I 增益
   * @param[in] velocity_igain I 增益
   * @return Error::kOk 成功
   *
   * @see GetVelocityIgain 查看详细说明
   */
  Error SetVelocityIgain(const uint16_t velocity_igain) {
    return WriteRegField(ControlTable::kVelocityIgain, velocity_igain);
  }

  /**
   * @brief 获取速度环 P 增益
   * @param[out] velocity_pgain P 增益
   * @return Error::kOk 成功
   *
   * 范围: 0-16383 | 初始值: 100
   *
   * 速度环 P 增益作用：
   * - 控制速度误差的响应
   * - 在 Velocity Control Mode 中使用
   * - 在 Position Control Mode 中作为内环控制
   *
   * @note 仅在 Velocity Control Mode 和 Position Control Mode 有效
   */
  Error GetVelocityPgain(uint16_t& velocity_pgain) {
    return ReadRegField(ControlTable::kVelocityPgain, velocity_pgain);
  }

  /**
   * @brief 设置速度环 P 增益
   * @param[in] velocity_pgain P 增益
   * @return Error::kOk 成功
   *
   * @see GetVelocityPgain 查看详细说明
   */
  Error SetVelocityPgain(const uint16_t velocity_pgain) {
    return WriteRegField(ControlTable::kVelocityPgain, velocity_pgain);
  }

  /**
   * @brief 获取位置环 D 增益
   * @param[out] position_dgain D 增益
   * @return Error::kOk 成功
   *
   * 范围: 0-16383 | 初始值: 4000 (XL330-M077), 3200 (XL330-M288)
   *
   * D 增益作用：
   * - 阻尼作用，减少超调和振荡
   * - 值越大，阻尼越强，但可能降低响应速度
   *
   * 调试建议：
   * 1. 先调好 P 增益
   * 2. 如果出现振荡，增加 D 增益
   * 3. 过大的 D 增益会使系统反应迟钝
   *
   * @note 对高频噪声敏感，编码器噪声可能影响 D 项
   */
  Error GetPositionDgain(uint16_t& position_dgain) {
    return ReadRegField(ControlTable::kPositionDgain, position_dgain);
  }

  /**
   * @brief 设置位置环 D 增益
   * @param[in] position_dgain D 增益
   * @return Error::kOk 成功
   *
   * @see GetPositionDgain 查看详细说明
   */
  Error SetPositionDgain(const uint16_t position_dgain) {
    return WriteRegField(ControlTable::kPositionDgain, position_dgain);
  }

  /**
   * @brief 获取位置环 I 增益
   * @param[out] position_igain I 增益
   * @return Error::kOk 成功
   *
   * 范围: 0-16383 | 初始值: 0
   *
   * I 增益作用：
   * - 消除稳态误差（静态位置偏差）
   * - 值过大会导致积分饱和和振荡
   * - 通常保持较小值或为 0
   *
   * 调试建议：
   * 1. 先调好 P 和 D 增益
   * 2. 如果存在稳态误差，缓慢增加 I 增益
   * 3. 观察是否出现低频振荡
   *
   * @warning I 增益过大会导致系统不稳定
   */
  Error GetPositionIgain(uint16_t& position_igain) {
    return ReadRegField(ControlTable::kPositionIgain, position_igain);
  }

  /**
   * @brief 设置位置环 I 增益
   * @param[in] position_igain I 增益
   * @return Error::kOk 成功
   *
   * @warning I 增益过大会导致系统不稳定
   * @see GetPositionIgain 查看详细说明
   */
  Error SetPositionIgain(const uint16_t position_igain) {
    return WriteRegField(ControlTable::kPositionIgain, position_igain);
  }

  /**
   * @brief 获取位置环 P 增益
   * @param[out] position_pgain P 增益
   * @return Error::kOk 成功
   *
   * 范围: 0-16383 | 初始值: 800 (XL330-M077), 640 (XL330-M288)
   *
   * P 增益作用：
   * - 控制位置误差的响应速度
   * - 值越大，响应越快，但过大会振荡
   * - 值越小，响应越慢，但更稳定
   *
   * 调试建议：
   * 1. 从较小值开始（如 100）
   * 2. 逐步增加直到响应速度满足要求
   * 3. 如果出现振荡，减小 P 增益或增大 D 增益
   * 4. 配合 Position D Gain 使用以减少超调
   *
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * @see GetPositionIgain, GetPositionDgain
   */
  Error GetPositionPgain(uint16_t& position_pgain) {
    return ReadRegField(ControlTable::kPositionPgain, position_pgain);
  }

  /**
   * @brief 设置位置环 P 增益
   * @param[in] position_pgain P 增益
   * @return Error::kOk 成功
   *
   * @see GetPositionPgain 查看详细说明
   */
  Error SetPositionPgain(const uint16_t position_pgain) {
    return WriteRegField(ControlTable::kPositionPgain, position_pgain);
  }

  /**
   * @brief 获取前馈二阶增益
   * @param[out] feedforward_2nd_gain 前馈二阶增益
   * @return Error::kOk 成功
   *
   * 范围: 0-16383 | 初始值: 0
   *
   * 前馈二阶增益（加速度前馈）：
   * - 根据目标加速度提供前馈补偿
   * - 改善加速阶段的跟踪性能
   *
   * @note 高级参数，通常保持为 0
   */
  Error GetFeedforward2ndGain(uint16_t& feedforward_2nd_gain) {
    return ReadRegField(ControlTable::kFeedforward2ndGain,
                        feedforward_2nd_gain);
  }

  /**
   * @brief 设置前馈二阶增益
   * @param[in] feedforward_2nd_gain 前馈二阶增益
   * @return Error::kOk 成功
   *
   * @note 高级参数，通常保持为 0
   */
  Error SetFeedforward2ndGain(const uint16_t feedforward_2nd_gain) {
    return WriteRegField(ControlTable::kFeedforward2ndGain,
                         feedforward_2nd_gain);
  }

  /**
   * @brief 获取前馈一阶增益
   * @param[out] feedforward_1st_gain 前馈一阶增益
   * @return Error::kOk 成功
   *
   * 范围: 0-16383 | 初始值: 0
   *
   * 前馈一阶增益（速度前馈）：
   * - 根据目标速度提供前馈补偿
   * - 减少速度跟踪误差
   * - 提高动态响应
   *
   * @note 需要根据实际负载调整
   */
  Error GetFeedforward1stGain(uint16_t& feedforward_1st_gain) {
    return ReadRegField(ControlTable::kFeedforward1stGain,
                        feedforward_1st_gain);
  }

  /**
   * @brief 设置前馈一阶增益
   * @param[in] feedforward_1st_gain 前馈一阶增益
   * @return Error::kOk 成功
   *
   * @note 需要根据实际负载调整
   */
  Error SetFeedforward1stGain(const uint16_t feedforward_1st_gain) {
    return WriteRegField(ControlTable::kFeedforward1stGain,
                         feedforward_1st_gain);
  }

  /**
   * @brief 获取总线看门狗
   * @param[out] bus_watchdog 总线看门狗时间
   * @return Error::kOk 成功
   *
   * 单位: 20ms | 范围: 0-254 | 初始值: 0
   *
   * 总线看门狗功能：
   * - 0: 禁用看门狗
   * - 非0: 启用看门狗，超时后自动关断力矩
   * - 超时时间 = bus_watchdog × 20ms
   *
   * @note 用于检测通信中断，防止失控
   */
  Error GetBusWatchdog(uint8_t& bus_watchdog) {
    return ReadRegField(ControlTable::kBusWatchdog, bus_watchdog);
  }

  /**
   * @brief 设置总线看门狗
   * @param[in] bus_watchdog 总线看门狗时间
   * @return Error::kOk 成功
   *
   * @note 用于检测通信中断，防止失控
   */
  Error SetBusWatchdog(const uint8_t bus_watchdog) {
    return WriteRegField(ControlTable::kBusWatchdog, bus_watchdog);
  }

  /** @} */ // PID 控制参数组结束

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
   * @param[out] goal_pwm 目标 PWM
   * @return Error::kOk 成功
   */
  Error GetGoalPwm(uint16_t& goal_pwm) {
    return ReadRegField(ControlTable::kGoalPwm, goal_pwm);
  }

  /**
   * @brief 设置目标 PWM (R/W)
   * @param[in] goal_pwm 目标 PWM
   * @return Error::kOk 成功
   *
   * @warning 开环控制可能导致失速或过载
   * @see GetGoalPwm 查看详细说明
   */
  Error SetGoalPwm(const uint16_t goal_pwm) {
    return WriteRegField(ControlTable::kGoalPwm, goal_pwm);
  }

  /**
   * @brief 获取目标电流 (R/W)
   * @param[out] goal_current 目标电流
   * @return Error::kOk 成功
   */
  Error GetGoalCurrent(uint16_t& goal_current) {
    return ReadRegField(ControlTable::kGoalCurrent, goal_current);
  }

  /**
   * @brief 设置目标电流 (R/W)
   * @param[in] goal_current 目标电流
   * @return Error::kOk 成功
   *
   * @see GetGoalCurrent 查看详细说明
   */
  Error SetGoalCurrent(const uint16_t goal_current) {
    return WriteRegField(ControlTable::kGoalCurrent, goal_current);
  }

  /**
   * @brief 获取目标速度 (R/W)
   * @param[out] goal_velocity 目标速度
   * @return Error::kOk 成功
   *
   * 单位: 0.229 rev/min | 范围: 0-2047 | 最大值: 2047 × 0.229 ≈ 469 rpm
   *
   * 换算示例：
   * - 100 → 22.9 rpm
   * - 200 → 45.8 rpm
   * - 1000 → 229 rpm
   *
   * 工作模式：
   *
   * 【Velocity Control Mode (1)】
   * - 控制舵机转速
   * - 正值：正转，负值：反转
   * - 受 Velocity Limit(44) 限制
   * - Profile Acceleration(108) 控制加速度
   * - 不控制位置，可无限旋转
   *
   * 【其他模式】
   * - 此寄存器无效
   *
   * @note 仅在 Velocity Control Mode 有效
   * @see SetProfileAcceleration, GetPresentVelocity, GetVelocityLimit
   */
  Error GetGoalVelocity(uint32_t& goal_velocity) {
    return ReadRegField(ControlTable::kGoalVelocity, goal_velocity);
  }

  /**
   * @brief 设置目标速度 (R/W)
   * @param[in] goal_velocity 目标速度
   * @return Error::kOk 成功
   *
   * @note 仅在 Velocity Control Mode 有效
   * @see GetGoalVelocity 查看详细说明
   */
  Error SetGoalVelocity(const uint32_t goal_velocity) {
    return WriteRegField(ControlTable::kGoalVelocity, goal_velocity);
  }

  /**
   * @brief 获取轨迹加速度
   * @param[out] profile_acceleration 轨迹加速度
   * @return Error::kOk 成功
   *
   * Profile Acceleration 在不同 Profile 类型下的作用：
   *
   * 【Velocity-based Profile (Drive Mode Bit 2 = 0)】
   * 单位: 214.577 rev/min² (近似)
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
   * - 限制: Profile Acceleration ≤ 50% of Profile Velocity(112)
   *
   * @note 在 Velocity Control Mode 中也使用此参数控制加速度
   * @see SetProfileVelocity
   */
  Error GetProfileAcceleration(uint32_t& profile_acceleration) {
    return ReadRegField(ControlTable::kProfileAcceleration,
                        profile_acceleration);
  }

  /**
   * @brief 设置轨迹加速度
   * @param[in] profile_acceleration 轨迹加速度
   * @return Error::kOk 成功
   *
   * @see GetProfileAcceleration 查看详细说明
   */
  Error SetProfileAcceleration(const uint32_t profile_acceleration) {
    return WriteRegField(ControlTable::kProfileAcceleration,
                         profile_acceleration);
  }

  /**
   * @brief 获取轨迹速度
   * @param[out] profile_velocity 轨迹速度
   * @return Error::kOk 成功
   *
   * Profile Velocity 在不同 Profile 类型下的作用：
   *
   * 【Velocity-based Profile (Drive Mode Bit 2 = 0)】
   * 单位: 0.229 rev/min
   * 作用: 设置运动的最大速度
   * - 0: 无限速度（不使用 Profile，立即到达目标）
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
   * Profile 类型选择：
   * | Profile Velocity | Profile Acceleration | Profile 类型    |
   * |------------------|----------------------|----------------|
   * | 0                | -                    | Step (无 Profile) |
   * | 非 0             | 0                    | Rectangular    |
   * | 非 0             | 非 0                 | Trapezoidal    |
   *
   * 加速时间计算（Velocity-based）：
   *   t1 = 64 × (Profile Velocity / Profile Acceleration)
   *
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * @see SetProfileAcceleration, SetGoalPosition, GetDriveMode
   */
  Error GetProfileVelocity(uint32_t& profile_velocity) {
    return ReadRegField(ControlTable::kProfileVelocity, profile_velocity);
  }

  /**
   * @brief 设置轨迹速度
   * @param[in] profile_velocity 轨迹速度
   * @return Error::kOk 成功
   *
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * @see GetProfileVelocity 查看详细说明
   */
  Error SetProfileVelocity(const uint32_t profile_velocity) {
    return WriteRegField(ControlTable::kProfileVelocity, profile_velocity);
  }

  /**
   * @brief 获取目标位置 (R/W)
   * @param[out] goal_position 目标位置
   * @return Error::kOk 成功
   */
  Error GetGoalPosition(uint32_t& goal_position) {
    return ReadRegField(ControlTable::kGoalPosition, goal_position);
  }

  /**
   * @brief 设置目标位置 (R/W)
   * @param[in] goal_position 目标位置
   * @return Error::kOk 成功
   *
   * @note 其他工作模式下此寄存器无效
   * @see GetGoalPosition 查看详细说明
   */
  Error SetGoalPosition(const uint32_t goal_position) {
    return WriteRegField(ControlTable::kGoalPosition, goal_position);
  }

  /** @} */ // 目标值设置组结束

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
   * @param[out] realtime_tick 实时时钟 (ms)
   * @return Error::kOk 成功
   */
  Error GetRealtimeTick(uint16_t& realtime_tick) {
    return ReadRegField(ControlTable::kRealtimeTick, realtime_tick);
  }

  /**
   * @brief 设置实时时钟 (R)
   * @param[in] realtime_tick 实时时钟
   * @return Error::kOk 成功
   */
  Error SetRealtimeTick(const uint16_t realtime_tick) {
    return WriteRegField(ControlTable::kRealtimeTick, realtime_tick);
  }

  /**
   * @brief 获取运动状态 (R)
   * @param[out] moving 运动状态
   * @return Error::kOk 成功
   */
  Error GetMoving(uint8_t& moving) {
    return ReadRegField(ControlTable::kMoving, moving);
  }

  /**
   * @brief 设置运动状态 (R)
   * @param[in] moving 运动状态
   * @return Error::kOk 成功
   */
  Error SetMoving(const uint8_t moving) {
    return WriteRegField(ControlTable::kMoving, moving);
  }

  /**
   * @brief 获取详细运动状态 (R)
   * @param[out] moving_status 详细运动状态位域
   * @return Error::kOk 成功
   */
  Error GetMovingStatus(uint8_t& moving_status) {
    return ReadRegField(ControlTable::kMovingStatus, moving_status);
  }

  /**
   * @brief 设置详细运动状态 (R)
   * @param[in] moving_status 详细运动状态位域
   * @return Error::kOk 成功
   */
  Error SetMovingStatus(const uint8_t moving_status) {
    return WriteRegField(ControlTable::kMovingStatus, moving_status);
  }

  /**
   * @brief 获取当前 PWM (R)
   * @param[out] present_pwm 当前 PWM
   * @return Error::kOk 成功
   */
  Error GetPresentPwm(uint16_t& present_pwm) {
    return ReadRegField(ControlTable::kPresentPwm, present_pwm);
  }

  /**
   * @brief 设置当前 PWM (R)
   * @param[in] present_pwm 当前 PWM
   * @return Error::kOk 成功
   */
  Error SetPresentPwm(const uint16_t present_pwm) {
    return WriteRegField(ControlTable::kPresentPwm, present_pwm);
  }

  /**
   * @brief 获取当前电流 (R)
   * @param[out] present_current 当前电流 (mA)
   * @return Error::kOk 成功
   */
  Error GetPresentCurrent(uint16_t& present_current) {
    return ReadRegField(ControlTable::kPresentCurrent, present_current);
  }

  /**
   * @brief 设置当前电流 (R)
   * @param[in] present_current 当前电流
   * @return Error::kOk 成功
   */
  Error SetPresentCurrent(const uint16_t present_current) {
    return WriteRegField(ControlTable::kPresentCurrent, present_current);
  }

  /**
   * @brief 获取当前速度 (R)
   * @param[out] present_velocity 当前速度
   * @return Error::kOk 成功
   *
   * 单位: 0.229 rev/min | 范围: 0-2047
   *
   * @note 通过编码器微分计算得到
   * @see SetGoalVelocity
   */
  Error GetPresentVelocity(uint32_t& present_velocity) {
    return ReadRegField(ControlTable::kPresentVelocity, present_velocity);
  }

  /**
   * @brief 设置当前速度 (R)
   * @param[in] present_velocity 当前速度
   * @return Error::kOk 成功
   */
  Error SetPresentVelocity(const uint32_t present_velocity) {
    return WriteRegField(ControlTable::kPresentVelocity, present_velocity);
  }

  /**
   * @brief 获取当前位置 (R)
   * @param[out] present_position 当前位置 (pulse)
   * @return Error::kOk 成功
   */
  Error GetPresentPosition(uint32_t& present_position) {
    return ReadRegField(ControlTable::kPresentPosition, present_position);
  }

  /**
   * @brief 设置当前位置 (R)
   * @param[in] present_position 当前位置
   * @return Error::kOk 成功
   */
  Error SetPresentPosition(const uint32_t present_position) {
    return WriteRegField(ControlTable::kPresentPosition, present_position);
  }

  /**
   * @brief 获取速度轨迹 (R)
   * @param[out] velocity_trajectory 速度轨迹
   * @return Error::kOk 成功
   *
   * 单位: 0.229 rev/min
   *
   * 速度轨迹：
   * - Profile 生成的期望速度值
   * - 速度控制器的参考输入
   *
   * @note 用于观察和调试 Profile 效果
   * @see SetProfileVelocity
   */
  Error GetVelocityTrajectory(uint32_t& velocity_trajectory) {
    return ReadRegField(ControlTable::kVelocityTrajectory, velocity_trajectory);
  }

  /**
   * @brief 设置速度轨迹 (R)
   * @param[in] velocity_trajectory 速度轨迹
   * @return Error::kOk 成功
   */
  Error SetVelocityTrajectory(const uint32_t velocity_trajectory) {
    return WriteRegField(ControlTable::kVelocityTrajectory,
                         velocity_trajectory);
  }

  /**
   * @brief 获取位置轨迹 (R)
   * @param[out] position_trajectory 位置轨迹
   * @return Error::kOk 成功
   */
  Error GetPositionTrajectory(uint32_t& position_trajectory) {
    return ReadRegField(ControlTable::kPositionTrajectory, position_trajectory);
  }

  /**
   * @brief 设置位置轨迹 (R)
   * @param[in] position_trajectory 位置轨迹
   * @return Error::kOk 成功
   */
  Error SetPositionTrajectory(const uint32_t position_trajectory) {
    return WriteRegField(ControlTable::kPositionTrajectory,
                         position_trajectory);
  }

  /**
   * @brief 获取当前输入电压 (R)
   * @param[out] present_input_voltage 当前输入电压
   * @return Error::kOk 成功
   */
  Error GetPresentInputVoltage(uint16_t& present_input_voltage) {
    return ReadRegField(ControlTable::kPresentInputVoltage,
                        present_input_voltage);
  }

  /**
   * @brief 设置当前输入电压 (R)
   * @param[in] present_input_voltage 当前输入电压
   * @return Error::kOk 成功
   */
  Error SetPresentInputVoltage(const uint16_t present_input_voltage) {
    return WriteRegField(ControlTable::kPresentInputVoltage,
                         present_input_voltage);
  }

  /**
   * @brief 获取当前温度 (R)
   * @param[out] present_temperature 当前温度 (°C)
   * @return Error::kOk 成功
   */
  Error GetPresentTemperature(uint8_t& present_temperature) {
    return ReadRegField(ControlTable::kPresentTemperature, present_temperature);
  }

  /**
   * @brief 设置当前温度 (R)
   * @param[in] present_temperature 当前温度
   * @return Error::kOk 成功
   */
  Error SetPresentTemperature(const uint8_t present_temperature) {
    return WriteRegField(ControlTable::kPresentTemperature,
                         present_temperature);
  }

  /** @} */ // 状态反馈组结束

  //==============================================================================
  // EEPROM 管理区（EEPROM）
  //==============================================================================
  /**
   * @name EEPROM 管理
   * 管理 EEPROM 的存储、加载和恢复操作。
   * EEPROM 用于保存配置参数，掉电不丢失。
   * @{
   */

  // 注意：EEPROM 管理功能通过 Protocol 接口实现
  // 具体实现请参考 ServoIdImpl, LoadEepromImpl, StoreEepromImpl, RecoveryEepromImpl

  /** @} */ // EEPROM 管理组结束

  // Protocol 接口实现（CRTP 模式）
  Error ServoIdImpl(uint8_t& id);
  Error ReturnLevelImpl(uint8_t& return_level);
  Error StatusImpl(uint8_t& status);
  Error RecoveryEepromImpl();
  Error LoadEepromImpl();
  Error StoreEepromImpl();
  Error StoreEepromImpl(const uint8_t address, const uint8_t size);

 private:
  uint8_t table_[TableBlocks::kTotal.size()] = {};
};

}  // namespace hortor::servo_slave
