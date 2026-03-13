// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file slave.h
 * @brief 伺服从机（协议从机 + 舵机 + 寄存器映射，I2C/Serial 通道）
 */

#pragma once

#include <Arduino.h>

#include "error.h"
#include "protocol/channel.h"
#include "protocol/slave.h"
#include "protocol/transport_i2c.h"
#include "regmap.h"

namespace hortor::slave {

/** @brief 伺服从机模板（见下方定义） */
template <typename ServoType, typename ChannelType>
class Slave;

template <typename ServoType, typename ChannelType>
using Base = protocol::Slave<Slave<ServoType, ChannelType>, Regmap, ChannelType>;

/**
 * @brief 伺服从机：绑定舵机与寄存器映射，实现协议指令与寄存器读写
 * @tparam ServoType 舵机类型（如 Servo<...>）
 * @tparam ChannelType 协议通道类型（如 ProtocolChannel<TransportI2C>）
 */
template <typename ServoType, typename ChannelType>
class Slave : public Base<ServoType, ChannelType> {
 public:
  /** @brief 绑定的舵机实例 */
  ServoType* servo();

  /**
   * @brief 初始化从机，绑定舵机、寄存器映射与通道
   * @param servo 舵机指针
   * @param regmap 寄存器映射指针
   * @param channel 协议通道指针
   * @return 错误码
   */
  Error Init(ServoType* servo, Regmap* regmap, ChannelType* channel);

  /**
   * @brief Reset 指令后钩子（恢复 EEPROM 等）
   * @param packet 指令包
   * @param response 是否需要回复
   * @return 错误码
   */
  Error AfterResetHandlerImpl(const protocol::InstPacket& packet, const bool response);

  /**
   * @brief 写寄存器后钩子（存 EEPROM、应用配置等）
   * @param address 起始地址
   * @param data 数据指针
   * @param size 字节数
   * @return 错误码
   */
  Error AfterWriteRegsImpl(const uint8_t address, const uint8_t* data, const size_t size);

  /**
   * @brief 每拍处理完指令后的钩子（写实时状态、更新寄存器）
   * @param dt 时间间隔 [s]
   * @return 错误码
   */
  Error AfterProcessImpl(float dt);

  /** @brief 将舵机状态写回寄存器映射 */
  Error UpdateStatus();

 private:
  ServoType* servo_         = nullptr;  ///< 舵机实例
  uint16_t   realtime_tick_ = 0;        ///< 实时时钟 [ms]

  Error ApplyProtocolConfig();   ///< 从寄存器应用协议配置到基类
  Error ApplyAlignToPosition();  ///< 执行对齐到目标位置
  Error ApplyMotorConfig();      ///< 从寄存器应用电机/舵机配置
  Error UpdateMotorStatus();     ///< 将舵机状态写回寄存器
};

}  // namespace hortor::slave

namespace hortor::slave {

template <typename ServoType, typename ChannelType>
ServoType* Slave<ServoType, ChannelType>::servo() {
  return servo_;
}

template <typename ServoType, typename ChannelType>
Error Slave<ServoType, ChannelType>::Init(ServoType* servo, Regmap* regmap, ChannelType* channel) {
  CHECK(Base<ServoType, ChannelType>::Init(regmap, channel));
  servo_ = servo;
  CHECK(ApplyProtocolConfig());
  CHECK(ApplyMotorConfig());
  CHECK(UpdateMotorStatus());
  return Error::kOk;
}

template <typename ServoType, typename ChannelType>
Error Slave<ServoType, ChannelType>::AfterResetHandlerImpl(const protocol::InstPacket& packet,
                                                           const bool                  response) {
  CHECK(this->regmap_->RecoveryEeprom());
  return Error::kOk;
}

template <typename ServoType, typename ChannelType>
Error Slave<ServoType, ChannelType>::AfterWriteRegsImpl(const uint8_t  address,
                                                        const uint8_t* data,
                                                        const size_t   size) {
  if (TableBlocks::kEeprom::InBlock(address, size)) {
    CHECK(this->regmap_->StoreEeprom());
  }
  if (TableBlocks::kAlign::InBlock(address, size)) {
    CHECK(ApplyAlignToPosition());
  }
  CHECK(ApplyProtocolConfig());
  CHECK(ApplyMotorConfig());
  CHECK(UpdateMotorStatus());
  return Error::kOk;
}

template <typename ServoType, typename ChannelType>
Error Slave<ServoType, ChannelType>::AfterProcessImpl(float dt) {
  realtime_tick_ += dt * 1000;
  this->regmap_->WriteRealtimeTick(realtime_tick_);
  CHECK(UpdateMotorStatus());
  return UpdateStatus();
}

template <typename ServoType, typename ChannelType>
Error Slave<ServoType, ChannelType>::UpdateStatus() {
  const auto hardware_error         = servo_->hardware_error_status();
  this->status_.input_voltage_error = hardware_error.input_voltage_error;
  this->status_.angle_limit_error   = hardware_error.angle_limit_error;
  this->status_.overheating_error   = hardware_error.overheating_error;
  this->status_.range_error         = hardware_error.range_error;
  this->status_.overload_error      = hardware_error.overload_error;
  return Error::kOk;
}

template <typename ServoType, typename ChannelType>
Error Slave<ServoType, ChannelType>::ApplyProtocolConfig() {
  this->set_id(this->regmap_->ReadId());
  this->set_return_level(this->regmap_->ReadStatusReturnLevel());
  return Error::kOk;
}

template <typename ServoType, typename ChannelType>
Error Slave<ServoType, ChannelType>::ApplyAlignToPosition() {
  const auto align_to_position = this->regmap_->ReadAlignToPosition();
  if (align_to_position) {
    servo_->AlignToPosition(align_to_position);
    this->regmap_->WriteAlignToPosition(0);
  }
  return Error::kOk;
}

template <typename ServoType, typename ChannelType>
Error Slave<ServoType, ChannelType>::ApplyMotorConfig() {
  const auto drive_mode = this->regmap_->ReadDriveMode();
  servo_->set_drive_mode(drive_mode);

  const auto operating_mode = this->regmap_->ReadOperatingMode();
  servo_->set_operating_mode(operating_mode);

  const auto shutdown = this->regmap_->ReadShutdown();
  servo_->set_shutdown(shutdown);

  const auto homing_offset = this->regmap_->ReadHomingOffset();
  servo_->set_homing_offset(homing_offset);

  const auto moving_threshold = this->regmap_->ReadMovingThreshold();
  servo_->set_moving_threshold(moving_threshold);

  const auto temperature_limit = this->regmap_->ReadTemperatureLimit();
  servo_->set_temperature_limit(temperature_limit);

  const auto max_voltage_limit = this->regmap_->ReadMaxVoltageLimit();
  servo_->set_max_voltage_limit(max_voltage_limit);

  const auto min_voltage_limit = this->regmap_->ReadMinVoltageLimit();
  servo_->set_min_voltage_limit(min_voltage_limit);

  const auto pwm_limit = this->regmap_->ReadPwmLimit();
  servo_->set_pwm_limit(pwm_limit);

  const auto current_limit = this->regmap_->ReadCurrentLimit();
  servo_->set_current_limit(current_limit);

  const auto velocity_limit = this->regmap_->ReadVelocityLimit();
  servo_->set_velocity_limit(velocity_limit);

  const auto max_position_limit = this->regmap_->ReadMaxPositionLimit();
  servo_->set_max_position_limit(max_position_limit);

  const auto min_position_limit = this->regmap_->ReadMinPositionLimit();
  servo_->set_min_position_limit(min_position_limit);

  const auto protection_time = this->regmap_->ReadProtectionTime();
  servo_->set_protection_time(protection_time);

  const auto vi = this->regmap_->ReadVelocityIGain();
  const auto vp = this->regmap_->ReadVelocityPGain();
  servo_->set_velocity_pid(vp, vi, 0);

  const auto pi = this->regmap_->ReadPositionIGain();
  const auto pp = this->regmap_->ReadPositionPGain();
  const auto pd = this->regmap_->ReadPositionDGain();
  servo_->set_position_pid(pp, pi, pd);

  const auto feedforward_2nd_gain = this->regmap_->ReadFeedforward2ndGain();
  servo_->set_feedforward_2nd_gain(feedforward_2nd_gain);

  const auto feedforward_1st_gain = this->regmap_->ReadFeedforward1stGain();
  servo_->set_feedforward_1st_gain(feedforward_1st_gain);

  const auto profile_acceleration = this->regmap_->ReadProfileAcceleration();
  servo_->set_profile_acceleration(profile_acceleration);

  const auto profile_velocity = this->regmap_->ReadProfileVelocity();
  servo_->set_profile_velocity(profile_velocity);

  const auto torque_enable = this->regmap_->ReadTorqueEnable();
  servo_->set_torque_enable(torque_enable);

  const auto hardware_error_status = this->regmap_->ReadHardwareErrorStatus();
  servo_->set_hardware_error_status(hardware_error_status);

  const auto goal_pwm = this->regmap_->ReadGoalPwm();
  servo_->set_goal_pwm(goal_pwm);

  const auto goal_current = this->regmap_->ReadGoalCurrent();
  servo_->set_goal_current(goal_current);

  const auto goal_velocity = this->regmap_->ReadGoalVelocity();
  servo_->set_goal_velocity(goal_velocity);

  const auto goal_position = this->regmap_->ReadGoalPosition();
  servo_->set_goal_position(goal_position);
  return Error::kOk;
}

template <typename ServoType, typename ChannelType>
Error Slave<ServoType, ChannelType>::UpdateMotorStatus() {
  const auto torque_enable = servo_->torque_enable();
  this->regmap_->WriteTorqueEnable(torque_enable);

  const auto hardware_error_status = servo_->hardware_error_status_value();
  this->regmap_->WriteHardwareErrorStatus(hardware_error_status);

  const auto moving = servo_->moving();
  this->regmap_->WriteMoving(moving);

  const auto moving_status = servo_->moving_status_value();
  this->regmap_->WriteMovingStatus(moving_status);

  const auto present_position = servo_->present_position();
  this->regmap_->WritePresentPosition(present_position);

  const auto present_velocity = servo_->present_velocity();
  this->regmap_->WritePresentVelocity(present_velocity);

  const auto present_current = servo_->present_current();
  this->regmap_->WritePresentCurrent(present_current);

  const auto present_input_voltage = servo_->present_input_voltage();
  this->regmap_->WritePresentInputVoltage(present_input_voltage);

  const auto present_temperature = servo_->present_temperature();
  this->regmap_->WritePresentTemperature(present_temperature);

  const auto present_pwm = servo_->present_pwm();
  this->regmap_->WritePresentPwm(present_pwm);

  const auto velocity_trajectory = servo_->velocity_trajectory();
  this->regmap_->WriteVelocityTrajectory(velocity_trajectory);

  const auto position_trajectory = servo_->position_trajectory();
  this->regmap_->WritePositionTrajectory(position_trajectory);

  return Error::kOk;
}

}  // namespace hortor::slave
