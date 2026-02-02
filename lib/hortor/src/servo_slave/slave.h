// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "hortor.h"
#include "protocol/i2c_port_handler.h"
#include "protocol/slave.h"
#include "regmap.h"

namespace hortor::servo_slave {
template <typename ServoType>
class Slave;

template <typename ServoType>
using SlaveBase =
    protocol::Slave<Slave<ServoType>, RegMap, protocol::I2cPortHandler>;

template <typename ServoType>
class Slave : public SlaveBase<ServoType> {
 public:
  ServoType* servo() { return servo_; }
  /**
   * @brief 链接伺服电机
   * @param servo 伺服电机
   * @return 错误码
   */
  Error set_servo(ServoType* servo) {
    servo_ = servo;
    return Error::kOk;
  }

  Error Init() {
    CHECK((SlaveBase<ServoType>::Init()));
    CHECK(this->ApplyProtocolConfig());
    CHECK(this->ApplyMotorConfig());
    CHECK(this->UpdateMotorStatus());
    return Error::kOk;
  }

  Error AfterResetHandlerImpl(const protocol::InstPacket& packet,
                              const bool response) {
    CHECK(this->regmap_->RecoveryEeprom());
    return Error::kOk;
  }

  Error AfterWriteRegsImpl(const uint8_t address,
                           const uint8_t* data,
                           const size_t size) {
    if (TableBlocks::kEeprom.InBlock(address, size)) {
      CHECK(this->regmap_->StoreEeprom());
    }
    if (TableBlocks::kSetToCenter.InBlock(address, size)) {
      CHECK(ApplySetToCenter());
    }
    CHECK(ApplyProtocolConfig());
    CHECK(ApplyMotorConfig());
    CHECK(UpdateMotorStatus());
    return Error::kOk;
  }

  Error AfterProcessImpl(float dt) {
    realtime_tick_ += dt * 1000;
    this->regmap_->WriteRealtimeTick(realtime_tick_);
    CHECK(UpdateMotorStatus());
    return UpdateStatus();
  }

  Error UpdateStatus() {
    const auto hardware_error = this->servo_->hardware_error_status();
    this->status_.input_voltage_error = hardware_error.input_voltage_error;
    this->status_.angle_limit_error = hardware_error.angle_limit_error;
    this->status_.overheating_error = hardware_error.overheating_error;
    this->status_.range_error = hardware_error.range_error;
    this->status_.overload_error = hardware_error.overload_error;
    return Error::kOk;
  }

 private:
  /**
   * @brief 伺服电机
   */
  ServoType* servo_ = nullptr;

  uint16_t realtime_tick_ = 0;
  /**
   * @brief 同步从机参数
   */
  Error ApplyProtocolConfig() {
    this->set_id(this->regmap_->ReadId());
    this->set_return_level(this->regmap_->ReadStatusReturnLevel());
    return Error::kOk;
  }

  Error ApplySetToCenter() {
    if (this->regmap_->ReadAlignToPosition()) {
      const auto align_to_position = this->regmap_->ReadAlignToPosition();
      this->servo_->AlignToPosition(align_to_position);
      this->regmap_->WriteAlignToPosition(0);
    }
    return Error::kOk;
  }

  /**
   * @brief 同步电机参数
   * @return 错误码
   */
  Error ApplyMotorConfig() {
    //==============================================================================
    // 运行模式组
    //==============================================================================
    const auto drive_mode = this->regmap_->ReadDriveMode();
    this->servo_->set_drive_mode(drive_mode);

    const auto operating_mode = this->regmap_->ReadOperatingMode();
    this->servo_->set_operating_mode(operating_mode);

    const auto shutdown = this->regmap_->ReadShutdown();
    this->servo_->set_shutdown(shutdown);

    //==============================================================================
    // 位置配置组
    //==============================================================================
    const auto homing_offset = this->regmap_->ReadHomingOffset();
    this->servo_->set_homing_offset(homing_offset);

    const auto moving_threshold = this->regmap_->ReadMovingThreshold();
    this->servo_->set_moving_threshold(moving_threshold);

    //==============================================================================
    // 保护限制组
    //==============================================================================
    const auto temperature_limit = this->regmap_->ReadTemperatureLimit();
    this->servo_->set_temperature_limit(temperature_limit);

    const auto max_voltage_limit = this->regmap_->ReadMaxVoltageLimit();
    this->servo_->set_max_voltage_limit(max_voltage_limit);

    const auto min_voltage_limit = this->regmap_->ReadMinVoltageLimit();
    this->servo_->set_min_voltage_limit(min_voltage_limit);

    const auto pwm_limit = this->regmap_->ReadPwmLimit();
    this->servo_->set_pwm_limit(pwm_limit);

    const auto current_limit = this->regmap_->ReadCurrentLimit();
    this->servo_->set_current_limit(current_limit);

    const auto velocity_limit = this->regmap_->ReadVelocityLimit();
    this->servo_->set_velocity_limit(velocity_limit);

    const auto max_position_limit = this->regmap_->ReadMaxPositionLimit();
    this->servo_->set_max_position_limit(max_position_limit);

    const auto min_position_limit = this->regmap_->ReadMinPositionLimit();
    this->servo_->set_min_position_limit(min_position_limit);

    const auto protection_time = this->regmap_->ReadProtectionTime();
    this->servo_->set_protection_time(protection_time);

    //==============================================================================
    // PID 参数组
    //==============================================================================
    const auto vi = this->regmap_->ReadVelocityIgain();
    const auto vp = this->regmap_->ReadVelocityPgain();
    this->servo_->set_velocity_pid(vp, vi, 0);

    const auto pi = this->regmap_->ReadPositionIgain();
    const auto pp = this->regmap_->ReadPositionPgain();
    const auto pd = this->regmap_->ReadPositionDgain();
    this->servo_->set_position_pid(pp, pi, pd);

    const auto feedforward_2nd_gain = this->regmap_->ReadFeedforward2ndGain();
    this->servo_->set_feedforward_2nd_gain(feedforward_2nd_gain);

    const auto feedforward_1st_gain = this->regmap_->ReadFeedforward1stGain();
    this->servo_->set_feedforward_1st_gain(
        this->regmap_->ReadFeedforward1stGain());

    //==============================================================================
    // 轨迹配置组
    //==============================================================================
    const auto profile_acceleration = this->regmap_->ReadProfileAcceleration();
    this->servo_->set_profile_acceleration(profile_acceleration);

    const auto profile_velocity = this->regmap_->ReadProfileVelocity();
    this->servo_->set_profile_velocity(profile_velocity);

    //==============================================================================
    // 控制命令组
    //==============================================================================
    const auto torque_enable = this->regmap_->ReadTorqueEnable();
    this->servo_->set_torque_enable(torque_enable);

    const auto hardware_error_status = this->regmap_->ReadHardwareErrorStatus();
    this->servo_->set_hardware_error_status(hardware_error_status);

    //==============================================================================
    // 目标值组
    //==============================================================================
    const auto goal_pwm = this->regmap_->ReadGoalPwm();
    this->servo_->set_goal_pwm(goal_pwm);

    const auto goal_current = this->regmap_->ReadGoalCurrent();
    this->servo_->set_goal_current(goal_current);

    const auto goal_velocity = this->regmap_->ReadGoalVelocity();
    this->servo_->set_goal_velocity(goal_velocity);

    const auto goal_position = this->regmap_->ReadGoalPosition();
    this->servo_->set_goal_position(goal_position);
    return Error::kOk;
  }

  Error UpdateMotorStatus() {
    const auto torque_enable = this->servo_->torque_enable();
    this->regmap_->WriteTorqueEnable(torque_enable);

    const auto hardware_error_status =
        this->servo_->hardware_error_status_value();
    this->regmap_->WriteHardwareErrorStatus(hardware_error_status);

    const auto moving = this->servo_->moving();
    this->regmap_->WriteMoving(moving);

    const auto moving_status = this->servo_->moving_status_value();
    this->regmap_->WriteMovingStatus(moving_status);

    const auto present_position = this->servo_->present_position();
    this->regmap_->WritePresentPosition(present_position);

    const auto present_velocity = this->servo_->present_velocity();
    this->regmap_->WritePresentVelocity(present_velocity);

    const auto present_current = this->servo_->present_current();
    this->regmap_->WritePresentCurrent(present_current);

    const auto present_input_voltage = this->servo_->present_input_voltage();
    this->regmap_->WritePresentInputVoltage(present_input_voltage);

    const auto present_temperature = this->servo_->present_temperature();
    this->regmap_->WritePresentTemperature(present_temperature);

    const auto present_pwm = this->servo_->present_pwm();
    this->regmap_->WritePresentPwm(present_pwm);

    const auto position_trajectory = this->servo_->position_trajectory();
    this->regmap_->WritePositionTrajectory(position_trajectory);

    const auto velocity_trajectory = this->servo_->velocity_trajectory();
    this->regmap_->WriteVelocityTrajectory(velocity_trajectory);
    return Error::kOk;
  }
};

}  // namespace hortor::servo_slave
