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
class Slave : public protocol::
                  Slave<Slave<ServoType>, RegMap, protocol::I2cPortHandler> {
 public:
  ServoType* GetServo() { return servo_; }
  /**
   * @brief 链接伺服电机
   * @param servo 伺服电机
   * @return 错误码
   */
  Error LinkServo(ServoType* servo) {
    servo_ = servo;
    return Error::kOk;
  }

  Error Init() {
    CHECK((protocol::Slave<Slave, RegMap, protocol::I2cPortHandler>::Init()));
    CHECK(this->ApplyProtocolConfig());
    CHECK(this->ApplyMotorConfig());
    CHECK(this->UpdateMotorStatus());
    return Error::kOk;
  }

  Error ResetImpl() {
    this->regmap_->RecoveryEeprom();
    return Error::kOk;
  }

  Error WriteRegsImpl(const uint8_t address,
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

  Error ProcessImpl(float dt) {
    realtime_tick_ += dt * 1000;
    this->regmap_->SetRealtimeTick(realtime_tick_);
    CHECK(UpdateMotorStatus());
    return UpdateStatus();
  }

  Error UpdateStatus() {
    const auto hardware_error = this->servo_->GetHardwareErrorStatus();
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
    this->SetId(this->regmap_->GetId());
    this->SetReturnLevel(this->regmap_->GetStatusReturnLevel());
    return Error::kOk;
  }

  Error ApplySetToCenter() {
    if (this->regmap_->GetAlignToPosition()) {
      const auto align_to_position = this->regmap_->GetAlignToPosition();
      this->servo_->AlignToPosition(align_to_position);
      this->regmap_->SetAlignToPosition(0);
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
    const auto drive_mode = this->regmap_->GetDriveMode();
    this->servo_->SetDriveMode(drive_mode);

    const auto operating_mode = this->regmap_->GetOperatingMode();
    this->servo_->SetOperatingMode(operating_mode);

    const auto shutdown = this->regmap_->GetShutdown();
    this->servo_->SetShutdown(shutdown);

    //==============================================================================
    // 位置配置组
    //==============================================================================
    const auto homing_offset = this->regmap_->GetHomingOffset();
    this->servo_->SetHomingOffset(homing_offset);

    const auto moving_threshold = this->regmap_->GetMovingThreshold();
    this->servo_->SetMovingThreshold(moving_threshold);

    //==============================================================================
    // 保护限制组
    //==============================================================================
    const auto temperature_limit = this->regmap_->GetTemperatureLimit();
    this->servo_->SetTemperatureLimit(temperature_limit);

    const auto max_voltage_limit = this->regmap_->GetMaxVoltageLimit();
    this->servo_->SetMaxVoltageLimit(max_voltage_limit);

    const auto min_voltage_limit = this->regmap_->GetMinVoltageLimit();
    this->servo_->SetMinVoltageLimit(min_voltage_limit);

    const auto pwm_limit = this->regmap_->GetPwmLimit();
    this->servo_->SetPwmLimit(pwm_limit);

    const auto current_limit = this->regmap_->GetCurrentLimit();
    this->servo_->SetCurrentLimit(current_limit);

    const auto velocity_limit = this->regmap_->GetVelocityLimit();
    this->servo_->SetVelocityLimit(velocity_limit);

    const auto max_position_limit = this->regmap_->GetMaxPositionLimit();
    this->servo_->SetMaxPositionLimit(max_position_limit);

    const auto min_position_limit = this->regmap_->GetMinPositionLimit();
    this->servo_->SetMinPositionLimit(min_position_limit);

    const auto protection_time = this->regmap_->GetProtectionTime();
    this->servo_->SetProtectionTime(protection_time);

    //==============================================================================
    // PID 参数组
    //==============================================================================
    const auto vi = this->regmap_->GetVelocityIgain();
    const auto vp = this->regmap_->GetVelocityPgain();
    this->servo_->SetVelocityPid(vp, vi, 0);

    const auto pi = this->regmap_->GetPositionIgain();
    const auto pp = this->regmap_->GetPositionPgain();
    const auto pd = this->regmap_->GetPositionDgain();
    this->servo_->SetPositionPid(pp, pi, pd);

    const auto feedforward_2nd_gain = this->regmap_->GetFeedforward2ndGain();
    this->servo_->SetFeedforward2ndGain(feedforward_2nd_gain);

    const auto feedforward_1st_gain = this->regmap_->GetFeedforward1stGain();
    this->servo_->SetFeedforward1stGain(this->regmap_->GetFeedforward1stGain());

    //==============================================================================
    // 轨迹配置组
    //==============================================================================
    const auto profile_acceleration = this->regmap_->GetProfileAcceleration();
    this->servo_->SetProfileAcceleration(profile_acceleration);

    const auto profile_velocity = this->regmap_->GetProfileVelocity();
    this->servo_->SetProfileVelocity(profile_velocity);

    //==============================================================================
    // 控制命令组
    //==============================================================================
    const auto torque_enable = this->regmap_->GetTorqueEnable();
    this->servo_->SetTorqueEnable(torque_enable);

    const auto hardware_error_status = this->regmap_->GetHardwareErrorStatus();
    this->servo_->SetHardwareErrorStatus(hardware_error_status);

    //==============================================================================
    // 目标值组
    //==============================================================================
    const auto goal_pwm = this->regmap_->GetGoalPwm();
    this->servo_->SetGoalPwm(goal_pwm);

    const auto goal_current = this->regmap_->GetGoalCurrent();
    this->servo_->SetGoalCurrent(goal_current);

    const auto goal_velocity = this->regmap_->GetGoalVelocity();
    this->servo_->SetGoalVelocity(goal_velocity);

    const auto goal_position = this->regmap_->GetGoalPosition();
    this->servo_->SetGoalPosition(goal_position);
    return Error::kOk;
  }

  Error UpdateMotorStatus() {
    const auto torque_enable = this->servo_->GetTorqueEnable();
    this->regmap_->SetTorqueEnable(torque_enable);

    const auto hardware_error_status =
        this->servo_->GetHardwareErrorStatusValue();
    this->regmap_->SetHardwareErrorStatus(hardware_error_status);

    const auto moving = this->servo_->GetMoving();
    this->regmap_->SetMoving(moving);

    const auto moving_status = this->servo_->GetMovingStatusValue();
    this->regmap_->SetMovingStatus(moving_status);

    const auto present_position = this->servo_->GetPresentPosition();
    this->regmap_->SetPresentPosition(present_position);

    const auto present_velocity = this->servo_->GetPresentVelocity();
    this->regmap_->SetPresentVelocity(present_velocity);

    const auto present_current = this->servo_->GetPresentCurrent();
    this->regmap_->SetPresentCurrent(present_current);

    const auto present_input_voltage = this->servo_->GetPresentInputVoltage();
    this->regmap_->SetPresentInputVoltage(present_input_voltage);

    const auto present_temperature = this->servo_->GetPresentTemperature();
    this->regmap_->SetPresentTemperature(present_temperature);

    const auto present_pwm = this->servo_->GetPresentPwm();
    this->regmap_->SetPresentPwm(present_pwm);

    const auto position_trajectory = this->servo_->GetPositionTrajectory();
    this->regmap_->SetPositionTrajectory(position_trajectory);

    const auto velocity_trajectory = this->servo_->GetVelocityTrajectory();
    this->regmap_->SetVelocityTrajectory(velocity_trajectory);
    return Error::kOk;
  }
};

}  // namespace hortor::servo_slave
