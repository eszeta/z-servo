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

#include "hortor.h"
#include "protocol/i2c_port_handler.h"
#include "protocol/slave.h"
#include "regmap.h"

namespace hortor::servo_slave {
template <typename ServoType>
class Slave : public protocol::
                  Slave<Slave<ServoType>, RegMap, protocol::I2cPortHandler> {
 public:
  ServoType& GetServo() { return *servo_; }
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
    this->SetId(this->regmap_.GetId());
    this->SetReturnLevel(this->regmap_.GetStatusReturnLevel());
    this->SetStatus(0);
    return Error::kOk;
  }

  Error ResetImpl() {
    this->regmap_.RecoveryEeprom();
    return Error::kOk;
  }

  Error WriteRegsImpl(const uint8_t address,
                      const uint8_t* data,
                      const size_t size) {
    if (TableBlocks::kEeprom.InBlock(address, size)) {
      CHECK(this->regmap_.StoreEeprom());
    }
    CHECK(ApplyProtocolConfig());
    CHECK(ApplyMotorConfig());
    CHECK(UpdateMotorStatus());
    return Error::kOk;
  }

  Error ResponseImpl(const uint8_t reply_idx,
                     const uint8_t* parameter,
                     const size_t parameter_size) {
    this->SetStatus(0);
    return Error::kOk;
  }

  Error ProcessImpl(float dt) {
    realtime_tick_ += dt * 1000;
    this->regmap_.SetRealtimeTick(realtime_tick_);
    return UpdateMotorStatus();
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
    this->SetId(this->regmap_.GetId());
    this->SetReturnLevel(this->regmap_.GetStatusReturnLevel());
    return Error::kOk;
  }
  /**
   * @brief 同步电机参数
   * @return 错误码
   */
  Error ApplyMotorConfig() {
    this->servo_->SetDriveMode(this->regmap_.GetDriveMode());
    this->servo_->SetOperatingMode(this->regmap_.GetOperatingMode());
    // this->regmap_.GetHomingOffset();
    // this->regmap_.GetMovingThreshold();
    // this->regmap_.GetTemperatureLimit();
    // this->regmap_.GetMaxVoltageLimit();
    // this->regmap_.GetMinVoltageLimit();
    // this->regmap_.GetPwmLimit();
    // this->regmap_.GetCurrentLimit();
    // this->regmap_.GetVelocityLimit();
    this->servo_->SetMaxPositionLimit(this->regmap_.GetMaxPositionLimit());
    this->servo_->SetMinPositionLimit(this->regmap_.GetMinPositionLimit());
    // this->regmap_.GetShutdown();
    this->servo_->SetTorqueEnable(this->regmap_.GetTorqueEnable());

    const auto vi = this->regmap_.GetVelocityIgain();
    const auto vp = this->regmap_.GetVelocityPgain();
    this->servo_->SetVelocityPid(vp, vi, 0);

    const auto pi = this->regmap_.GetPositionIgain();
    const auto pp = this->regmap_.GetPositionPgain();
    const auto pd = this->regmap_.GetPositionDgain();
    this->servo_->SetPositionPid(pp, pi, pd);

    this->servo_->SetFeedforward2ndGain(this->regmap_.GetFeedforward2ndGain());
    this->servo_->SetFeedforward1stGain(this->regmap_.GetFeedforward1stGain());
    this->servo_->SetGoalPwm(this->regmap_.GetGoalPwm());
    // this->regmap_.GetGoalCurrent();
    // this->regmap_.GetGoalVelocity();
    this->servo_->SetProfileAcceleration(
        this->regmap_.GetProfileAcceleration());
    this->servo_->SetProfileVelocity(this->regmap_.GetProfileVelocity());
    this->servo_->SetGoalPosition(this->regmap_.GetGoalPosition());
    return Error::kOk;
  }

  Error UpdateMotorStatus() {
    this->regmap_.SetMovingStatus(this->servo_->GetMovingStatusValue());
    this->regmap_.SetPresentPosition(this->servo_->GetPresentPosition());
    this->regmap_.SetPresentVelocity(this->servo_->GetPresentVelocity());
    this->regmap_.SetPresentCurrent(this->servo_->GetPresentCurrent());
    this->regmap_.SetPresentInputVoltage(
        this->servo_->GetPresentInputVoltage());
    this->regmap_.SetPresentTemperature(this->servo_->GetPresentTemperature());
    this->regmap_.SetPresentPwm(this->servo_->GetPresentPwm());
    this->regmap_.SetPositionTrajectory(this->servo_->GetPositionTrajectory());
    this->regmap_.SetVelocityTrajectory(this->servo_->GetVelocityTrajectory());
    return Error::kOk;
  }
};

}  // namespace hortor::servo_slave
