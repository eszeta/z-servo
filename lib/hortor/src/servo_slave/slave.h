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
    CHECK(SyncSlaveParameters());
    CHECK(SyncMotorParameters());
    return Error::kOk;
  }

  Error ResponseImpl(const uint8_t reply_idx,
                     const uint8_t* parameter,
                     const size_t parameter_size) {
    this->SetStatus(0);
    return Error::kOk;
  }

 private:
  /**
   * @brief 伺服电机
   */
  ServoType* servo_ = nullptr;
  /**
   * @brief 同步从机参数
   */
  Error SyncSlaveParameters() {
    this->SetId(this->regmap_.GetId());
    this->SetReturnLevel(this->regmap_.GetStatusReturnLevel());
    return Error::kOk;
  }
  /**
   * @brief 同步电机参数
   * @return 错误码
   */
  Error SyncMotorParameters() {
    this->regmap_.GetVelocityIgain();
    return Error::kOk;
  }
};

}  // namespace hortor::servo_slave
