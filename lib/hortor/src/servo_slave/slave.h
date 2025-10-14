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

class Slave : public protocol::Slave<Slave, RegMap, protocol::I2cPortHandler> {
 public:
  Error Init() {
    CHECK((protocol::Slave<Slave, RegMap, protocol::I2cPortHandler>::Init()));
    uint8_t id = 0;
    CHECK(regmap_.GetId(id));
    uint8_t return_level = 0;
    CHECK(regmap_.GetStatusReturnLevel(return_level));
    SetId(id);
    SetReturnLevel(return_level);
    SetStatus(0);
    return Error::kOk;
  }

  Error ResetImpl() {
    regmap_.RecoveryEeprom();
    return Error::kOk;
  }

  Error WriteRegsImpl(const uint8_t address,
                      const uint8_t* data,
                      const size_t size) {
    return Error::kOk;
  }

  Error ResponseImpl(const uint8_t reply_idx,
                     const uint8_t* parameter,
                     const size_t parameter_size) {
    SetStatus(0);
    return Error::kOk;
  }
};

}  // namespace hortor::servo_slave
