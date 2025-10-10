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
#include "register/register_accessor.h"
#include "register/register_raw_adapter.h"
#include "types.h"

namespace hortor::protocol {

class PortTableAccessor : public reg::RegisterAccessor {
 public:
  virtual Error Init() = 0;
  virtual Error GetServoId(uint8_t& id) = 0;
  virtual Error GetStatus(uint8_t& status) = 0;
  /**
   * @brief 获取状态包返回级别
   * @return 状态包返回级别
   * 0	PING Instruction
   *    Returns the Status Packet for PING Instruction only
   *
   * 1	PING Instruction
   *    READ Instruction
   *    Returns the Status Packet for PING and READ Instruction
   *
   * 2	All Instructions
   *    Returns the Status Packet for all Instructions
   */
  virtual Error GetReturnLevel(uint8_t& return_level) = 0;

  virtual Error RecoveryEeprom() = 0;
  virtual Error LoadEeprom() = 0;
  virtual Error StoreEeprom() = 0;
  virtual Error StoreEeprom(const uint8_t address, const uint8_t size) = 0;
};
}  // namespace hortor::protocol