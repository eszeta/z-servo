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

#include "regmap/reg_field.h"
#include "servo/types.h"

namespace hortor::protocol {
/**
 * @brief 指令包索引
 */
namespace PacketIndex {
constexpr uint8_t kHeader1 = 0;
constexpr uint8_t kHeader2 = 1;
constexpr uint8_t kId = 2;
constexpr uint8_t kLength = 3;
constexpr uint8_t kInstruction = 4;
constexpr uint8_t kError = 4;
constexpr uint8_t kParameter = 5;
};  // namespace PacketIndex

namespace Instruction {
enum : uint8_t {
  kPing = 0x01,
  kReadData = 0x02,
  kWriteData = 0x03,
  kRegWrite = 0x04,
  kAction = 0x05,
  kReset = 0x06,
  kReboot = 0x08,
  kSyncWrite = 0x82,
  kBulkRead = 0x92,
};
}  // namespace Instruction

constexpr uint8_t kBroadcastId = 0xfe;

template <typename T = uint8_t>
struct __packed ControlTableItem {
  const regmap::RegField<T> reg;
  const T default_value;

  constexpr ControlTableItem(const uint8_t address,
                             const uint8_t shift,
                             const uint8_t bits,
                             const T default_value)
      : reg(address, shift, bits), default_value(default_value) {}
};

struct ControlTableBlock {
  const uint8_t begin;
  const uint8_t end;

  constexpr uint8_t size() const { return end - begin; }
  constexpr bool InBlock(const uint8_t address, const uint8_t size) const {
    return address < end && address + size > begin;
  }
};

}  // namespace hortor::protocol