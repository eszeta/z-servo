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

#include "types.h"

namespace hortor::protocol {
constexpr uint8_t Instruction::kPing;
constexpr uint8_t Instruction::kReadData;
constexpr uint8_t Instruction::kWriteData;
constexpr uint8_t Instruction::kRegWrite;
constexpr uint8_t Instruction::kAction;
constexpr uint8_t Instruction::kSyncWrite;
constexpr uint8_t Instruction::kBulkRead;
constexpr uint8_t Instruction::kReset;
}  // namespace hortor::protocol