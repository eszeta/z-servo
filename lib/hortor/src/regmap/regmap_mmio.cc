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

#include "regmap_mmio.h"

#include "regmap.h"
#include "servo/types.h"

namespace hortor::regmap {

Error RegMapMmio::Init(uint8_t* regs, const size_t size) {
  if (!regs || size == 0) {
    return Error::kInvalidParameter;
  }
  regs_ = regs;
  size_ = size;
  return Error::kOk;
}

Error RegMapMmio::WriteBytesImpl(const uint8_t address,
                                 const uint8_t* data,
                                 const size_t size) {
  if (address + size > size_) {
    return Error::kInvalidParameter;
  }
  for (size_t i = 0; i < size; ++i) {
    regs_[address + i] = data[i];
  }
  return Error::kOk;
}

Error RegMapMmio::ReadBytesImpl(const uint8_t address,
                                const size_t size,
                                uint8_t* data) {
  if (address + size > size_) {
    return Error::kInvalidParameter;
  }
  for (size_t i = 0; i < size; ++i) {
    data[i] = regs_[address + i];
  }
  return Error::kOk;
}

}  // namespace hortor::regmap
