// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include "regmap_mmio.h"

#include "regmap.h"
#include "base/types.h"

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
