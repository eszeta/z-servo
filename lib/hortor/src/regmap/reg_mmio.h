// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Wire.h>

#include "base/types.h"
#include "hortor.h"
#include "regmap.h"

namespace hortor::regmap {

/**
 * @brief 内存映射寄存器访问实现
 *
 * 用于直接访问内存映射的寄存器区域。
 */
class RegMmio : public hortor::Noncopyable {
 public:
  Error Init(uint8_t* regs, const size_t size);
  Error Write(const uint8_t address, const uint8_t* data, const size_t size);
  Error Read(const uint8_t address, const size_t size, uint8_t* data);

 protected:
  uint8_t* regs_{};
  size_t   size_{};
};

}  // namespace hortor::regmap

namespace hortor::regmap {

inline Error RegMmio::Init(uint8_t* regs, const size_t size) {
  VERIFY(regs && size != 0, Error::kInvalidArg);
  regs_ = regs;
  size_ = size;
  return Error::kOk;
}

inline Error RegMmio::Write(const uint8_t address, const uint8_t* data, const size_t size) {
  VERIFY(address + size <= size_, Error::kInvalidArg);
  for (size_t i = 0; i < size; ++i) {
    regs_[address + i] = data[i];
  }
  return Error::kOk;
}

inline Error RegMmio::Read(const uint8_t address, const size_t size, uint8_t* data) {
  VERIFY(address + size <= size_, Error::kInvalidArg);
  for (size_t i = 0; i < size; ++i) {
    data[i] = regs_[address + i];
  }
  return Error::kOk;
}

}  // namespace hortor::regmap
