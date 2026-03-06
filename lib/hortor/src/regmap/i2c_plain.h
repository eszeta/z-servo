// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Wire.h>

#include "base/types.h"
#include "hortor.h"
#include "regmap.h"

namespace hortor::regmap {

/**
 * @brief I2C通信实现
 *
 * 实现通过I2C协议与设备通信的功能。
 */
class I2CPlain : public hortor::Noncopyable {
 public:
  Error Init(TwoWire* wire, const int address);
  Error Write(const uint8_t address, const uint8_t* data, const size_t size);
  Error Read(const uint8_t address, const size_t size, uint8_t* data);

 protected:
  TwoWire* wire_{};
  int      address_{};
};

}  // namespace hortor::regmap

namespace hortor::regmap {

inline Error I2CPlain::Init(TwoWire* wire, const int address) {
  VERIFY(wire, Error::kInvalidArg);
  wire_    = wire;
  address_ = address;
  return Error::kOk;
}

inline Error I2CPlain::Write(const uint8_t address, const uint8_t* data,
                             const size_t size) {
  VERIFY(wire_ && data, Error::kInvalidArg);
  wire_->beginTransmission(address_);
  wire_->write(address);
  wire_->write(data, size);
  if (wire_->endTransmission() != 0) {
    return Error::kIO;
  }
  return Error::kOk;
}

inline Error I2CPlain::Read(const uint8_t address, const size_t size,
                            uint8_t* data) {
  VERIFY(wire_ && data, Error::kInvalidArg);
  wire_->beginTransmission(address_);
  wire_->write(address);
  wire_->endTransmission(false);
  wire_->requestFrom(address_, size);
  for (size_t i = 0; i < size; i++) {
    if (wire_->available()) {
      data[i] = wire_->read();
    } else {
      return Error::kIO;
    }
  }
  return Error::kOk;
}

}  // namespace hortor::regmap
