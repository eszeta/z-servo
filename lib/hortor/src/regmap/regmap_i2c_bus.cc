// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include "regmap_i2c_bus.h"

#include <Wire.h>

#include "base/types.h"
#include "regmap.h"

namespace hortor::regmap {

Error RegMapI2CBus::Init(TwoWire* wire, const int address) {
  if (!wire) {
    return Error::kInvalidParameter;
  }
  wire_ = wire;
  address_ = address;
  return Error::kOk;
}

Error RegMapI2CBus::WriteImpl(const uint8_t address,
                                   const uint8_t* data,
                                   const size_t size) {
  if (!wire_ || !data) {
    return Error::kInvalidParameter;
  }
  wire_->beginTransmission(address_);
  wire_->write(address);
  wire_->write(data, size);
  if (wire_->endTransmission() != 0) {
    return Error::kIOErr;
  }
  return Error::kOk;
}

Error RegMapI2CBus::ReadImpl(const uint8_t address,
                                  const size_t size,
                                  uint8_t* data) {
  if (!wire_ || !data) {
    return Error::kInvalidParameter;
  }
  wire_->beginTransmission(address_);
  wire_->write(address);
  wire_->endTransmission(false);
  wire_->requestFrom(address_, size);
  for (size_t i = 0; i < size; i++) {
    if (wire_->available()) {
      data[i] = wire_->read();
    } else {
      return Error::kIOErr;
    }
  }
  return Error::kOk;
}

}  // namespace hortor::regmap
