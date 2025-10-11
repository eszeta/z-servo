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

#include "regmap_i2c_bus.h"

#include <Wire.h>

#include "regmap.h"
#include "servo/types.h"

namespace hortor::regmap {

Error RegMapI2CBus::Init(TwoWire* wire, const int address) {
  if (!wire) {
    return Error::kInvalidParameter;
  }
  wire_ = wire;
  address_ = address;
  return Error::kOk;
}

Error RegMapI2CBus::LinkAccessor(RegMap& accessor) {
  accessor.SetWriteMultiple(
      [this](const uint8_t address, const uint8_t* data, const size_t size) {
        return WriteMultiple(address, data, size);
      });
  accessor.SetReadMultiple(
      [this](const uint8_t address, const size_t size, uint8_t* data) {
        return ReadMultiple(address, size, data);
      });
  return Error::kOk;
}

Error RegMapI2CBus::WriteMultiple(const uint8_t address,
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

Error RegMapI2CBus::ReadMultiple(const uint8_t address,
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

