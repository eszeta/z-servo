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

#include "MA330_spi_adapter.h"

#include "MA330_accessor.h"

namespace hortor_servo {
namespace MA330 {

Error MA330SpiAdapter::LinkAccessor(MA330Accessor& accessor) {
  accessor.SetWrite([this](const uint8_t address, const uint8_t data) {
    return Write(address, data);
  });
  accessor.SetWriteMultiple(
      [this](const uint8_t address, const uint8_t* data, const size_t size) {
        return WriteMultiple(address, data, size);
      });
  accessor.SetRead([this](const uint8_t address, uint8_t* data) {
    return Read(address, data);
  });
  accessor.SetReadMultiple(
      [this](const uint8_t address, const size_t size, uint8_t* data) {
        return ReadMultiple(address, size, data);
      });
  accessor.SetReadRaw(
      [this](uint16_t* angle_raw) { return ReadRaw(angle_raw); });
  return Error::kOk;
}

Error MA330SpiAdapter::ReadRaw(uint16_t* angle_raw) {
  *angle_raw = transfer16(0x0000);
  return Error::kOk;
}

Error MA330SpiAdapter::Write(const uint8_t address, const uint8_t data) {
  uint16_t cmd = 0x8000 | ((address & 0x1F) << 8) | data;
  transfer16(cmd);
  delay(20);  // 20ms delay required
  transfer16(0x0000);
  return Error::kOk;
}

Error MA330SpiAdapter::Read(const uint8_t address, uint8_t* data) {
  uint16_t cmd = 0x4000 | ((address & 0x001F) << 8);
  uint16_t value = transfer16(cmd);
  delayMicroseconds(1);
  value = transfer16(0x0000);
  *data = value >> 8;
  return Error::kOk;
};

uint16_t MA330SpiAdapter::transfer16(uint16_t outValue) {
  spi_->beginTransaction(spi_settings_);
  if (cs_pin_ >= 0) digitalWrite(cs_pin_, LOW);
  uint16_t value = spi_->transfer16(outValue);
  if (cs_pin_ >= 0) digitalWrite(cs_pin_, HIGH);
  spi_->endTransaction();
  return value;
}
}  // namespace MA330
}  // namespace hortor_servo
