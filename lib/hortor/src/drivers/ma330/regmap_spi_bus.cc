// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include "regmap_spi_bus.h"

#include "regmap.h"

namespace hortor::drivers::MA330 {

Error RegMapSpiBus::ReadRaw(uint16_t& angle_raw) {
  angle_raw = transfer16(0x0000);
  return Error::kOk;
}

Error RegMapSpiBus::Write(const uint8_t address, const uint8_t data) {
  uint16_t cmd = 0x8000 | ((address & 0x1F) << 8) | data;
  transfer16(cmd);
  delay(20);  // 20ms delay required
  transfer16(0x0000);
  return Error::kOk;
}

Error RegMapSpiBus::Read(const uint8_t address, uint8_t* data) {
  uint16_t cmd = 0x4000 | ((address & 0x001F) << 8);
  uint16_t value = transfer16(cmd);
  delayMicroseconds(1);
  value = transfer16(0x0000);
  *data = value >> 8;
  return Error::kOk;
};

uint16_t RegMapSpiBus::transfer16(uint16_t outValue) {
  spi_->beginTransaction(spi_settings_);
  if (cs_pin_ >= 0) digitalWrite(cs_pin_, LOW);
  uint16_t value = spi_->transfer16(outValue);
  if (cs_pin_ >= 0) digitalWrite(cs_pin_, HIGH);
  spi_->endTransaction();
  return value;
}
}  // namespace hortor::drivers::MA330
