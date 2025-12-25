// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include "regmap_spi_bus.h"

#include "regmap.h"
#include "servo/types.h"

namespace hortor::regmap {

Error RegMapSpiBus::Init(SPIClass *spi,
                         const int cs_pin,
                         const SPISettings &spi_settings) {
  if (!spi_) {
    return Error::kInvalidParameter;
  }
  spi_ = spi;
  cs_pin_ = cs_pin;
  spi_settings_ = spi_settings;
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);
  return Error::kOk;
}

Error RegMapSpiBus::WriteBytesImpl(const uint8_t address,
                                   const uint8_t *data,
                                   const size_t size) {
  if (!spi_ || !data) {
    return Error::kInvalidParameter;
  }
  spi_->beginTransaction(spi_settings_);
  if (cs_pin_ >= 0) digitalWrite(cs_pin_, LOW);
  spi_->transfer(address);
  spi_->transfer(const_cast<uint8_t *>(data), nullptr, size);
  if (cs_pin_ >= 0) digitalWrite(cs_pin_, HIGH);
  spi_->endTransaction();
  return Error::kOk;
}

Error RegMapSpiBus::ReadBytesImpl(const uint8_t address,
                                  const size_t size,
                                  uint8_t *data) {
  if (!spi_) {
    return Error::kInvalidParameter;
  }
  spi_->beginTransaction(spi_settings_);
  if (cs_pin_ >= 0) digitalWrite(cs_pin_, LOW);
  spi_->transfer(0x80 | address);
  spi_->transfer(data, size);
  if (cs_pin_ >= 0) digitalWrite(cs_pin_, HIGH);
  spi_->endTransaction();
  return Error::kOk;
}

}  // namespace hortor::regmap
