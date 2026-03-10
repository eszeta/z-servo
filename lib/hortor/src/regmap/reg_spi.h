// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "error.h"
#include "noncopyable.h"
#include "regmap.h"
#include "servo/types.h"

namespace hortor::regmap {

class RegSPI : public hortor::Noncopyable {
 public:
  SPIClass*   spi();
  int         cs_pin() const;
  SPISettings spi_settings() const;

  Error Init(SPIClass* spi, int cs_pin, const SPISettings& spi_settings);
  Error Write(const uint8_t address, const uint8_t* data, const size_t size);
  Error Read(const uint8_t address, const size_t size, uint8_t* data);

 protected:
  SPIClass*   spi_{};
  int         cs_pin_{};
  SPISettings spi_settings_;
};

}  // namespace hortor::regmap

namespace hortor::regmap {

inline SPIClass* RegSPI::spi() {
  return spi_;
}

inline int RegSPI::cs_pin() const {
  return cs_pin_;
}

inline SPISettings RegSPI::spi_settings() const {
  return spi_settings_;
}

inline Error RegSPI::Init(SPIClass* spi, int cs_pin, const SPISettings& spi_settings) {
  VERIFY(spi, Error::kInvalidArg);
  spi_          = spi;
  cs_pin_       = cs_pin;
  spi_settings_ = spi_settings;
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);
  return Error::kOk;
}

inline Error RegSPI::Write(const uint8_t address, const uint8_t* data, const size_t size) {
  VERIFY(spi_ && data, Error::kInvalidArg);
  spi_->beginTransaction(spi_settings_);
  if (cs_pin_ >= 0)
    digitalWrite(cs_pin_, LOW);
  spi_->transfer(address);
  spi_->transfer(const_cast<uint8_t*>(data), nullptr, size);
  if (cs_pin_ >= 0)
    digitalWrite(cs_pin_, HIGH);
  spi_->endTransaction();
  return Error::kOk;
}

inline Error RegSPI::Read(const uint8_t address, const size_t size, uint8_t* data) {
  VERIFY(spi_, Error::kInvalidArg);
  spi_->beginTransaction(spi_settings_);
  if (cs_pin_ >= 0)
    digitalWrite(cs_pin_, LOW);
  spi_->transfer(0x80 | address);
  spi_->transfer(data, size);
  if (cs_pin_ >= 0)
    digitalWrite(cs_pin_, HIGH);
  spi_->endTransaction();
  return Error::kOk;
}

}  // namespace hortor::regmap
