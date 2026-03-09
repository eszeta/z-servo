// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <SPI.h>

#include "base/encoder.h"
#include "base/types.h"
#include "regmap.h"

namespace hortor::drivers::MA330 {
constexpr uint8_t kResolutionBits = 14;

class Encoder;
using Base = servo::Encoder<Encoder, kResolutionBits>;

class Encoder : public Base {
 public:
  static constexpr uint8_t kResolutionBits =
      hortor::drivers::MA330::kResolutionBits;
  struct Config : public Base::Config {
    SPIClass* spi;
    uint8_t   cs_pin;
  };

  Error   Init(const Config& config);
  Error   ReadRawImpl(uint32_t& out_raw);
  Regmap* regmap();

 private:
  Regmap regmap_;
};

}  // namespace hortor::drivers::MA330

namespace hortor::drivers::MA330 {

inline Error Encoder::Init(const Config& config) {
  CHECK(regmap_.Init(config.spi, config.cs_pin,
                     SPISettings(1000000, MSBFIRST, SPI_MODE3)));
  CHECK(Base::Init(config));
  return Error::kOk;
}

inline Error Encoder::ReadRawImpl(uint32_t& out_raw) {
  uint16_t raw;
  CHECK(regmap_.ReadRaw(raw));
  out_raw = static_cast<uint32_t>(raw);
  return Error::kOk;
}

inline Regmap* Encoder::regmap() {
  return &regmap_;
}

}  // namespace hortor::drivers::MA330
