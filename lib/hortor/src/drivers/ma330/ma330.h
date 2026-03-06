// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <SPI.h>

#include "base/encoder.h"
#include "base/types.h"
#include "regmap.h"

namespace hortor::drivers::MA330 {
constexpr uint8_t kResolutionBits = 14;

class MA330;
using MA330Base = servo::Encoder<MA330, kResolutionBits>;
class MA330 : public MA330Base {
 public:
  static constexpr uint8_t kResolutionBits =
      hortor::drivers::MA330::kResolutionBits;
  struct Config : public MA330Base::Config {
    SPIClass* spi;
    uint8_t   cs_pin;
  };

  Error   Init(const Config& config);
  Error   ReadRawImpl(uint32_t& out_raw);
  RegMap* regmap();

 private:
  RegMap regmap_;
};

}  // namespace hortor::drivers::MA330

namespace hortor::drivers::MA330 {

inline Error MA330::Init(const Config& config) {
  CHECK(regmap_.Init(config.spi, config.cs_pin,
                     SPISettings(1000000, MSBFIRST, SPI_MODE3)));
  CHECK(MA330Base::Init(config));
  return Error::kOk;
}

inline Error MA330::ReadRawImpl(uint32_t& out_raw) {
  uint16_t raw;
  CHECK(regmap_.ReadRaw(raw));
  out_raw = static_cast<uint32_t>(raw);
  return Error::kOk;
}

inline RegMap* MA330::regmap() {
  return &regmap_;
}

}  // namespace hortor::drivers::MA330
