// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "regmap/reg_field.h"

namespace hortor::drivers::MA330 {

enum class FieldStrength : uint8_t {
  kNormal = 0x00,
  kLow = 0x01,
  kHigh = 0x02,
  kErr = 0x03  // impossible state
};

namespace MA330Regs {
using RegFieldU08 = regmap::RegFieldU08;
static constexpr RegFieldU08 kZ_L{0x00, 0, 8};
static constexpr RegFieldU08 kZ_H{0x01, 0, 8};
static constexpr RegFieldU08 kBCT{0x02, 0, 8};
static constexpr RegFieldU08 kETX{0x03, 0, 1};
static constexpr RegFieldU08 kETY{0x03, 1, 1};
static constexpr RegFieldU08 kILIP{0x04, 2, 4};
static constexpr RegFieldU08 kPPT_L{0x04, 6, 2};
static constexpr RegFieldU08 kPPT_H{0x05, 0, 8};
static constexpr RegFieldU08 kMGHT{0x06, 2, 3};
static constexpr RegFieldU08 kMGLT{0x06, 5, 3};
static constexpr RegFieldU08 kNPP{0x07, 5, 3};
static constexpr RegFieldU08 kRD{0x09, 7, 1};
static constexpr RegFieldU08 kFW{0x0E, 0, 8};
static constexpr RegFieldU08 kHYS{0x10, 0, 8};
static constexpr RegFieldU08 kMGL_MGH{0x1B, 6, 2};
};  // namespace MA330Regs

}  // namespace hortor::drivers::MA330
