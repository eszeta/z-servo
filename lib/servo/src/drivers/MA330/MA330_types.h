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

#pragma once

#include "../../core/register_field.h"

namespace hortor_servo {
namespace MA330 {

static constexpr uint8_t kResolution = 14;
static SPISettings MA330SPISettings(1000000, MSBFIRST, SPI_MODE3);

enum class FieldStrength : uint8_t {
  kNormal = 0x00,
  kLow = 0x01,
  kHigh = 0x02,
  kErr = 0x03  // impossible state
};

namespace MA330Regs {
static constexpr RegisterField kZ_L(0x00, 0, 8);
static constexpr RegisterField kZ_H(0x01, 0, 8);
static constexpr RegisterField kBCT(0x02, 0, 8);
static constexpr RegisterField kETX(0x03, 0, 1);
static constexpr RegisterField kETY(0x03, 1, 1);
static constexpr RegisterField kILIP(0x04, 2, 4);
static constexpr RegisterField kPPT_L(0x04, 6, 2);
static constexpr RegisterField kPPT_H(0x05, 0, 8);
static constexpr RegisterField kMGHT(0x06, 2, 3);
static constexpr RegisterField kMGLT(0x06, 5, 3);
static constexpr RegisterField kNPP(0x07, 5, 3);
static constexpr RegisterField kRD(0x09, 7, 1);
static constexpr RegisterField kFW(0x0E, 0, 8);
static constexpr RegisterField kHYS(0x10, 0, 8);
static constexpr RegisterField kMGL_MGH(0x1B, 6, 2);
}  // namespace MA330Regs
}  // namespace MA330
}  // namespace hortor_servo
