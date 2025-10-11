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

#include "regmap.h"

#include "types.h"

namespace hortor::drivers::MA330 {

using Regs = MA330Regs;

Error RegMap::GetZero(uint16_t& zero) {
  CHECK(ReadRegField(Regs::kZ_H, Regs::kZ_L, zero));
  return Error::kOk;
};

Error RegMap::GetBiasCurrentTrimming(uint8_t& bias_current_trimming) {
  CHECK(ReadRegField(Regs::kBCT, bias_current_trimming));
  return Error::kOk;
};

Error RegMap::IsEnableTrimmingX(bool& enable) {
  CHECK(ReadRegField(Regs::kETX, enable));
  return Error::kOk;
};

Error RegMap::IsEnableTrimmingY(bool& enable) {
  CHECK(ReadRegField(Regs::kETY, enable));
  return Error::kOk;
};

Error RegMap::GetPulsesPerTurn(uint16_t& pulses_per_turn) {
  uint16_t result;
  CHECK(ReadRegField(Regs::kPPT_H, Regs::kPPT_L, result));
  pulses_per_turn = result + 1;
  return Error::kOk;
};

Error RegMap::GetIndexLength(uint8_t& index_length) {
  CHECK(ReadRegField(Regs::kILIP, index_length));
  return Error::kOk;
};

Error RegMap::GetNumberPolePairs(uint8_t& number_pole_pairs) {
  CHECK(ReadRegField(Regs::kNPP, number_pole_pairs));
  return Error::kOk;
};

Error RegMap::GetRotationDirection(uint8_t& rotation_direction) {
  CHECK(ReadRegField(Regs::kRD, rotation_direction));
  return Error::kOk;
};

Error RegMap::GetFieldStrengthHighThreshold(uint8_t& high_threshold) {
  CHECK(ReadRegField(Regs::kMGHT, high_threshold));
  return Error::kOk;
};

Error RegMap::GetFieldStrengthLowThreshold(uint8_t& low_threshold) {
  CHECK(ReadRegField(Regs::kMGLT, low_threshold));
  return Error::kOk;
};

Error RegMap::GetFilterWidth(uint8_t& filter_width) {
  CHECK(ReadRegField(Regs::kFW, filter_width));
  return Error::kOk;
};

Error RegMap::GetHysteresis(uint8_t& hysteresis) {
  CHECK(ReadRegField(Regs::kHYS, hysteresis));
  return Error::kOk;
};

Error RegMap::GetFieldStrength(FieldStrength& field_strength) {
  uint8_t result;
  CHECK(ReadRegField(Regs::kMGL_MGH, result));
  field_strength = static_cast<FieldStrength>(result);
  return Error::kOk;
};

Error RegMap::SetZero(uint16_t value) {
  CHECK(WriteRegField(Regs::kZ_H, Regs::kZ_L, value));
  return Error::kOk;
};

Error RegMap::SetBiasCurrentTrimming(uint8_t value) {
  CHECK(WriteRegField(Regs::kBCT, value));
  return Error::kOk;
};

Error RegMap::SetTrimmingEnabled(bool Xenabled, bool Yenabled) {
  CHECK(WriteRegField(Regs::kETX, Xenabled));
  CHECK(WriteRegField(Regs::kETY, Yenabled));
  return Error::kOk;
};

Error RegMap::SetPulsesPerTurn(uint16_t value) {
  uint16_t pptVal = value - 1;
  CHECK(WriteRegField(Regs::kPPT_H, Regs::kPPT_L, pptVal));
  return Error::kOk;
};

Error RegMap::SetIndexLength(uint8_t value) {
  CHECK(WriteRegField(Regs::kILIP, value));
  return Error::kOk;
};

Error RegMap::SetNumberPolePairs(uint8_t value) {
  CHECK(WriteRegField(Regs::kNPP, value));
  return Error::kOk;
};

Error RegMap::SetRotationDirection(uint8_t value) {
  CHECK(WriteRegField(Regs::kRD, value));
  return Error::kOk;
};

Error RegMap::SetFilterWidth(uint8_t value) {
  CHECK(WriteRegField(Regs::kFW, value));
  return Error::kOk;
};

Error RegMap::SetHysteresis(uint8_t value) {
  CHECK(WriteRegField(Regs::kHYS, value));
  return Error::kOk;
};

Error RegMap::SetFieldStrengthThresholds(uint8_t high_threshold,
                                         uint8_t low_threshold) {
  CHECK(WriteRegField(Regs::kMGLT, low_threshold));
  CHECK(WriteRegField(Regs::kMGHT, high_threshold));
  return Error::kOk;
};

}  // namespace hortor::drivers::MA330
