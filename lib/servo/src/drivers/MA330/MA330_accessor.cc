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
#include "MA330_accessor.h"

#include "MA330_types.h"
namespace hortor_servo {
namespace MA330 {

uint16_t MA330Accessor::ReadRaw() {
  uint16_t angle;
  read_raw_(&angle);
  return angle;
};

Error MA330Accessor::GetZero(uint16_t* zero) {
  CHECK_ERROR(ReadRegField(MA330Regs::kZ_H, MA330Regs::kZ_L, zero));
  return Error::kOk;
};

Error MA330Accessor::GetBiasCurrentTrimming(uint8_t* bias_current_trimming) {
  CHECK_ERROR(ReadRegField(MA330Regs::kBCT, bias_current_trimming));
  return Error::kOk;
};

Error MA330Accessor::IsEnableTrimmingX(bool* enable) {
  CHECK_ERROR(ReadRegField(MA330Regs::kETX, enable));
  return Error::kOk;
};

Error MA330Accessor::IsEnableTrimmingY(bool* enable) {
  CHECK_ERROR(ReadRegField(MA330Regs::kETY, enable));
  return Error::kOk;
};

Error MA330Accessor::GetPulsesPerTurn(uint16_t* pulses_per_turn) {
  uint16_t result;
  CHECK_ERROR(ReadRegField(MA330Regs::kPPT_H, MA330Regs::kPPT_L, &result));
  *pulses_per_turn = result + 1;
  return Error::kOk;
};

Error MA330Accessor::GetIndexLength(uint8_t* index_length) {
  CHECK_ERROR(ReadRegField(MA330Regs::kILIP, index_length));
  return Error::kOk;
};

Error MA330Accessor::GetNumberPolePairs(uint8_t* number_pole_pairs) {
  CHECK_ERROR(ReadRegField(MA330Regs::kNPP, number_pole_pairs));
  return Error::kOk;
};

Error MA330Accessor::GetRotationDirection(uint8_t* rotation_direction) {
  CHECK_ERROR(ReadRegField(MA330Regs::kRD, rotation_direction));
  return Error::kOk;
};

Error MA330Accessor::GetFieldStrengthHighThreshold(uint8_t* high_threshold) {
  CHECK_ERROR(ReadRegField(MA330Regs::kMGHT, high_threshold));
  return Error::kOk;
};

Error MA330Accessor::GetFieldStrengthLowThreshold(uint8_t* low_threshold) {
  CHECK_ERROR(ReadRegField(MA330Regs::kMGLT, low_threshold));
  return Error::kOk;
};

Error MA330Accessor::GetFilterWidth(uint8_t* filter_width) {
  CHECK_ERROR(ReadRegField(MA330Regs::kFW, filter_width));
  return Error::kOk;
};

Error MA330Accessor::GetHysteresis(uint8_t* hysteresis) {
  CHECK_ERROR(ReadRegField(MA330Regs::kHYS, hysteresis));
  return Error::kOk;
};

Error MA330Accessor::GetFieldStrength(FieldStrength* field_strength) {
  uint8_t result;
  CHECK_ERROR(ReadRegField(MA330Regs::kMGL_MGH, &result));
  *field_strength = static_cast<FieldStrength>(result);
  return Error::kOk;
};

Error MA330Accessor::SetZero(uint16_t value) {
  CHECK_ERROR(WriteRegField(MA330Regs::kZ_H, MA330Regs::kZ_L, value));
  return Error::kOk;
};

Error MA330Accessor::SetBiasCurrentTrimming(uint8_t value) {
  CHECK_ERROR(WriteRegField(MA330Regs::kBCT, value));
  return Error::kOk;
};

Error MA330Accessor::SetTrimmingEnabled(bool Xenabled, bool Yenabled) {
  CHECK_ERROR(WriteRegField(MA330Regs::kETX, Xenabled));
  CHECK_ERROR(WriteRegField(MA330Regs::kETY, Yenabled));
  return Error::kOk;
};

Error MA330Accessor::SetPulsesPerTurn(uint16_t value) {
  uint16_t pptVal = value - 1;
  CHECK_ERROR(WriteRegField(MA330Regs::kPPT_H, MA330Regs::kPPT_L, pptVal));
  return Error::kOk;
};

Error MA330Accessor::SetIndexLength(uint8_t value) {
  CHECK_ERROR(WriteRegField(MA330Regs::kILIP, value));
  return Error::kOk;
};

Error MA330Accessor::SetNumberPolePairs(uint8_t value) {
  CHECK_ERROR(WriteRegField(MA330Regs::kNPP, value));
  return Error::kOk;
};

Error MA330Accessor::SetRotationDirection(uint8_t value) {
  CHECK_ERROR(WriteRegField(MA330Regs::kRD, value));
  return Error::kOk;
};

Error MA330Accessor::SetFilterWidth(uint8_t value) {
  CHECK_ERROR(WriteRegField(MA330Regs::kFW, value));
  return Error::kOk;
};

Error MA330Accessor::SetHysteresis(uint8_t value) {
  CHECK_ERROR(WriteRegField(MA330Regs::kHYS, value));
  return Error::kOk;
};

Error MA330Accessor::SetFieldStrengthThresholds(uint8_t high_threshold, uint8_t low_threshold) {
  CHECK_ERROR(WriteRegField(MA330Regs::kMGLT, low_threshold));
  CHECK_ERROR(WriteRegField(MA330Regs::kMGHT, high_threshold));
  return Error::kOk;
};

}  // namespace MA330
}  // namespace hortor_servo
