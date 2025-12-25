// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include "regmap.h"

#include "types.h"

namespace hortor::drivers::MA330 {

Error RegMap::ReadZero(uint16_t& zero) {
  CHECK(ReadRegField(MA330Regs::kZ_H, MA330Regs::kZ_L, zero));
  return Error::kOk;
};

Error RegMap::ReadBiasCurrentTrimming(uint8_t& bias_current_trimming) {
  CHECK(ReadRegField(MA330Regs::kBCT, bias_current_trimming));
  return Error::kOk;
};

Error RegMap::IsEnableTrimmingX(bool& enable) {
  CHECK(ReadRegField(MA330Regs::kETX, enable));
  return Error::kOk;
};

Error RegMap::IsEnableTrimmingY(bool& enable) {
  CHECK(ReadRegField(MA330Regs::kETY, enable));
  return Error::kOk;
};

Error RegMap::ReadPulsesPerTurn(uint16_t& pulses_per_turn) {
  uint16_t result;
  CHECK(ReadRegField(MA330Regs::kPPT_H, MA330Regs::kPPT_L, result));
  pulses_per_turn = result + 1;
  return Error::kOk;
};

Error RegMap::ReadIndexLength(uint8_t& index_length) {
  CHECK(ReadRegField(MA330Regs::kILIP, index_length));
  return Error::kOk;
};

Error RegMap::ReadNumberPolePairs(uint8_t& number_pole_pairs) {
  CHECK(ReadRegField(MA330Regs::kNPP, number_pole_pairs));
  return Error::kOk;
};

Error RegMap::ReadRotationDirection(uint8_t& rotation_direction) {
  CHECK(ReadRegField(MA330Regs::kRD, rotation_direction));
  return Error::kOk;
};

Error RegMap::ReadFieldStrengthHighThreshold(uint8_t& high_threshold) {
  CHECK(ReadRegField(MA330Regs::kMGHT, high_threshold));
  return Error::kOk;
};

Error RegMap::ReadFieldStrengthLowThreshold(uint8_t& low_threshold) {
  CHECK(ReadRegField(MA330Regs::kMGLT, low_threshold));
  return Error::kOk;
};

Error RegMap::ReadFilterWidth(uint8_t& filter_width) {
  CHECK(ReadRegField(MA330Regs::kFW, filter_width));
  return Error::kOk;
};

Error RegMap::ReadHysteresis(uint8_t& hysteresis) {
  CHECK(ReadRegField(MA330Regs::kHYS, hysteresis));
  return Error::kOk;
};

Error RegMap::ReadFieldStrength(FieldStrength& field_strength) {
  uint8_t result;
  CHECK(ReadRegField(MA330Regs::kMGL_MGH, result));
  field_strength = static_cast<FieldStrength>(result);
  return Error::kOk;
};

Error RegMap::WriteZero(uint16_t value) {
  CHECK(WriteRegField(MA330Regs::kZ_H, MA330Regs::kZ_L, value));
  return Error::kOk;
};

Error RegMap::WriteBiasCurrentTrimming(uint8_t value) {
  CHECK(WriteRegField(MA330Regs::kBCT, value));
  return Error::kOk;
};

Error RegMap::WriteTrimmingEnabled(bool Xenabled, bool Yenabled) {
  CHECK(WriteRegField(MA330Regs::kETX, Xenabled));
  CHECK(WriteRegField(MA330Regs::kETY, Yenabled));
  return Error::kOk;
};

Error RegMap::WritePulsesPerTurn(uint16_t value) {
  uint16_t pptVal = value - 1;
  CHECK(WriteRegField(MA330Regs::kPPT_H, MA330Regs::kPPT_L, pptVal));
  return Error::kOk;
};

Error RegMap::WriteIndexLength(uint8_t value) {
  CHECK(WriteRegField(MA330Regs::kILIP, value));
  return Error::kOk;
};

Error RegMap::WriteNumberPolePairs(uint8_t value) {
  CHECK(WriteRegField(MA330Regs::kNPP, value));
  return Error::kOk;
};

Error RegMap::WriteRotationDirection(uint8_t value) {
  CHECK(WriteRegField(MA330Regs::kRD, value));
  return Error::kOk;
};

Error RegMap::WriteFilterWidth(uint8_t value) {
  CHECK(WriteRegField(MA330Regs::kFW, value));
  return Error::kOk;
};

Error RegMap::WriteHysteresis(uint8_t value) {
  CHECK(WriteRegField(MA330Regs::kHYS, value));
  return Error::kOk;
};

Error RegMap::WriteFieldStrengthThresholds(uint8_t high_threshold,
                                           uint8_t low_threshold) {
  CHECK(WriteRegField(MA330Regs::kMGLT, low_threshold));
  CHECK(WriteRegField(MA330Regs::kMGHT, high_threshold));
  return Error::kOk;
};

}  // namespace hortor::drivers::MA330
