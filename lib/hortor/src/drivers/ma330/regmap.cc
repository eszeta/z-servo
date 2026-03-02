// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include "regmap.h"

#include "types.h"

namespace hortor::drivers::MA330 {

Error RegMap::ReadZero(uint16_t& zero) {
  using kZ_H = MA330Regs::kZ_H;
  using kZ_L = MA330Regs::kZ_L;
  CHECK(ReadField<uint16_t, kZ_H, kZ_L>(zero));
  return Error::kOk;
};

Error RegMap::ReadBiasCurrentTrimming(uint8_t& bias_current_trimming) {
  using kBCT = MA330Regs::kBCT;
  CHECK(ReadField<kBCT>(bias_current_trimming));
  return Error::kOk;
};

Error RegMap::IsEnableTrimmingX(bool& enable) {
  using kETX = MA330Regs::kETX;
  CHECK(ReadField<kETX>(enable));
  return Error::kOk;
};

Error RegMap::IsEnableTrimmingY(bool& enable) {
  using kETY = MA330Regs::kETY;
  CHECK(ReadField<kETY>(enable));
  return Error::kOk;
};

Error RegMap::ReadPulsesPerTurn(uint16_t& pulses_per_turn) {
  uint16_t result;
  using kPPT_H = MA330Regs::kPPT_H;
  using kPPT_L = MA330Regs::kPPT_L;
  CHECK(ReadField<uint16_t, kPPT_H, kPPT_L>(result));
  pulses_per_turn = result + 1;
  return Error::kOk;
};

Error RegMap::ReadIndexLength(uint8_t& index_length) {
  using kILIP = MA330Regs::kILIP;
  CHECK(ReadField<kILIP>(index_length));
  return Error::kOk;
};

Error RegMap::ReadNumberPolePairs(uint8_t& number_pole_pairs) {
  using kNPP = MA330Regs::kNPP;
  CHECK(ReadField<kNPP>(number_pole_pairs));
  return Error::kOk;
};

Error RegMap::ReadRotationDirection(uint8_t& rotation_direction) {
  using kRD = MA330Regs::kRD;
  CHECK(ReadField<kRD>(rotation_direction));
  return Error::kOk;
};

Error RegMap::ReadFieldStrengthHighThreshold(uint8_t& high_threshold) {
  using kMGHT = MA330Regs::kMGHT;
  CHECK(ReadField<kMGHT>(high_threshold));
  return Error::kOk;
};

Error RegMap::ReadFieldStrengthLowThreshold(uint8_t& low_threshold) {
  using kMGLT = MA330Regs::kMGLT;
  CHECK(ReadField<kMGLT>(low_threshold));
  return Error::kOk;
};

Error RegMap::ReadFilterWidth(uint8_t& filter_width) {
  using kFW = MA330Regs::kFW;
  CHECK(ReadField<kFW>(filter_width));
  return Error::kOk;
};

Error RegMap::ReadHysteresis(uint8_t& hysteresis) {
  using kHYS = MA330Regs::kHYS;
  CHECK(ReadField<kHYS>(hysteresis));
  return Error::kOk;
};

Error RegMap::ReadFieldStrength(FieldStrength& field_strength) {
  uint8_t result;
  using kMGL_MGH = MA330Regs::kMGL_MGH;
  CHECK(ReadField<kMGL_MGH>(result));
  field_strength = static_cast<FieldStrength>(result);
  return Error::kOk;
};

Error RegMap::WriteZero(uint16_t value) {
  using kZ_H = MA330Regs::kZ_H;
  using kZ_L = MA330Regs::kZ_L;
  CHECK(WriteField<uint16_t, kZ_H, kZ_L>(value));
  return Error::kOk;
};

Error RegMap::WriteBiasCurrentTrimming(uint8_t value) {
  using kBCT = MA330Regs::kBCT;
  CHECK(WriteField<kBCT>(value));
  return Error::kOk;
};

Error RegMap::WriteTrimmingEnabled(bool Xenabled, bool Yenabled) {
  using kETX = MA330Regs::kETX;
  using kETY = MA330Regs::kETY;
  CHECK(WriteField<kETX>(Xenabled));
  CHECK(WriteField<kETY>(Yenabled));
  return Error::kOk;
};

Error RegMap::WritePulsesPerTurn(uint16_t value) {
  uint16_t pptVal = value - 1;
  using kPPT_H = MA330Regs::kPPT_H;
  using kPPT_L = MA330Regs::kPPT_L;
  CHECK(WriteField<uint16_t, kPPT_H, kPPT_L>(pptVal));
  return Error::kOk;
};

Error RegMap::WriteIndexLength(uint8_t value) {
  using kILIP = MA330Regs::kILIP;
  CHECK(WriteField<kILIP>(value));
  return Error::kOk;
};

Error RegMap::WriteNumberPolePairs(uint8_t value) {
  using kNPP = MA330Regs::kNPP;
  CHECK(WriteField<kNPP>(value));
  return Error::kOk;
};

Error RegMap::WriteRotationDirection(uint8_t value) {
  using kRD = MA330Regs::kRD;
  CHECK(WriteField<kRD>(value));
  return Error::kOk;
};

Error RegMap::WriteFilterWidth(uint8_t value) {
  using kFW = MA330Regs::kFW;
  CHECK(WriteField<kFW>(value));
  return Error::kOk;
};

Error RegMap::WriteHysteresis(uint8_t value) {
  using kHYS = MA330Regs::kHYS;
  CHECK(WriteField<kHYS>(value));
  return Error::kOk;
};

Error RegMap::WriteFieldStrengthThresholds(uint8_t high_threshold,
                                           uint8_t low_threshold) {
  using kMGLT = MA330Regs::kMGLT;
  using kMGHT = MA330Regs::kMGHT;
  CHECK(WriteField<kMGLT>(low_threshold));
  CHECK(WriteField<kMGHT>(high_threshold));
  return Error::kOk;
};

}  // namespace hortor::drivers::MA330
