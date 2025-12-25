// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "regmap_spi_bus.h"
#include "types.h"

namespace hortor::drivers::MA330 {

/**
 * @brief MA330寄存器映射类（CRTP模式）
 *
 * 继承自 RegMapSpiBus，通过 CRTP 实现编译期静态多态。
 */
class RegMap : public RegMapSpiBus {
 public:
  Error GetZero(uint16_t& zero);
  Error GetBiasCurrentTrimming(uint8_t& bias_current_trimming);
  Error IsEnableTrimmingX(bool& enable);
  Error IsEnableTrimmingY(bool& enable);
  Error GetPulsesPerTurn(uint16_t& pulses_per_turn);
  Error GetIndexLength(uint8_t& index_length);
  Error GetNumberPolePairs(uint8_t& number_pole_pairs);
  Error GetRotationDirection(uint8_t& direction);
  Error GetFilterWidth(uint8_t& filter_width);
  Error GetHysteresis(uint8_t& hysteresis);
  Error GetFieldStrengthHighThreshold(uint8_t& high_threshold);
  Error GetFieldStrengthLowThreshold(uint8_t& low_threshold);
  Error GetFieldStrength(FieldStrength& field_strength);

  Error SetZero(uint16_t zero);
  Error SetBiasCurrentTrimming(uint8_t bias_current_trimming);
  Error SetTrimmingEnabled(bool Xenabled, bool Yenabled);
  Error SetPulsesPerTurn(uint16_t pulses_per_turn);
  Error SetIndexLength(uint8_t index_length);
  Error SetNumberPolePairs(uint8_t number_pole_pairs);
  Error SetRotationDirection(uint8_t direction);
  Error SetFilterWidth(uint8_t filter_width);
  Error SetHysteresis(uint8_t hysteresis);
  Error SetFieldStrengthThresholds(uint8_t high_threshold,
                                   uint8_t low_threshold);
};

}  // namespace hortor::drivers::MA330
