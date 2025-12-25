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
  Error ReadZero(uint16_t& zero);
  Error ReadBiasCurrentTrimming(uint8_t& bias_current_trimming);
  Error IsEnableTrimmingX(bool& enable);
  Error IsEnableTrimmingY(bool& enable);
  Error ReadPulsesPerTurn(uint16_t& pulses_per_turn);
  Error ReadIndexLength(uint8_t& index_length);
  Error ReadNumberPolePairs(uint8_t& number_pole_pairs);
  Error ReadRotationDirection(uint8_t& direction);
  Error ReadFilterWidth(uint8_t& filter_width);
  Error ReadHysteresis(uint8_t& hysteresis);
  Error ReadFieldStrengthHighThreshold(uint8_t& high_threshold);
  Error ReadFieldStrengthLowThreshold(uint8_t& low_threshold);
  Error ReadFieldStrength(FieldStrength& field_strength);

  Error WriteZero(uint16_t zero);
  Error WriteBiasCurrentTrimming(uint8_t bias_current_trimming);
  Error WriteTrimmingEnabled(bool Xenabled, bool Yenabled);
  Error WritePulsesPerTurn(uint16_t pulses_per_turn);
  Error WriteIndexLength(uint8_t index_length);
  Error WriteNumberPolePairs(uint8_t number_pole_pairs);
  Error WriteRotationDirection(uint8_t direction);
  Error WriteFilterWidth(uint8_t filter_width);
  Error WriteHysteresis(uint8_t hysteresis);
  Error WriteFieldStrengthThresholds(uint8_t high_threshold,
                                     uint8_t low_threshold);
};

}  // namespace hortor::drivers::MA330
