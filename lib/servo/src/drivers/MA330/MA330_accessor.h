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

#include <Arduino.h>
#include <SPI.h>

#include <functional>

#include "../../utils/register/register_accessor.h"
#include "MA330_types.h"

namespace hortor_servo {
namespace MA330 {

class MA330Accessor : public RegisterAccessor {
 public:
  using ReadRawFunc = std::function<Error(uint16_t* angle_raw)>;
  Error Init() { return Error::kOk; }
  uint16_t ReadRaw();

  Error GetZero(uint16_t* zero);
  Error GetBiasCurrentTrimming(uint8_t* bias_current_trimming);
  Error IsEnableTrimmingX(bool* enable);
  Error IsEnableTrimmingY(bool* enable);
  Error GetPulsesPerTurn(uint16_t* pulses_per_turn);
  Error GetIndexLength(uint8_t* index_length);
  Error GetNumberPolePairs(uint8_t* number_pole_pairs);
  Error GetRotationDirection(uint8_t* direction);
  Error GetFilterWidth(uint8_t* filter_width);
  Error GetHysteresis(uint8_t* hysteresis);
  Error GetFieldStrengthHighThreshold(uint8_t* high_threshold);
  Error GetFieldStrengthLowThreshold(uint8_t* low_threshold);
  Error GetFieldStrength(FieldStrength* field_strength);

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
  /**
   * @brief 设置读取原始值的函数
   * @param read_raw 读取原始值的函数
   */
  void SetReadRaw(ReadRawFunc read_raw) { read_raw_ = read_raw; }

 private:
  ReadRawFunc read_raw_;
};

}  // namespace MA330
}  // namespace hortor_servo
