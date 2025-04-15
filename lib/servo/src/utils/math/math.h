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

namespace hortor_servo {
/// @brief 微秒转换为秒
static constexpr float kMicroToSec = 1e-6f;

/// @brief 毫秒转换为秒
static constexpr float kMilliToSec = 1e-3f;

/// @brief 将一个值从一个分辨率映射到另一个分辨率
/// @param value 需要映射的值
/// @param from 源分辨率
/// @param to 目标分辨率
/// @return 映射后的值
static constexpr uint32_t mapResolution(uint32_t value,
                                        uint32_t from,
                                        uint32_t to) {
  if (from != to) {
    if (from > to) {
      value = (value < (uint32_t)(1 << (from - to)))
                  ? 0
                  : ((value + 1) >> (from - to)) - 1;
    } else {
      if (value != 0) {
        value = ((value + 1) << (to - from)) - 1;
      }
    }
  }
  return value;
}
}  // namespace hortor_servo