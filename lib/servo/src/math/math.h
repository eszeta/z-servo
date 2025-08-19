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

// 浮点数比较阈值，用于判断值是否接近0
static constexpr float kFloatThreshold = 0.001f;

/// @brief 将一个值从一个分辨率映射到另一个分辨率
/// @param value 需要映射的值
/// @param from 源分辨率
/// @param to 目标分辨率
/// @return 映射后的值
template <typename T>
static constexpr T mapResolution(T value, uint8_t from, uint8_t to) {
  const float max_from = static_cast<float>((1ULL << from) - 1);
  const float max_to = static_cast<float>((1ULL << to) - 1);
  const float scale = max_to / max_from;
  return static_cast<T>(value * scale);
}
}  // namespace hortor_servo