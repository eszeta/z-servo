// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

namespace hortor::math {

/// @brief 微秒转换为秒
constexpr float kMicroToSec = 1e-6f;

/// @brief 毫秒转换为秒
constexpr float kMilliToSec = 1e-3f;

// 浮点数比较阈值，用于判断值是否接近0
constexpr float kFloatThreshold = 0.001f;

template <typename T>
constexpr T     mapResolution(T value, uint8_t from, uint8_t to);
constexpr int   mod(const int dividend, const int divisor);
constexpr float fmodf_pos(float x, float y);
constexpr float wrap_pm(float x, float y);

}  // namespace hortor::math

namespace hortor::math {

template <typename T>
constexpr T mapResolution(T value, uint8_t from, uint8_t to) {
  const float max_from = static_cast<float>((1ULL << from) - 1);
  const float max_to   = static_cast<float>((1ULL << to) - 1);
  const float scale    = max_to / max_from;
  return static_cast<T>(value * scale);
}

constexpr int mod(const int dividend, const int divisor) {
  int r = dividend % divisor;
  if (r < 0)
    r += divisor;
  return r;
}

constexpr float fmodf_pos(float x, float y) {
  float r = fmod(x, y);
  if (r < 0)
    r += y;
  return r;
}

constexpr float wrap_pm(float x, float y) {
  float intval = round(x / y);
  return x - intval * y;
}

}  // namespace hortor::math