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

/**
 * @brief 将一个值从一个分辨率映射到另一个分辨率
 * @param value 需要映射的值
 * @param from 源分辨率
 * @param to 目标分辨率
 * @return 映射后的值
 */
template <typename T>
constexpr T mapResolution(T value, uint8_t from, uint8_t to) {
  const float max_from = static_cast<float>((1ULL << from) - 1);
  const float max_to = static_cast<float>((1ULL << to) - 1);
  const float scale = max_to / max_from;
  return static_cast<T>(value * scale);
}

/**
 * @brief 模运算, per https://stackoverflow.com/a/19288271
 * @param dividend 被除数
 * @param divisor 除数
 * @return 模运算结果
 */
constexpr int mod(const int dividend, const int divisor) {
  int r = dividend % divisor;
  if (r < 0) r += divisor;
  return r;
}

/**
 * @brief 模运算, per https://stackoverflow.com/a/19288271
 * @param x 被除数
 * @param y 除数
 * @return 模运算结果
 */
constexpr float fmodf_pos(float x, float y) {
  float r = fmod(x, y);
  if (r < 0) r += y;
  return r;
}

/**
 * @brief 包裹函数（将x包裹到[-y/2, +y/2]范围）
 * @param x 需要限制的值
 * @param y 范围
 * @return 限制后的值
 */
constexpr float wrap_pm(float x, float y) {
  float intval = round(x / y);
  return x - intval * y;
}

}  // namespace hortor::math