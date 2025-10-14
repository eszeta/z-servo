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
namespace hortor::math {

template <uint8_t Bits>
struct Resolution {
 public:
  constexpr explicit Resolution() {}
  /** @brief 目标分辨率（位数），决定了传感器的精度和量程 */
  static constexpr uint8_t kBits = Bits;
  /** @brief 最大值 */
  static constexpr uint32_t kMax = (1 << kBits) - 1;
  /** @brief Counts Per Revolution */
  static constexpr uint16_t kEncoderCpr = (1 << kBits);
  /** @brief 角度到计数值的转换系数，用于将角度转换为计数值 */
  static constexpr float kAngleToRaw = kEncoderCpr / 360.0f;
  /** @brief 弧度到计数值的转换系数，用于将弧度转换为计数值 */
  static constexpr float kRadianToRaw = kEncoderCpr / TWO_PI;
  /** @brief 计数值到角度的转换系数，用于将计数值转换为角度 */
  static constexpr float kRawToAngle = 360.0f / kEncoderCpr;
  /** @brief 计数值到弧度的转换系数，用于将计数值转换为弧度 */
  static constexpr float kRawToRadian = TWO_PI / kEncoderCpr;
};

}  // namespace hortor::math