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

struct Resolution {
 public:
  explicit Resolution(uint8_t resolution_bits) : kBits(resolution_bits) {}
  /** @brief 目标分辨率（位数），决定了传感器的精度和量程 */
  const uint8_t kBits;
  /** @brief Counts Per Revolution */
  const uint16_t kEncoderCpr = (1 << kBits);
  /** @brief 角度到原始值的转换系数，用于将角度转换为原始值 */
  const float kAngleToRaw = kEncoderCpr / 360.0f;
  /** @brief 弧度到原始值的转换系数，用于将弧度转换为原始值 */
  const float kRadianToRaw = kEncoderCpr / TWO_PI;
  /** @brief 原始值到角度的转换系数，用于将原始值转换为角度 */
  const float kRawToAngle = 360.0f / kEncoderCpr;
  /** @brief 原始值到弧度的转换系数，用于将原始值转换为弧度 */
  const float kRawToRadian = TWO_PI / kEncoderCpr;
};

}  // namespace hortor_servo