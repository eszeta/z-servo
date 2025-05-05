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

#include "sensor.h"

#include "../utils/math/math.h"
namespace hortor_servo {
void Sensor::Init() {
  GetRaw();
  delay(10);  // 等待传感器稳定
  raw_ = GetRaw();
  raw_prev_ = raw_;
}

void Sensor::Process(float dt) {
  raw_ = GetRaw();

  const auto d_angle = static_cast<int16_t>(raw_ - raw_prev_);
  // 如果发生溢出，将其记录为一圈
  if (std::abs(d_angle) > 0.8f * kResolution.kEncoderCpr) {
    full_rotations_ += (d_angle > 0) ? -1 : 1;
  }
  raw_prev_ = raw_;

  accumulated_dt_ += dt;
  // 确保累积时间达到最小采样间隔
  if (accumulated_dt_ >= kMinElapsedTime) {
    // 计算角度变化
    const auto angle_diff =
        (full_rotations_ - full_rotations_prev_) * kResolution.kEncoderCpr +
        d_angle;

    // 计算速度（单位：计数/秒）
    velocity_ = static_cast<float>(angle_diff) / accumulated_dt_;

    // 更新上一次的值
    full_rotations_prev_ = full_rotations_;
    accumulated_dt_ = 0;  // 重置累计时间
  }
}
}  // namespace hortor_servo