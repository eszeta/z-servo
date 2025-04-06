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

namespace hortor_servo {

Sensor::Sensor(const uint8_t resolution)
    : kResolution(resolution),
      kFullScale(1 << resolution),
      kOverflowTh(0.8f * kFullScale) {}

void Sensor::Init() {
  GetRaw();
  delay(10);  // 等待传感器稳定
  const uint16_t raw = GetRaw();
  vel_raw_prev_ = raw;
  raw_prev_ = raw;
  raw_val_ = raw;
}

void Sensor::Process(uint32_t dt) {
  raw_val_ = GetRaw();
  CalculateFullRotations();
  CalculateVelocity(dt);
}

void Sensor::CalculateFullRotations() {
  const uint16_t d_angle = raw_val_ - raw_prev_;
  // 如果发生溢出，将其记录为一圈
  if (abs(d_angle) > kOverflowTh) {
    full_rotations_ += (d_angle > 0) ? -1 : 1;
  }
  raw_prev_ = raw_val_;
}

void Sensor::CalculateVelocity(uint32_t dt) {
  accumulated_dt_ += dt;

  // 确保累积时间达到最小采样间隔
  if (accumulated_dt_ < kMinElapsedTime) return;

  // 计算角度变化
  const float angle_diff =
      static_cast<float>(full_rotations_ * kFullScale + raw_val_) -
      static_cast<float>(vel_full_rotations_ * kFullScale + vel_raw_prev_);

  // 计算速度（单位：计数/秒）
  const float time_seconds = accumulated_dt_ * 1e-6f;
  velocity_ = angle_diff / time_seconds;

  // 更新上一次的值
  vel_raw_prev_ = raw_val_;
  vel_full_rotations_ = full_rotations_;
  accumulated_dt_ = 0;  // 重置累计时间
}

float Sensor::GetVelocity() { return velocity_; }

uint16_t Sensor::GetMechanicalAngle() { return raw_val_; }

uint16_t Sensor::GetAngle() { return full_rotations_ * kFullScale + raw_val_; }

int32_t Sensor::GetFullRotations() { return full_rotations_; }
}  // namespace hortor_servo