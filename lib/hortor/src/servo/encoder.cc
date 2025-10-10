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

#include "encoder.h"

#include "math/math.h"

namespace hortor::servo {

Error Encoder::Init() {
  // 读取初始原始值，等待传感器稳定后再次读取
  GetRaw(rew_pos_);
  delay(10);
  GetRaw(rew_pos_);

  // 初始化所有位置和速度状态变量
  pos_counts_ = rew_pos_;

  return Error::kOk;
}

Error Encoder::Process(float dt) {
  // 读取新的原始计数值
  uint16_t raw_new;
  CHECK(GetRaw(raw_new));

  // 计算位置增量（新位置 - 旧位置）
  int32_t delta_enc = raw_new - rew_pos_;
  delta_enc = math::mod(delta_enc, kResolution.kEncoderCpr);

  // 跨零点处理：选择最短路径
  if (delta_enc > kResolution.kEncoderCpr / 2) {
    delta_enc -= kResolution.kEncoderCpr;
  }

  // 更新线性累加位置（可跨越多圈）
  pos_counts_ += delta_enc;

  // 更新原始值记录
  rew_pos_ = raw_new;
  return Error::kOk;
}

}  // namespace hortor::servo