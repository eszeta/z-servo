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
#include "lowpass_filter.h"

#include <Arduino.h>

namespace hortor_servo {
LowPassFilter::LowPassFilter() {
  time_constant_ = 0.0f;
  y_prev_ = 0.0f;
}

float LowPassFilter::Compute(const float x, const float dt) {
  const float alpha = time_constant_ / (time_constant_ + dt);
  const float one_minus_alpha = 1.0f - alpha;
  const float y = alpha * y_prev_ + one_minus_alpha * x;
  y_prev_ = y;
  return y;
}
}  // namespace hortor_servo
