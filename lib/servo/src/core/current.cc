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

#include "current.h"

#include <Arduino.h>

namespace hortor_servo {

float Current::ReadAverageCurrents(const int n) {
  float current = GetCurrent();
  for (int i = 0; i < n; ++i) {
    const float new_current = GetCurrent();
    current = current * 0.6f + 0.4f * new_current;
    delay(3);
  }
  return current;
}

}  // namespace hortor_servo
