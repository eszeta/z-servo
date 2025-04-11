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

#include "MT6701.h"

#include "MT6701_types.h"

namespace hortor_servo {
namespace MT6701 {

Error MT6701::InitI2C(TwoWire *wire) {
  CHECK(i2c_transport_.Init(wire, kI2CAddress));
  CHECK(i2c_transport_.LinkAccessor(accessor_));
  CHECK(accessor_.Init());
  hortor_servo::Sensor::Init();
  return Error::kOk;
}

uint16_t MT6701::GetRaw() {
  uint16_t angle = 0;
  accessor_.ReadRaw(&angle, nullptr, nullptr, nullptr);
  return angle;
}
}  // namespace MT6701
}  // namespace hortor_servo
