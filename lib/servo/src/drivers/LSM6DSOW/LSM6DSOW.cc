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

#include "LSM6DSOW.h"

#include "LSM6DSOW_types.h"

namespace hortor_servo {
namespace LSM6DSOW {

Error LSM6DSOW::InitI2C(TwoWire* wire) {
  CHECK(i2c_transport_.Init(wire, kI2CAddress));
  CHECK(i2c_transport_.LinkAccessor(accessor_));
  CHECK(accessor_.Init());
  return Error::kOk;
}
}  // namespace LSM6DSOW
}  // namespace hortor_servo
