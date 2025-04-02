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

#include "register_accessor.h"

#include "../../core/register.h"

namespace hortor_servo {

Error RegisterAccessor::WriteRegField(const Register& reg, uint8_t value) {
  uint8_t data;
  CHECK(Read(reg.address, &data));
  Register::SetValue(reg, value, data);
  CHECK(Write(reg.address, data));
  return Error::kOk;
}

Error RegisterAccessor::WriteRegField(const Register& high,
                                      const Register& low,
                                      const uint16_t value) {
  uint8_t high_value, low_value;
  CHECK(Read(high.address, &high_value));
  CHECK(Read(low.address, &low_value));
  Register::SetCombinedValue(high, low, value, high_value, low_value);
  CHECK(Write(high.address, high_value));
  CHECK(Write(low.address, low_value));
  return Error::kOk;
}

Error RegisterAccessor::ReadRegField(const Register& reg, uint8_t* value) {
  uint8_t data;
  CHECK(Read(reg.address, &data));
  *value = Register::GetValue(reg, data);
  return Error::kOk;
}

Error RegisterAccessor::ReadRegField(const Register& reg, bool* value) {
  uint8_t data;
  CHECK(Read(reg.address, &data));
  *value = Register::GetValue(reg, data) != 0;
  return Error::kOk;
}

Error RegisterAccessor::ReadRegField(const Register& high,
                                     const Register& low,
                                     uint16_t* value) {
  uint8_t high_value, low_value;
  CHECK(Read(high.address, &high_value));
  CHECK(Read(low.address, &low_value));
  *value = Register::GetCombinedValue(high, low, high_value, low_value);
  return Error::kOk;
}
}  // namespace hortor_servo
