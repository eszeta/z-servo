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

#include "../../core/register_field.h"

namespace hortor_servo {

Error RegisterAccessor::WriteRegField(const Field& reg, uint8_t value) {
  uint8_t data;
  CHECK_ERROR(Read(reg.address, &data));
  RegisterField::SetValue(reg, value, data);
  CHECK_ERROR(Write(reg.address, data));
  return Error::kOk;
}

Error RegisterAccessor::WriteRegField(const Field& high, const Field& low, const uint16_t value) {
  uint8_t high_value, low_value;
  CHECK_ERROR(Read(high.address, &high_value));
  CHECK_ERROR(Read(low.address, &low_value));
  RegisterField::SetCombinedValue(high, low, value, high_value, low_value);
  CHECK_ERROR(Write(high.address, high_value));
  CHECK_ERROR(Write(low.address, low_value));
  return Error::kOk;
}

Error RegisterAccessor::ReadRegField(const Field& reg, uint8_t* value) {
  uint8_t data;
  CHECK_ERROR(Read(reg.address, &data));
  *value = RegisterField::GetValue(reg, data);
  return Error::kOk;
}

Error RegisterAccessor::ReadRegField(const Field& reg, bool* value) {
  uint8_t data;
  CHECK_ERROR(Read(reg.address, &data));
  *value = RegisterField::GetValue(reg, data) != 0;
  return Error::kOk;
}

Error RegisterAccessor::ReadRegField(const Field& high, const Field& low, uint16_t* value) {
  uint8_t high_value, low_value;
  CHECK_ERROR(Read(high.address, &high_value));
  CHECK_ERROR(Read(low.address, &low_value));
  *value = RegisterField::GetCombinedValue(high, low, high_value, low_value);
  return Error::kOk;
}
}  // namespace hortor_servo
