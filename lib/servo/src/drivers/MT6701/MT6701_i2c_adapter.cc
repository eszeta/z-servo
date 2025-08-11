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

#include "MT6701_i2c_adapter.h"

#include "MT6701_accessor.h"

namespace hortor_servo {
namespace MT6701 {

using Regs = MT6701Regs;

Error MT6701I2cAdapter::LinkAccessor(MT6701Accessor& accessor) {
  accessor.SetWrite([this](const uint8_t address, const uint8_t data) {
    return Write(address, data);
  });
  accessor.SetWriteMultiple(
      [this](const uint8_t address, const uint8_t* data, const size_t size) {
        return WriteMultiple(address, data, size);
      });
  accessor.SetRead([this](const uint8_t address, uint8_t* data) {
    return Read(address, data);
  });
  accessor.SetReadMultiple(
      [this](const uint8_t address, const size_t size, uint8_t* data) {
        return ReadMultiple(address, size, data);
      });
  accessor.SetReadRaw([this](uint16_t* angle_raw,
                             Status* field_status,
                             bool* button_pushed,
                             bool* track_loss) {
    return ReadRaw(angle_raw, field_status, button_pushed, track_loss);
  });
  return Error::kOk;
}

Error MT6701I2cAdapter::ReadRaw(uint16_t* angle_raw,
                                  Status* field_status,
                                  bool* button_pushed,
                                  bool* track_loss) {
  uint8_t angle6, angle0;
  CHECK(Read(Regs::kANGLE_6.address, &angle6));
  CHECK(Read(Regs::kANGLE_0.address, &angle0));

  if (angle_raw) {
    *angle_raw = Register::GetCombinedValue(
        Regs::kANGLE_6, Regs::kANGLE_0, angle6, angle0);
  }

  // I2C模式下不支持这些状态读取
  (void)field_status;
  (void)button_pushed;
  (void)track_loss;

  return Error::kOk;
}

}  // namespace MT6701
}  // namespace hortor_servo
