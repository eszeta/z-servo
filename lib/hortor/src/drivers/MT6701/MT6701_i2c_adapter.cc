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

namespace hortor::drivers::MT6701 {

Error MT6701I2cAdapter::LinkAccessor(MT6701Accessor& accessor) {
  RegisterI2CAdapter::LinkAccessor(accessor);
  accessor.SetReadRaw([this](uint16_t& angle_raw,
                             Status& field_status,
                             bool& button_pushed,
                             bool& track_loss) {
    return ReadRaw(angle_raw, field_status, button_pushed, track_loss);
  });
  return Error::kOk;
}

Error MT6701I2cAdapter::ReadRaw(uint16_t& angle_raw,
                                Status& field_status,
                                bool& button_pushed,
                                bool& track_loss) {
  uint8_t angle6, angle0;
  CHECK(ReadMultiple(MT6701Regs::kANGLE_6.address, 1, &angle6));
  CHECK(ReadMultiple(MT6701Regs::kANGLE_0.address, 1, &angle0));

  angle_raw = reg::GetCombinedValue(
      MT6701Regs::kANGLE_6, MT6701Regs::kANGLE_0, angle6, angle0);

  // I2C模式下不支持这些状态读取
  (void)field_status;
  (void)button_pushed;
  (void)track_loss;

  return Error::kOk;
}

}  // namespace hortor::drivers::MT6701