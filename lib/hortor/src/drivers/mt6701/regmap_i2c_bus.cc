// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include "regmap_i2c_bus.h"

namespace hortor::drivers::MT6701 {

Error RegMapI2CBus::ReadRaw(uint16_t& angle_raw,
                            Status& field_status,
                            bool& button_pushed,
                            bool& track_loss) {
  uint8_t angle6, angle0;
  CHECK(ReadBytes(MT6701Regs::kANGLE_6.address, 1, &angle6));
  CHECK(ReadBytes(MT6701Regs::kANGLE_0.address, 1, &angle0));

  angle_raw = regmap::GetCombinedValue(
      MT6701Regs::kANGLE_6, MT6701Regs::kANGLE_0, angle6, angle0);

  // I2C模式下不支持这些状态读取
  (void)field_status;
  (void)button_pushed;
  (void)track_loss;

  return Error::kOk;
}

}  // namespace hortor::drivers::MT6701