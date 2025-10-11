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

#include "regmap_spi_bus.h"

namespace hortor::drivers::MT6701 {

Error RegMapSpiBus::ReadRaw(uint16_t& angle_raw,
                            Status& field_status,
                            bool& button_pushed,
                            bool& track_loss) {
  if (!spi_) {
    return Error::kInvalidParameter;
  }

  uint8_t data[3];
  digitalWrite(cs_pin_, LOW);
  spi_->beginTransaction(spi_settings_);

  data[0] = spi_->transfer(0xFF);
  data[1] = spi_->transfer(0xFF);
  data[2] = spi_->transfer(0xFF);

  spi_->endTransaction();
  digitalWrite(cs_pin_, HIGH);

  struct {
    uint16_t angle : 14;  // 14位角度数据，范围0-16383
    uint8_t status : 2;   // 2位磁场状态，指示磁场强度
    uint8_t button : 1;   // 1位按钮状态，指示按钮是否被按下
    uint8_t track : 1;    // 1位跟踪丢失状态，指示是否丢失跟踪
  } __packed* raw = reinterpret_cast<decltype(raw)>(data);

  angle_raw = raw->angle;
  field_status = static_cast<Status>(raw->status);
  button_pushed = raw->button;
  track_loss = raw->track;

  return Error::kOk;
}

}  // namespace hortor::drivers::MT6701