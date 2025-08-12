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
#include "inst_i2c_adapter.h"

#include <Arduino.h>
#include <Wire.h>

#include "core/types.h"
#include "inst/inst_types.h"

namespace hortor_servo {

Error InstI2cAdapter::Process(float dt) { return Error::kOk; }

Error InstI2cAdapter::Response(const uint8_t reply_idx, const uint8_t *data) {
  const size_t size = inst_utils::GetBufferSize(data);
  memcpy(tx_buffer_, data, size);
  return Error::kOk;
}

void InstI2cAdapter::OnReceive(int howMany) {
  size_t pos = 0;
  while (wire_->available()) {
    rx_buffer_[pos++] = wire_->read();
  }
  if (inst_utils::CheckChecksum(rx_buffer_)) {
    execute_(rx_buffer_);
  }
}

void InstI2cAdapter::OnRequest() {
  wire_->write(tx_buffer_, inst_utils::GetBufferSize(tx_buffer_));
}
}  // namespace hortor_servo