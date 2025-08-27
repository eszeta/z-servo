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
#include "inst_port_i2c.h"

#include <Arduino.h>
#include <Wire.h>

#include "core/types.h"
#include "inst/inst_protocol.h"
#include "inst/inst_types.h"

namespace hortor_servo {

Error InstPortI2c::Process(float dt) { return Error::kOk; }

Error InstPortI2c::Response(const uint8_t reply_idx, const StatusPacket *packet) {
  const size_t size = packet->GetBufferSize();
  std::copy(packet->buffer, packet->buffer + size, status_packet_.buffer);
  return Error::kOk;
}

Error InstPortI2c::OnReceive(int howMany) {
  bool is_complete = false;
  while (wire_->available()) {
    uint8_t data = wire_->read();
    CHECK(protocol_.Process(inst_packet_, data, &is_complete));
    if (is_complete) {
      CHECK(execute_(&inst_packet_));
    }
  }
  return Error::kOk;
}

Error InstPortI2c::OnRequest() {
  const size_t size = status_packet_.GetBufferSize();
  wire_->write(status_packet_.buffer, size);
  return Error::kOk;
}
}  // namespace hortor_servo