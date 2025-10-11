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

#include "i2c_port_handler.h"

#include <Arduino.h>
#include <Wire.h>

#include "protocol.h"

namespace hortor::protocol {

Error InstI2cPortHandler::ProcessImpl(InstProtocol &protocol,
                                      const float dt,
                                      InstPacket &inst_packet,
                                      bool &is_complete) {
  while (wire_->available()) {
    uint8_t data = wire_->read();
    CHECK(protocol.Process(inst_packet, data, is_complete));
  }
  return Error::kOk;
}

Error InstI2cPortHandler::ResponseImpl(const StatusPacket &packet,
                                       const uint8_t reply_idx) {
  const size_t size = packet.GetBufferSize();
  memcpy(status_packet_.buffer, packet.buffer, size);
  return Error::kOk;
}

Error InstI2cPortHandler::OnReceive(int howMany) { return Error::kOk; }

Error InstI2cPortHandler::OnRequest() {
  const size_t size = status_packet_.GetBufferSize();
  wire_->write(status_packet_.buffer, size);
  return Error::kOk;
}

}  // namespace hortor::protocol