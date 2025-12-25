// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include "i2c_port_handler.h"

#include <Arduino.h>
#include <Wire.h>

#include "protocol.h"

namespace hortor::protocol {

Error I2cPortHandler::ProcessImpl(InstProtocol& protocol,
                                  const float dt,
                                  InstPacket& inst_packet,
                                  bool& is_complete) {
  while (wire_->available()) {
    uint8_t data = wire_->read();
    CHECK(protocol.Process(inst_packet, data, is_complete));
  }
  return Error::kOk;
}

Error I2cPortHandler::ResponseImpl(const StatusPacket& packet,
                                   const uint8_t reply_idx) {
  const size_t size = packet.GetBufferSize();
  memcpy(status_packet_.buffer, packet.buffer, size);
  return Error::kOk;
}

Error I2cPortHandler::OnReceive(int howMany) { return Error::kOk; }

Error I2cPortHandler::OnRequest() {
  const size_t size = status_packet_.GetBufferSize();
  wire_->write(status_packet_.buffer, size);
  return Error::kOk;
}

}  // namespace hortor::protocol