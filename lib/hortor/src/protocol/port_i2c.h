// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "hortor.h"
#include "port.h"
#include "protocol.h"

namespace hortor::protocol {

class PortI2C : public Port<PortI2C> {
 public:
  Error Init(TwoWire* wire) {
    wire_ = wire;
    return Error::kOk;
  }

  Error ProcessImpl(InstProtocol& protocol,
                    const float   dt,
                    InstPacket&   inst_packet,
                    bool&         is_complete);

  Error ResponseImpl(const StatusPacket& packet, const uint8_t reply_idx);

  Error OnReceive(int howMany);

  Error OnRequest();

 private:
  TwoWire* wire_ = nullptr;
};

}  // namespace hortor::protocol

namespace hortor::protocol {

inline Error PortI2C::ProcessImpl(InstProtocol& protocol,
                                  const float   dt,
                                  InstPacket&   inst_packet,
                                  bool&         is_complete) {
  while (wire_->available()) {
    uint8_t data = wire_->read();
    CHECK(protocol.Process(inst_packet, data, is_complete));
  }
  return Error::kOk;
}

inline Error PortI2C::ResponseImpl(const StatusPacket& packet,
                                   const uint8_t       reply_idx) {
  const size_t size = packet.GetBufferSize();
  memcpy(status_packet_.buffer, packet.buffer, size);
  return Error::kOk;
}

inline Error PortI2C::OnReceive(int howMany) {
  return Error::kOk;
}

inline Error PortI2C::OnRequest() {
  const size_t size = status_packet_.GetBufferSize();
  wire_->write(status_packet_.buffer, size);
  return Error::kOk;
}

}  // namespace hortor::protocol
