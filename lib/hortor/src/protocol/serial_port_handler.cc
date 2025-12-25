// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include "serial_port_handler.h"

#include <Arduino.h>

#include "math/math.h"
#include "port_handler.h"
#include "protocol.h"

#ifdef ARDUINO_ARCH_STM32
#include <HardwareSerial.h>
#endif

namespace hortor::protocol {

Error SerialPortHandler::ProcessImpl(InstProtocol& protocol,
                                     const float dt,
                                     InstPacket& inst_packet,
                                     bool& is_complete) {
  // 延时回包
  delay_time_ -= dt;
  if (delay_time_ <= 0 && response_pending_) {
    response_pending_ = false;
    const size_t buffer_size = status_packet_.GetBufferSize();
    const size_t size = serial_->write(status_packet_.buffer, buffer_size);
    if (size != buffer_size) {
      return Error::kIOErr;
    }
  }

  // 接收数据
  while (serial_->available()) {
    uint8_t data = serial_->read();
    CHECK(protocol.Process(inst_packet, data, is_complete));
  }
  return Error::kOk;
}

Error SerialPortHandler::ResponseImpl(const StatusPacket& packet,
                                      const uint8_t reply_idx) {
  const size_t size = packet.GetBufferSize();
  memcpy(status_packet_.buffer, packet.buffer, size);
  delay_time_ = response_delay_ * (reply_idx + 1);
  response_pending_ = true;
  return Error::kOk;
}

}  // namespace hortor::protocol