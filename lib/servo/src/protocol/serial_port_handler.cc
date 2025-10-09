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

#include "serial_port_handler.h"

#include <Arduino.h>

#include "math/math.h"
#include "port_handler.h"
#include "protocol.h"

#ifdef ARDUINO_ARCH_STM32
#include <HardwareSerial.h>
#endif

namespace hortor_servo {

Error InstSerialPortHandler::Process(InstProtocol &protocol,
                                     const float dt,
                                     InstPacket &inst_packet,
                                     bool &is_complete) {
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

Error InstSerialPortHandler::Response(const StatusPacket &packet,
                                      const uint8_t reply_idx) {
  const size_t size = packet.GetBufferSize();
  memcpy(status_packet_.buffer, packet.buffer, size);
  delay_time_ = response_delay_ * (reply_idx + 1) * kMilliToSec;  // 毫秒转秒
  response_pending_ = true;
  return Error::kOk;
}

}  // namespace hortor_servo