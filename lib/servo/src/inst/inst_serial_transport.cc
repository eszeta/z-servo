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
#include "inst_serial_transport.h"

#include <Arduino.h>

#include "./inst_types.h"

#ifdef ARDUINO_ARCH_STM32
#include <HardwareSerial.h>
#endif

namespace hortor_servo {

Error InstSerialTransport::Process(uint32_t dt) {
  // 延时回包
  delay_time_ -= dt;
  if (delay_time_ <= 0 && is_dirty_) {
    is_dirty_ = false;
    const size_t buffer_size = inst_utils::GetBufferSize(tx_buffer_);
    const size_t size = serial_->write(tx_buffer_, buffer_size);
    if (size != buffer_size) {
      return Error::kIOErr;
    }
  }

  // 接收数据
  while (serial_->available()) {
    uint8_t data = serial_->read();
    CHECK(Receive(data));
  }
  return Error::kOk;
}

Error InstSerialTransport::Receive(uint8_t data) {
  switch (packet_state_) {
    case PacketState::kHeader1: {
      if (data == 0xff) {
        packet_state_ = PacketState::kHeader2;
        rx_buffer_[PacketIndex::kHeader1] = data;
      }
      break;
    }
    case PacketState::kHeader2: {
      if (data == 0xff) {
        packet_state_ = PacketState::kId;
        rx_buffer_[PacketIndex::kHeader2] = data;
      } else {
        packet_state_ = PacketState::kHeader1;
        return Error::kInvalidPacket;
      }
      break;
    }
    case PacketState::kId: {
      rx_buffer_[PacketIndex::kId] = data;
      packet_state_ = PacketState::kLength;
      break;
    }
    case PacketState::kLength: {
      rx_buffer_[PacketIndex::kLength] = data;
      param_pos_ = 0;
      packet_state_ = PacketState::kInstruction;
      break;
    }
    case PacketState::kInstruction: {
      rx_buffer_[PacketIndex::kInstruction] = data;
      packet_state_ = PacketState::kParameter;
      break;
    }
    case PacketState::kParameter: {
      const uint8_t param_size = inst_utils::GetParameterSize(rx_buffer_);
      rx_buffer_[PacketIndex::kParameter + param_pos_] = data;
      param_pos_ += 1;
      if (param_pos_ == param_size) {
        packet_state_ = PacketState::kChecksum;
      }
      break;
    }
    case PacketState::kChecksum: {
      const uint8_t checksum = inst_utils::GetChecksum(rx_buffer_);
      if (checksum != data) {
        packet_state_ = PacketState::kHeader1;
        return Error::kInvalidPacket;
      }
      CHECK(execute_(rx_buffer_));
      break;
    }
    default: {
      packet_state_ = PacketState::kHeader1;
      return Error::kInvalidPacket;
    }
  }
  return Error::kOk;
}

Error InstSerialTransport::Response(const uint8_t reply_idx, const uint8_t *data) {
  const size_t size = inst_utils::GetBufferSize(data);
  memcpy(tx_buffer_, data, size);
  delay_time_ = response_delay_ * (reply_idx + 1);
  is_dirty_ = true;
  return Error::kOk;
}
}  // namespace hortor_servo