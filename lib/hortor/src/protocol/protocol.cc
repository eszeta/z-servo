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

#include "protocol.h"

#include <Arduino.h>

namespace hortor::protocol {

InstProtocol::InstProtocol() = default;

InstProtocol::~InstProtocol() = default;

Error InstProtocol::Process(InstPacket &packet,
                            const uint8_t recv_data,
                            bool &is_complete) {
  is_complete = false;
  switch (packet_state_) {
    case PacketState::kHeader1: {
      if (recv_data == 0xff) {
        packet_state_ = PacketState::kHeader2;
        packet.header1 = recv_data;
      }
      break;
    }
    case PacketState::kHeader2: {
      if (recv_data == 0xff) {
        packet_state_ = PacketState::kId;
        packet.header2 = recv_data;
      } else {
        packet_state_ = PacketState::kHeader1;
        return Error::kInvalidPacket;
      }
      break;
    }
    case PacketState::kId: {
      packet.id = recv_data;
      packet_state_ = PacketState::kLength;
      break;
    }
    case PacketState::kLength: {
      // Header(2Byte) + ID + Length = 4
      if (recv_data + 4 > InstPacket::kBufferCapacity || recv_data < 2) {
        packet_state_ = PacketState::kHeader1;
        return Error::kInvalidPacket;
      } else {
        packet.length = recv_data;
        packet_state_ = PacketState::kInstructionOrError;
      }
      break;
    }
    case PacketState::kInstructionOrError: {
      packet.instructionOrError = recv_data;
      const uint8_t param_size = packet.GetParameterSize();
      if (param_size > 0) {
        param_pos_ = 0;
        packet_state_ = PacketState::kParameter;
      } else {
        packet_state_ = PacketState::kChecksum;
      }
      break;
    }
    case PacketState::kParameter: {
      const uint8_t param_size = packet.GetParameterSize();
      packet.parameter[param_pos_] = recv_data;
      param_pos_ += 1;
      if (param_pos_ == param_size) {
        packet_state_ = PacketState::kChecksum;
      }
      break;
    }
    case PacketState::kChecksum: {
      const uint8_t checksum = packet.CalculateChecksum();
      packet.SetChecksum(recv_data);
      is_complete = true;
      if (checksum != recv_data) {
        packet_state_ = PacketState::kHeader1;
        return Error::kChecksumError;
      }
      break;
    }
    default: {
      packet_state_ = PacketState::kHeader1;
      return Error::kInvalidState;
    }
  }
  return Error::kOk;
}

Error InstProtocol::CreateResponse(const uint8_t id,
                                   const StatusErrorBits &status,
                                   const uint8_t *parameter,
                                   const size_t parameter_size,
                                   StatusPacket &packet) {
  packet.header1 = 0xff;
  packet.header2 = 0xff;
  packet.id = id;
  packet.instructionOrError = status.value;
  if (parameter == nullptr) {
    packet.SetParameterSize(0);
  } else {
    packet.SetParameterSize(parameter_size);
    memcpy(packet.parameter, parameter, parameter_size);
  }
  packet.SetChecksum(packet.CalculateChecksum());
  return Error::kOk;
}

}  // namespace hortor::protocol