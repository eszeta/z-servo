// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "hortor.h"
#include "protocol.h"
#include "transport.h"

namespace hortor::protocol {

/**
 * @brief I2C 从机传输层
 *
 * OnReceive 中直接解析到 inst_packet，无 rx 缓冲。
 * Write 存储指针，OnRequest 时发送，无 tx 缓冲。
 */
class TransportI2C : public Transport<TransportI2C> {
 public:
  Error Init(TwoWire* wire);

  bool   ReadByteImpl(uint8_t& byte);
  size_t AvailableImpl();
  Error  WriteImpl(const uint8_t* data, const size_t size);
  bool   ConsumePacketImpl();
  void   SetReceiverImpl(InstProtocol* protocol, InstPacket* inst_packet);

  void OnReceive(int howMany);
  void OnRequest();

 private:
  TwoWire* wire_ = nullptr;

  InstProtocol* protocol_ptr_    = nullptr;
  InstPacket*   inst_packet_ptr_ = nullptr;
  volatile bool packet_ready_    = false;

  const uint8_t* tx_data_ = nullptr;
  size_t         tx_size_ = 0;
};

}  // namespace hortor::protocol

namespace hortor::protocol {

inline Error TransportI2C::Init(TwoWire* wire) {
  wire_ = wire;
  return Error::kOk;
}

inline bool TransportI2C::ReadByteImpl(uint8_t& byte) {
  (void)byte;
  return false;
}

inline size_t TransportI2C::AvailableImpl() {
  return 0;
}

inline Error TransportI2C::WriteImpl(const uint8_t* data, const size_t size) {
  tx_data_ = data;
  tx_size_ = size;
  return Error::kOk;
}

inline bool TransportI2C::ConsumePacketImpl() {
  if (!packet_ready_) {
    return false;
  }
  packet_ready_ = false;
  return true;
}

inline void TransportI2C::SetReceiverImpl(InstProtocol* protocol, InstPacket* inst_packet) {
  protocol_ptr_    = protocol;
  inst_packet_ptr_ = inst_packet;
}

inline void TransportI2C::OnReceive(int howMany) {
  (void)howMany;
  if (wire_ == nullptr || protocol_ptr_ == nullptr || inst_packet_ptr_ == nullptr) {
    return;
  }
  while (wire_->available()) {
    bool is_complete = false;
    protocol_ptr_->Process(*inst_packet_ptr_, wire_->read(), is_complete);
    if (is_complete) {
      packet_ready_ = true;
    }
  }
}

inline void TransportI2C::OnRequest() {
  if (wire_ != nullptr && tx_data_ != nullptr && tx_size_ > 0) {
    wire_->write(tx_data_, tx_size_);
  }
}

}  // namespace hortor::protocol
