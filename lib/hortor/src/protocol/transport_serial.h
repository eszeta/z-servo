// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "hortor.h"
#include "protocol.h"
#include "transport.h"

#ifdef ARDUINO_ARCH_STM32
#include <HardwareSerial.h>
#endif

namespace hortor::protocol {

/**
 * @brief 串口传输层
 *
 * 直接读写 Serial，无缓冲。响应延迟由 ProtocolChannel 处理。
 */
class TransportSerial : public Transport<TransportSerial> {
 public:
  Error Init(HardwareSerial* serial) {
    serial_ = serial;
    return Error::kOk;
  }

  bool ReadByteImpl(uint8_t& byte);
  size_t AvailableImpl();
  Error WriteImpl(const uint8_t* data, const size_t size);
  bool ConsumePacketImpl() { return false; }
  void SetReceiverImpl(InstProtocol*, InstPacket*) {}

 private:
  HardwareSerial* serial_ = nullptr;
};

}  // namespace hortor::protocol

namespace hortor::protocol {

inline bool TransportSerial::ReadByteImpl(uint8_t& byte) {
  if (serial_ == nullptr || serial_->available() == 0) {
    return false;
  }
  byte = serial_->read();
  return true;
}

inline size_t TransportSerial::AvailableImpl() {
  return serial_ != nullptr ? serial_->available() : 0;
}

inline Error TransportSerial::WriteImpl(const uint8_t* data, const size_t size) {
  if (serial_ == nullptr || data == nullptr) {
    return Error::kInvalidArg;
  }
  const size_t written = serial_->write(data, size);
  return written == size ? Error::kOk : Error::kIO;
}

}  // namespace hortor::protocol
