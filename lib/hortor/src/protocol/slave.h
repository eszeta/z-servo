// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "base/servo.h"
#include "channel.h"
#include "hortor.h"
#include "protocol.h"
#include "types.h"

namespace hortor::protocol {

/**
 * @brief 协议从机类（CRTP 基类）
 *
 * @tparam DerivedType 派生类类型（CRTP）
 * @tparam RegmapType 寄存器映射类型
 * @tparam ChannelType 协议通道类型（ProtocolChannel<TransportI2C> 等）
 */
template <typename DerivedType, typename RegmapType, typename ChannelType>
class Slave : public hortor::Noncopyable {
 public:
  /**
   * @brief 初始化
   * @return 错误码
   */
  Error Init();

  /**
   * @brief 处理指令
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error Process(float dt);

  /**
   * @brief 处理指令实现（派生类可重写）
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error AfterProcess(float dt);

  /**
   * @brief 获取寄存器映射
   * @return 寄存器映射
   */
  RegmapType* regmap();
  void        set_regmap(RegmapType* regmap);

  /**
   * @brief 获取协议通道
   * @return 协议通道
   */
  ChannelType* channel();
  void         set_channel(ChannelType* channel);

  void    set_id(const uint8_t id);
  uint8_t id() const;
  void    set_return_level(const uint8_t return_level);
  uint8_t return_level() const;

 protected:
  Error Execute(const InstPacket& packet);

  Error Response(const uint8_t  reply_idx,
                 const uint8_t* parameter,
                 const size_t   parameter_size);

  Error WriteRegs(const uint8_t  address,
                  const uint8_t* data,
                  const size_t   size);

  Error AfterWriteRegs(const uint8_t  address,
                       const uint8_t* data,
                       const size_t   size);

  Error PingHandler(const InstPacket& packet);
  Error ReadDataHandler(const InstPacket& packet, const bool response);
  Error WriteDataHandler(const InstPacket& packet, const bool response);
  Error RegWriteHandler(const InstPacket& packet, const bool response);
  Error ActionHandler(const InstPacket& packet, const bool response);
  Error SyncWriteHandler(const InstPacket& packet, const bool response);
  Error BulkReadHandler(const InstPacket& packet, const bool response);
  Error ResetHandler(const InstPacket& packet, const bool response);
  Error AfterResetHandler(const InstPacket& packet, const bool response);
  Error InstructionError(const bool response);
  bool  CheckResponse(const uint8_t instruction) const;

  /**
   * @brief slaveID
   */
  uint8_t id_ = 0;
  /**
   * @brief 返回级别
   */
  uint8_t return_level_ = 0;
  /**
   * @brief 状态
   */
  StatusErrorBits status_{};
  /**
   * @brief 寄存器映射指针
   */
  RegmapType* regmap_ = nullptr;
  /**
   * @brief 协议通道
   */
  ChannelType* channel_ = nullptr;
  /**
   * @brief 异步写缓冲区
   */
  uint8_t async_write_buffer_[128] = {};
  /**
   * @brief 异步写缓冲区大小
   */
  size_t async_write_buffer_size_ = 0;

  /**
   * @brief 读数据缓冲区（ReadData/BulkRead 共用，避免栈上大分配）
   */
  uint8_t read_buffer_[128] = {};

};

}  // namespace hortor::protocol

namespace hortor::protocol {

template <typename DerivedType, typename RegmapType, typename ChannelType>
Error Slave<DerivedType, RegmapType, ChannelType>::Init() {
  return Error::kOk;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
Error Slave<DerivedType, RegmapType, ChannelType>::Process(float dt) {
  bool is_complete = false;
  CHECK(channel_->Process(dt, is_complete));
  if (is_complete) {
    CHECK(Execute(channel_->inst_packet()));
  }
  CHECK(AfterProcess(dt));
  return Error::kOk;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
Error Slave<DerivedType, RegmapType, ChannelType>::AfterProcess(float dt) {
  return static_cast<DerivedType*>(this)->AfterProcessImpl(dt);
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
RegmapType* Slave<DerivedType, RegmapType, ChannelType>::regmap() {
  return regmap_;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
void Slave<DerivedType, RegmapType, ChannelType>::set_regmap(RegmapType* regmap) {
  regmap_ = regmap;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
ChannelType* Slave<DerivedType, RegmapType, ChannelType>::channel() {
  return channel_;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
void Slave<DerivedType, RegmapType, ChannelType>::set_channel(
    ChannelType* channel) {
  channel_ = channel;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
void Slave<DerivedType, RegmapType, ChannelType>::set_id(const uint8_t id) {
  id_ = id;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
uint8_t Slave<DerivedType, RegmapType, ChannelType>::id() const {
  return id_;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
void Slave<DerivedType, RegmapType, ChannelType>::set_return_level(
    const uint8_t return_level) {
  return_level_ = return_level;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
uint8_t Slave<DerivedType, RegmapType, ChannelType>::return_level() const {
  return return_level_;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
Error Slave<DerivedType, RegmapType, ChannelType>::Execute(
    const InstPacket& packet) {
  const auto isBroadcast = packet.id == kBroadcastId;
  const auto isSelf      = packet.id == id_;
  if (!isSelf && !isBroadcast) {
    return Error::kOk;
  }
  const auto instruction    = packet.instructionOrError;
  const auto response       = CheckResponse(instruction);
  status_.instruction_error = false;
  status_.range_error       = false;
  switch (instruction) {
    case Instruction::kPing:
      CHECK(PingHandler(packet));
      break;
    case Instruction::kReadData:
      CHECK(ReadDataHandler(packet, response));
      break;
    case Instruction::kWriteData:
      CHECK(WriteDataHandler(packet, response));
      break;
    case Instruction::kRegWrite:
      CHECK(RegWriteHandler(packet, response));
      break;
    case Instruction::kAction:
      CHECK(ActionHandler(packet, response));
      break;
    case Instruction::kReset:
      CHECK(ResetHandler(packet, response));
      break;
    case Instruction::kSyncWrite:
      CHECK(SyncWriteHandler(packet, response));
      break;
    case Instruction::kBulkRead:
      CHECK(BulkReadHandler(packet, response));
      break;
    default:
      CHECK(InstructionError(response));
      break;
  }
  return Error::kOk;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
Error Slave<DerivedType, RegmapType, ChannelType>::Response(
    const uint8_t  reply_idx,
    const uint8_t* parameter,
    const size_t   parameter_size) {
  StatusPacket packet;
  CHECK(channel_->parser()->CreateResponse(id_, status_, parameter,
                                          parameter_size, packet));
  CHECK(channel_->Response(packet, reply_idx));
  return Error::kOk;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
Error Slave<DerivedType, RegmapType, ChannelType>::WriteRegs(const uint8_t address,
                                                          const uint8_t* data,
                                                          const size_t   size) {
  VERIFY(data != nullptr && size != 0, Error::kInvalidArg);
  CHECK(regmap_->Write(address, data, size));
  CHECK(AfterWriteRegs(address, data, size));
  return Error::kOk;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
Error Slave<DerivedType, RegmapType, ChannelType>::AfterWriteRegs(
    const uint8_t  address,
    const uint8_t* data,
    const size_t   size) {
  return static_cast<DerivedType*>(this)->AfterWriteRegsImpl(address, data,
                                                             size);
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
Error Slave<DerivedType, RegmapType, ChannelType>::PingHandler(
    const InstPacket& packet) {
  CHECK(Response(0, nullptr, 0));
  return Error::kOk;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
Error Slave<DerivedType, RegmapType, ChannelType>::ReadDataHandler(
    const InstPacket& packet,
    const bool        response) {
  const uint8_t address = packet.parameter[0];
  const uint8_t size    = packet.parameter[1];
  CHECK(regmap_->Read(address, size, read_buffer_));
  if (response) {
    CHECK(Response(0, read_buffer_, size));
  }
  return Error::kOk;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
Error Slave<DerivedType, RegmapType, ChannelType>::WriteDataHandler(
    const InstPacket& packet,
    const bool        response) {
  const uint8_t address = packet.parameter[0];
  const uint8_t size    = packet.GetParameterSize() - 1;
  CHECK(WriteRegs(address, packet.parameter + 1, size));
  if (response) {
    CHECK(Response(0, nullptr, 0));
  }
  return Error::kOk;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
Error Slave<DerivedType, RegmapType, ChannelType>::RegWriteHandler(
    const InstPacket& packet,
    const bool        response) {
  const size_t size = packet.GetBufferSize();
  if (async_write_buffer_size_ + size > sizeof(async_write_buffer_)) {
    return Error::kOutOfRange;
  }
  memcpy(async_write_buffer_ + async_write_buffer_size_, packet.buffer, size);
  async_write_buffer_size_ += size;
  if (response) {
    CHECK(Response(0, nullptr, 0));
  }
  return Error::kOk;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
Error Slave<DerivedType, RegmapType, ChannelType>::ActionHandler(
    const InstPacket& packet,
    const bool        response) {
  const uint8_t* buffer = async_write_buffer_;
  if (async_write_buffer_size_ == 0) {
    if (response) {
      status_.instruction_error = true;
      CHECK(Response(0, nullptr, 0));
    }
    return Error::kOk;
  }
  while (async_write_buffer_size_ > 0) {
    const InstPacket* const reg_write_packet =
        reinterpret_cast<const InstPacket*>(buffer);
    const size_t packet_size = reg_write_packet->GetBufferSize();
    constexpr size_t kMinPacketSize = 6;  // header(2)+id(1)+length(1)+inst(1)+chk(1)
    if (async_write_buffer_size_ < packet_size || packet_size < kMinPacketSize ||
        packet_size > InstPacket::kBufferCapacity) {
      break;
    }
    const uint8_t address = reg_write_packet->parameter[0];
    const uint8_t size    = reg_write_packet->GetParameterSize() - 1;
    CHECK(WriteRegs(address, reg_write_packet->parameter + 1, size));
    async_write_buffer_size_ -= packet_size;
    buffer += packet_size;
  }
  async_write_buffer_size_ = 0;
  if (response) {
    CHECK(Response(0, nullptr, 0));
  }
  return Error::kOk;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
Error Slave<DerivedType, RegmapType, ChannelType>::SyncWriteHandler(
    const InstPacket& packet,
    const bool        response) {
  const uint8_t  address        = packet.parameter[0];
  const uint8_t  data_size      = packet.parameter[1];
  const uint8_t  parameter_size = packet.GetParameterSize();
  const uint8_t  block_size     = data_size + 1;
  const uint8_t  block_count    = (parameter_size - 2) / block_size;
  const uint8_t* parameter      = packet.parameter + 2;
  for (uint8_t i = 0; i < block_count; i++) {
    const uint8_t target_id = parameter[0];
    if (id_ == target_id) {
      const uint8_t* data = parameter + 1;
      CHECK(WriteRegs(address, data, data_size));
    }
    parameter += block_size;
  }
  if (response) {
    CHECK(Response(0, nullptr, 0));
  }
  return Error::kOk;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
Error Slave<DerivedType, RegmapType, ChannelType>::BulkReadHandler(
    const InstPacket& packet,
    const bool        response) {
  const uint8_t  parameter_size = packet.GetParameterSize();
  const uint8_t  block_count    = (parameter_size - 1) / 3;
  const uint8_t* parameter      = packet.parameter + 1;
  for (uint8_t i = 0; i < block_count; i++) {
    const uint8_t target_id = parameter[1];
    if (id_ == target_id) {
      const uint8_t data_size = parameter[0];
      const uint8_t address   = parameter[2];
      CHECK(regmap_->Read(address, data_size, read_buffer_));
      if (response) {
        CHECK(Response(i, read_buffer_, data_size));
      }
      return Error::kOk;
    }
    parameter += 3;
  }
  return Error::kOk;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
Error Slave<DerivedType, RegmapType, ChannelType>::ResetHandler(
    const InstPacket& packet,
    const bool        response) {
  if (response) {
    CHECK(Response(0, nullptr, 0));
  }
  CHECK(AfterResetHandler(packet, response));
  return Error::kOk;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
Error Slave<DerivedType, RegmapType, ChannelType>::AfterResetHandler(
    const InstPacket& packet,
    const bool        response) {
  return static_cast<DerivedType*>(this)->AfterResetHandlerImpl(packet,
                                                                response);
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
Error Slave<DerivedType, RegmapType, ChannelType>::InstructionError(
    const bool response) {
  status_.instruction_error = true;
  if (response) {
    CHECK(Response(0, nullptr, 0));
  }
  return Error::kBadData;
}

template <typename DerivedType, typename RegmapType, typename ChannelType>
bool Slave<DerivedType, RegmapType, ChannelType>::CheckResponse(
    const uint8_t instruction) const {
  switch (return_level_) {
    case 0:
      return instruction == Instruction::kPing;
    case 1:
      return instruction == Instruction::kPing ||
             instruction == Instruction::kReadData ||
             instruction == Instruction::kBulkRead;
    case 2:
      return true;
  }
  return false;
}

}  // namespace hortor::protocol