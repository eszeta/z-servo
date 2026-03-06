// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "base/servo.h"
#include "hortor.h"
#include "port_handler.h"
#include "protocol.h"
#include "types.h"

namespace hortor::protocol {

/**
 * @brief 协议从机类（CRTP 基类）
 *
 * @tparam DERIVED 派生类类型（CRTP）
 * @tparam RegMapType 寄存器映射类型
 * @tparam PortHandlerType 端口处理器类型
 */
template <typename DerivedType, typename RegMapType, typename PortHandlerType>
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
  RegMapType* regmap();
  void        set_regmap(RegMapType* regmap);

  /**
   * @brief 获取端口处理器
   * @return 端口处理器
   */
  PortHandlerType* port_handler();
  void             set_port_handler(PortHandlerType* port_handler);

  void    set_id(const uint8_t id);
  uint8_t id() const;
  void    set_return_level(const uint8_t return_level);
  uint8_t return_level() const;

 protected:
  Error Execute(const InstPacket& packet);

  Error Response(const uint8_t reply_idx, const uint8_t* parameter,
                 const size_t parameter_size);

  Error WriteRegs(const uint8_t address, const uint8_t* data,
                  const size_t size);

  Error AfterWriteRegs(const uint8_t address, const uint8_t* data,
                       const size_t size);

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
  RegMapType* regmap_ = nullptr;
  /**
   * @brief 指令传输接口
   */
  PortHandlerType* port_handler_ = nullptr;
  /**
   * @brief 异步写缓冲区
   */
  uint8_t async_write_buffer_[128] = {};
  /**
   * @brief 异步写缓冲区大小
   */
  size_t async_write_buffer_size_ = 0;

  InstProtocol protocol_{};
  InstPacket   inst_packet_{};
  StatusPacket status_packet_{};
};

}  // namespace hortor::protocol

namespace hortor::protocol {

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
Error Slave<DerivedType, RegMapType, PortHandlerType>::Init() {
  return Error::kOk;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
Error Slave<DerivedType, RegMapType, PortHandlerType>::Process(float dt) {
  bool is_complete = false;
  CHECK(port_handler_->Process(protocol_, dt, inst_packet_, is_complete));
  if (is_complete) {
    CHECK(Execute(inst_packet_));
  }
  CHECK(AfterProcess(dt));
  return Error::kOk;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
Error Slave<DerivedType, RegMapType, PortHandlerType>::AfterProcess(float dt) {
  return static_cast<DerivedType*>(this)->AfterProcessImpl(dt);
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
RegMapType* Slave<DerivedType, RegMapType, PortHandlerType>::regmap() {
  return regmap_;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
void Slave<DerivedType, RegMapType, PortHandlerType>::set_regmap(
    RegMapType* regmap) {
  regmap_ = regmap;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
PortHandlerType*
Slave<DerivedType, RegMapType, PortHandlerType>::port_handler() {
  return port_handler_;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
void Slave<DerivedType, RegMapType, PortHandlerType>::set_port_handler(
    PortHandlerType* port_handler) {
  port_handler_ = port_handler;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
void Slave<DerivedType, RegMapType, PortHandlerType>::set_id(const uint8_t id) {
  id_ = id;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
uint8_t Slave<DerivedType, RegMapType, PortHandlerType>::id() const {
  return id_;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
void Slave<DerivedType, RegMapType, PortHandlerType>::set_return_level(
    const uint8_t return_level) {
  return_level_ = return_level;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
uint8_t Slave<DerivedType, RegMapType, PortHandlerType>::return_level() const {
  return return_level_;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
Error Slave<DerivedType, RegMapType, PortHandlerType>::Execute(
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

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
Error Slave<DerivedType, RegMapType, PortHandlerType>::Response(
    const uint8_t reply_idx, const uint8_t* parameter,
    const size_t parameter_size) {
  CHECK(protocol_.CreateResponse(id_, status_, parameter, parameter_size,
                                 status_packet_));
  CHECK(port_handler_->Response(status_packet_, reply_idx));
  return Error::kOk;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
Error Slave<DerivedType, RegMapType, PortHandlerType>::WriteRegs(
    const uint8_t address, const uint8_t* data, const size_t size) {
  VERIFY(data != nullptr && size != 0, Error::kInvalidArg);
  CHECK(regmap_->Write(address, data, size));
  CHECK(AfterWriteRegs(address, data, size));
  return Error::kOk;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
Error Slave<DerivedType, RegMapType, PortHandlerType>::AfterWriteRegs(
    const uint8_t address, const uint8_t* data, const size_t size) {
  return static_cast<DerivedType*>(this)->AfterWriteRegsImpl(address, data,
                                                             size);
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
Error Slave<DerivedType, RegMapType, PortHandlerType>::PingHandler(
    const InstPacket& packet) {
  CHECK(Response(0, nullptr, 0));
  return Error::kOk;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
Error Slave<DerivedType, RegMapType, PortHandlerType>::ReadDataHandler(
    const InstPacket& packet, const bool response) {
  const uint8_t address = packet.parameter[0];
  const uint8_t size    = packet.parameter[1];
  uint8_t       buffer[128];
  CHECK(regmap_->Read(address, size, buffer));
  if (response) {
    CHECK(Response(0, buffer, size));
  }
  return Error::kOk;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
Error Slave<DerivedType, RegMapType, PortHandlerType>::WriteDataHandler(
    const InstPacket& packet, const bool response) {
  const uint8_t address = packet.parameter[0];
  const uint8_t size    = packet.GetParameterSize() - 1;
  CHECK(WriteRegs(address, packet.parameter + 1, size));
  if (response) {
    CHECK(Response(0, nullptr, 0));
  }
  return Error::kOk;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
Error Slave<DerivedType, RegMapType, PortHandlerType>::RegWriteHandler(
    const InstPacket& packet, const bool response) {
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

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
Error Slave<DerivedType, RegMapType, PortHandlerType>::ActionHandler(
    const InstPacket& packet, const bool response) {
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
    if (async_write_buffer_size_ < packet_size) {
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

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
Error Slave<DerivedType, RegMapType, PortHandlerType>::SyncWriteHandler(
    const InstPacket& packet, const bool response) {
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

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
Error Slave<DerivedType, RegMapType, PortHandlerType>::BulkReadHandler(
    const InstPacket& packet, const bool response) {
  const uint8_t  parameter_size = packet.GetParameterSize();
  const uint8_t  block_count    = (parameter_size - 1) / 3;
  uint8_t        buffer[128];
  const uint8_t* parameter = packet.parameter + 1;
  for (uint8_t i = 0; i < block_count; i++) {
    const uint8_t target_id = parameter[1];
    if (id_ == target_id) {
      const uint8_t data_size = parameter[0];
      const uint8_t address   = parameter[2];
      CHECK(regmap_->Read(address, data_size, buffer));
      if (response) {
        CHECK(Response(i, buffer, data_size));
      }
      return Error::kOk;
    }
    parameter += 3;
  }
  return Error::kOk;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
Error Slave<DerivedType, RegMapType, PortHandlerType>::ResetHandler(
    const InstPacket& packet, const bool response) {
  if (response) {
    CHECK(Response(0, nullptr, 0));
  }
  CHECK(AfterResetHandler(packet, response));
  return Error::kOk;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
Error Slave<DerivedType, RegMapType, PortHandlerType>::AfterResetHandler(
    const InstPacket& packet, const bool response) {
  return static_cast<DerivedType*>(this)->AfterResetHandlerImpl(packet,
                                                                response);
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
Error Slave<DerivedType, RegMapType, PortHandlerType>::InstructionError(
    const bool response) {
  status_.instruction_error = true;
  if (response) {
    CHECK(Response(0, nullptr, 0));
  }
  return Error::kBadData;
}

template <typename DerivedType, typename RegMapType, typename PortHandlerType>
bool Slave<DerivedType, RegMapType, PortHandlerType>::CheckResponse(
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