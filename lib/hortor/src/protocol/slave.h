// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "base/servo.h"
#include "hortor.h"
#include "port_handler.h"
#include "protocol.h"
#include "regmap.h"
#include "types.h"

namespace hortor::protocol {

/**
 * @brief 协议从机类（CRTP 基类）
 *
 * @tparam Derived 派生类类型（CRTP）
 * @tparam RegMapType 寄存器映射类型
 * @tparam PortHandlerType 端口处理器类型
 */
template <typename Derived, typename RegMapType, typename PortHandlerType>
class Slave {
 public:
  /**
   * @brief 构造函数
   */
  Slave() = default;

  /**
   * @brief 初始化
   * @return 错误码
   */
  Error Init() { return Error::kOk; }

  /**
   * @brief 处理指令
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error Process(float dt) {
    bool is_complete = false;
    CHECK(port_handler_->Process(protocol_, dt, inst_packet_, is_complete));
    if (is_complete) {
      CHECK(Execute(inst_packet_));
    }
    CHECK(AfterProcess(dt));
    return Error::kOk;
  }

  /**
   * @brief 处理指令实现（派生类可重写）
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error AfterProcess(float dt) {
    return static_cast<Derived*>(this)->AfterProcessImpl(dt);
  }

  /**
   * @brief 获取寄存器映射
   * @return 寄存器映射
   */
  RegMapType* regmap() { return regmap_; }
  void LinkRegMap(RegMapType* regmap) { regmap_ = regmap; }
  /**
   * @brief 获取端口处理器
   * @return 端口处理器
   */
  PortHandlerType* port_handler() { return port_handler_; }
  void LinkPortHandler(PortHandlerType* port_handler) {
    port_handler_ = port_handler;
  }

  /**
   * @brief 设置ID
   * @param id ID
   */
  void set_id(const uint8_t id) { id_ = id; }
  /**
   * @brief 获取ID
   * @return ID
   */
  uint8_t id() const { return id_; }
  /**
   * @brief 设置返回级别
   * @param return_level 返回级别
   */
  void set_return_level(const uint8_t return_level) {
    return_level_ = return_level;
  }
  /**
   * @brief 获取返回级别
   * @return 返回级别
   */
  uint8_t return_level() const { return return_level_; }

 protected:
  /**
   * @brief 执行指令
   * @param data 指令数据
   * @return 错误码
   */
  Error Execute(const InstPacket& packet) {
    const auto isBroadcast = packet.id == kBroadcastId;
    const auto isSelf = packet.id == id_;
    if (!isSelf && !isBroadcast) {
      return Error::kOk;
    }
    const auto instruction = packet.instructionOrError;
    const auto response = CheckResponse(instruction);
    status_.instruction_error = false;
    status_.range_error = false;
    switch (instruction) {
      case Instruction::kPing: {
        CHECK(PingHandler(packet));
        break;
      }
      case Instruction::kReadData: {
        CHECK(ReadDataHandler(packet, response));
        break;
      }
      case Instruction::kWriteData: {
        CHECK(WriteDataHandler(packet, response));
        break;
      }
      case Instruction::kRegWrite: {
        CHECK(RegWriteHandler(packet, response));
        break;
      }
      case Instruction::kAction: {
        CHECK(ActionHandler(packet, response));
        break;
      }
      case Instruction::kReset: {
        CHECK(ResetHandler(packet, response));
        break;
      }
      case Instruction::kSyncWrite: {
        CHECK(SyncWriteHandler(packet, response));
        break;
      }
      case Instruction::kBulkRead: {
        CHECK(BulkReadHandler(packet, response));
        break;
      }
      default: {
        CHECK(InstructionError(response));
        break;
      }
    }
    return Error::kOk;
  }

  /**
   * @brief 响应指令
   * @param reply_idx 响应索引
   * @param parameter 参数
   * @param parameter_size 参数大小
   * @return 错误码
   */
  Error Response(const uint8_t reply_idx,
                 const uint8_t* parameter,
                 const size_t parameter_size) {
    CHECK(protocol_.CreateResponse(
        id_, status_, parameter, parameter_size, status_packet_));
    CHECK(port_handler_->Response(status_packet_, reply_idx));
    return Error::kOk;
  }

  /**
   * @brief 写寄存器
   * @param address 地址
   * @param data 数据
   * @param size 大小
   * @return 错误码
   */
  Error WriteRegs(const uint8_t address,
                  const uint8_t* data,
                  const size_t size) {
    if (data == nullptr || size == 0) {
      return Error::kInvalidParameter;
    }
    CHECK(regmap_->WriteBytes(address, data, size));
    CHECK(AfterWriteRegs(address, data, size));
    return Error::kOk;
  }

  /**
   * @brief 写寄存器实现（派生类可重写）
   * @param address 地址
   * @param data 数据
   * @param size 大小
   * @return 错误码
   */
  Error AfterWriteRegs(const uint8_t address,
                       const uint8_t* data,
                       const size_t size) {
    return static_cast<Derived*>(this)->AfterWriteRegsImpl(address, data, size);
  }
  
  /**
   * @brief PING指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error PingHandler(const InstPacket& packet) {
    CHECK(Response(0, nullptr, 0));
    return Error::kOk;
  }
  /**
   * @brief 读取数据指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error ReadDataHandler(const InstPacket& packet, const bool response) {
    const uint8_t address = packet.parameter[0];
    const uint8_t size = packet.parameter[1];
    uint8_t buffer[128];
    CHECK(regmap_->ReadBytes(address, size, buffer));
    if (response) {
      CHECK(Response(0, buffer, size));
    }
    return Error::kOk;
  }
  /**
   * @brief 写数据指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error WriteDataHandler(const InstPacket& packet, const bool response) {
    const uint8_t address = packet.parameter[0];
    const uint8_t size = packet.GetParameterSize() - 1;
    CHECK(WriteRegs(address, packet.parameter + 1, size));
    if (response) {
      CHECK(Response(0, nullptr, 0));
    }
    return Error::kOk;
  }
  /**
   * @brief 写寄存器指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error RegWriteHandler(const InstPacket& packet, const bool response) {
    const size_t size = packet.GetBufferSize();
    if (async_write_buffer_size_ + size > sizeof(async_write_buffer_)) {
      return Error::kArrayOutOfRange;
    }
    memcpy(async_write_buffer_ + async_write_buffer_size_, packet.buffer, size);
    async_write_buffer_size_ += size;
    if (response) {
      CHECK(Response(0, nullptr, 0));
    }
    return Error::kOk;
  }
  /**
   * @brief 执行指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error ActionHandler(const InstPacket& packet, const bool response) {
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
      const uint8_t size = reg_write_packet->GetParameterSize() - 1;
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
  /**
   * @brief 同步写指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error SyncWriteHandler(const InstPacket& packet, const bool response) {
    const uint8_t address = packet.parameter[0];
    const uint8_t data_size = packet.parameter[1];
    const uint8_t parameter_size = packet.GetParameterSize();
    const uint8_t block_size = data_size + 1;
    const uint8_t block_count = (parameter_size - 2) / block_size;
    const uint8_t* parameter = packet.parameter + 2;
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
  /**
   * @brief 批量读取指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error BulkReadHandler(const InstPacket& packet, const bool response) {
    const uint8_t parameter_size = packet.GetParameterSize();
    const uint8_t block_count = (parameter_size - 1) / 3;
    uint8_t response_idx = 0;
    uint8_t buffer[128];
    const uint8_t* parameter = packet.parameter + 1;
    for (uint8_t i = 0; i < block_count; i++) {
      const uint8_t target_id = parameter[1];
      if (id_ == target_id) {
        const uint8_t data_size = parameter[0];
        const uint8_t address = parameter[2];
        response_idx = i;
        CHECK(regmap_->ReadBytes(address, data_size, buffer));
        if (response) {
          CHECK(Response(response_idx, buffer, data_size));
        }
        return Error::kOk;
      }
      parameter += 3;
    }
    return Error::kOk;
  }
  /**
   * @brief 恢复指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error ResetHandler(const InstPacket& packet, const bool response) {
    if (response) {
      CHECK(Response(0, nullptr, 0));
    }
    CHECK(AfterResetHandler(packet, response));
    return Error::kOk;
  }

  /**
   * @brief 批量读取指令实现（派生类可重写）
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error AfterResetHandler(const InstPacket& packet, const bool response) {
    return static_cast<Derived*>(this)->AfterResetHandlerImpl(packet, response);
  }

  /**
   * @brief 指令错误
   * @param response 是否响应
   * @return 错误码
   */
  Error InstructionError(const bool response) {
    status_.instruction_error = true;
    if (response) {
      CHECK(Response(0, nullptr, 0));
    }
    return Error::kInvalidInstruction;
  }

  bool CheckResponse(const uint8_t instruction) const {
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

  /**
   * @brief 处理指令实现（派生类可重写）
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error AfterProcessImpl(float dt) { return Error::kOk; }

  /**
   * @brief 写寄存器实现（派生类可重写）
   * @param address 地址
   * @param data 数据
   * @param size 大小
   * @return 错误码
   */
  Error AfterWriteRegsImpl(const uint8_t address,
                           const uint8_t* data,
                           const size_t size) {
    return Error::kOk;
  }

  /**
   * @brief 恢复指令实现（派生类可重写）
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error AfterResetHandlerImpl(const InstPacket& packet, const bool response) {
    return Error::kOk;
  }

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
  InstPacket inst_packet_{};
  StatusPacket status_packet_{};
};

}  // namespace hortor::protocol