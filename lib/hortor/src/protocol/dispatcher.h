// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dispatcher.h
 * @brief 协议指令分发器（回调驱动）
 *
 * 负责：
 * - 管理指令处理器回调表
 * - 接收完整指令包后分发到对应回调
 * - 管理协议参数（ID、返回级别、状态）
 * - 生成并发送响应
 *
 * 特点：
 * - 不涉及寄存器操作，完全由回调处理业务逻辑
 * - Handler 直接填充响应数据
 * - RegmapType 完全可选
 */

#pragma once

#include <Arduino.h>

#include <functional>

#include "error.h"
#include "noncopyable.h"
#include "protocol/protocol.h"
#include "protocol/protocol_types.h"

namespace hortor::protocol {

/**
 * @brief 指令处理器回调签名
 *
 * @param packet 指令包（只读）
 * @param status 状态错误位（可修改）
 * @param response_data 响应数据缓冲（Handler 填充）
 * @param response_size 响应数据大小（Handler 设置）
 * @return 错误码
 */
using InstHandler = std::function<Error(const InstPacket&, StatusErrorBits&, uint8_t*, size_t&)>;

/**
 * @brief 协议指令分发器（CRTP-free，回调驱动）
 *
 * @tparam ChannelType 协议通道类型（Channel<TransportI2C> 等）
 */
template <typename ChannelType>
class Dispatcher : public hortor::Noncopyable {
 public:
  /**
   * @brief 初始化分发器，绑定协议通道
   * @param channel 协议通道指针
   * @return 错误码
   */
  Error Init(ChannelType* channel);

  /**
   * @brief 主循环：收包 → 分发 → 回复
   * @param dt 时间间隔（秒）
   * @return 错误码
   */
  Error Process(float dt);

  /**
   * @brief 注册指令处理器回调
   * @param instruction 指令码
   * @param handler 处理器回调函数
   */
  void RegisterHandler(uint8_t instruction, const InstHandler& handler);

  /**
   * @brief 设置从机 ID
   * @param id 从机 ID (0–252)
   */
  void set_id(uint8_t id) { id_ = id; }

  /** @brief 获取从机 ID */
  uint8_t id() const { return id_; }

  /**
   * @brief 设置返回级别 (0/1/2)
   * @param level 返回级别
   */
  void set_return_level(uint8_t level) { return_level_ = level; }

  /** @brief 获取返回级别 */
  uint8_t return_level() const { return return_level_; }

  /**
   * @brief 获取状态错误位（可修改）
   * @return 状态引用
   */
  StatusErrorBits& status() { return status_; }

  /**
   * @brief 发送响应包
   * @param parameter 参数区指针（可为 nullptr）
   * @param parameter_size 参数区长度
   * @param reply_idx 回复索引（用于多回复延迟）
   * @return 错误码
   */
  Error SendResponse(const uint8_t* parameter, size_t parameter_size, uint8_t reply_idx = 0);

 private:
  /**
   * @brief 根据 return_level_ 判断是否需要回复
   * @param instruction 指令码
   * @return 需要回复为 true
   */
  bool CheckResponse(uint8_t instruction) const;

  ChannelType*    channel_      = nullptr;     ///< 协议通道
  uint8_t         id_           = 0;           ///< 从机 ID
  uint8_t         return_level_ = 0;           ///< 返回级别 0/1/2
  StatusErrorBits status_{};                   ///< 状态错误位
  InstHandler     handlers_[256]{};            ///< 回调表
  uint8_t         response_buffer_[128] = {};  ///< 响应数据暂存
};

}  // namespace hortor::protocol

namespace hortor::protocol {

template <typename ChannelType>
Error Dispatcher<ChannelType>::Init(ChannelType* channel) {
  channel_ = channel;
  return Error::kOk;
}

template <typename ChannelType>
Error Dispatcher<ChannelType>::Process(float dt) {
  bool is_complete = false;
  CHECK(channel_->Process(dt, is_complete));

  if (!is_complete) {
    return Error::kOk;
  }

  const auto& packet       = channel_->inst_packet();
  const auto  is_broadcast = packet.id == kBroadcastId;
  const auto  is_self      = packet.id == id_;

  if (!is_self && !is_broadcast) {
    return Error::kOk;
  }

  const auto  instruction = packet.instructionOrError;
  const auto& handler     = handlers_[instruction];

  // 重置状态
  status_.instruction_error = false;
  status_.range_error       = false;

  if (!handler) {
    // 未注册的指令
    status_.instruction_error = true;
    if (CheckResponse(instruction)) {
      CHECK(SendResponse(nullptr, 0));
    }
    return Error::kBadData;
  }

  // 调用回调处理
  uint8_t response_data[128]{};
  size_t  response_size = 0;
  CHECK(handler(packet, status_, response_data, response_size));

  // 如果需要回复
  if (CheckResponse(instruction)) {
    CHECK(SendResponse(response_data, response_size));
  }

  return Error::kOk;
}

template <typename ChannelType>
void Dispatcher<ChannelType>::RegisterHandler(uint8_t instruction, const InstHandler& handler) {
  handlers_[instruction] = handler;
}

template <typename ChannelType>
Error Dispatcher<ChannelType>::SendResponse(const uint8_t* parameter,
                                            size_t         parameter_size,
                                            uint8_t        reply_idx) {
  StatusPacket packet;
  CHECK(channel_->parser()->CreateResponse(id_, status_, parameter, parameter_size, packet));
  CHECK(channel_->Response(packet, reply_idx));
  return Error::kOk;
}

template <typename ChannelType>
bool Dispatcher<ChannelType>::CheckResponse(uint8_t instruction) const {
  switch (return_level_) {
    case 0:
      return instruction == Instruction::kPing;
    case 1:
      return instruction == Instruction::kPing || instruction == Instruction::kReadData ||
             instruction == Instruction::kBulkRead;
    case 2:
      return true;
  }
  return false;
}

}  // namespace hortor::protocol
