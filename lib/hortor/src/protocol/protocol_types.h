// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file protocol_types.h
 * @brief 协议相关类型（包索引、指令码、控制表、状态错误位）
 */

#pragma once

#include <Arduino.h>

#include "noncopyable.h"
#include "regmap/reg_field.h"

namespace hortor::protocol {
/**
 * @brief 指令包索引
 */
namespace PacketIndex {
constexpr uint8_t kHeader1     = 0;
constexpr uint8_t kHeader2     = 1;
constexpr uint8_t kId          = 2;
constexpr uint8_t kLength      = 3;
constexpr uint8_t kInstruction = 4;
constexpr uint8_t kError       = 4;
constexpr uint8_t kParameter   = 5;
};  // namespace PacketIndex

namespace Instruction {
enum : uint8_t {
  kPing      = 0x01,
  kReadData  = 0x02,
  kWriteData = 0x03,
  kRegWrite  = 0x04,
  kAction    = 0x05,
  kReset     = 0x06,
  kReboot    = 0x08,
  kSyncWrite = 0x82,
  kBulkRead  = 0x92,
};
}  // namespace Instruction

constexpr uint8_t kHeaderByte  = 0xff;
constexpr uint8_t kBroadcastId = 0xfe;

template <typename T, uint8_t Address>
using ControlTableItem = regmap::Field<T, Address, 0, sizeof(T) * 8>;

template <uint8_t Address>
using ControlTableItemU8 = ControlTableItem<uint8_t, Address>;
template <uint8_t Address>
using ControlTableItemU16 = ControlTableItem<uint16_t, Address>;
template <uint8_t Address>
using ControlTableItemU32 = ControlTableItem<uint32_t, Address>;
template <uint8_t Address>
using ControlTableItemS8 = ControlTableItem<int8_t, Address>;
template <uint8_t Address>
using ControlTableItemS16 = ControlTableItem<int16_t, Address>;
template <uint8_t Address>
using ControlTableItemS32 = ControlTableItem<int32_t, Address>;
template <uint8_t Address>
using ControlTableItemB8 = ControlTableItem<bool, Address>;

template <typename BeginType, typename EndType>
struct ControlTableBlock : public hortor::Noncopyable {
  static constexpr uint8_t kBegin = BeginType::kAddress;
  static constexpr uint8_t kEnd   = EndType::kAddress + EndType::kSize;

  static constexpr uint8_t size() { return kEnd - kBegin; }
  static constexpr bool    InBlock(const uint8_t address, const uint8_t access_size) {
    return address < kEnd && address + access_size > kBegin;
  }
};

union StatusErrorBits {
  uint8_t value = 0;
  struct {
    /**
     * @brief 输入电压超出范围 bit0
     * @note 当输入电压超出电压限制时，它会被设为1
     */
    bool input_voltage_error : 1;
    /**
     * @brief 角度超出范围 bit1
     * @note 当角度超出角度限制时，它会被设为1
     */
    bool angle_limit_error : 1;
    /**
     * @brief 温度超过上限 bit2
     * @note 当温度超过温度限制时，它会被设为1
     */
    bool overheating_error : 1;
    /**
     * @brief 范围错误 bit3
     * @note 当指令超出使用范围时，它会被设为1
     */
    bool range_error : 1;
    /**
     * @brief 校验和错误 bit4
     * @note 当所传输的指令数据包的校验和不正确时，它会被设为1
     */
    bool checksum_error : 1;
    /**
     * @brief 过载错误 bit5
     * @note 当当前负载无法通过设定的扭矩进行控制时，其值设为1
     */
    bool overload_error : 1;
    /**
     * @brief 指令错误 bit6
     * @note 若发送未定义的指令或在未发送Reg
     * Write指令的情况下发送action指令，则将其设为1
     */
    bool instruction_error : 1;
    bool reserved_bit7     : 1;  ///< 位7 保留
  };
};

}  // namespace hortor::protocol