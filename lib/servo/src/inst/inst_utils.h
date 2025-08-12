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
#pragma once
#include <Arduino.h>

#include "inst/inst_types.h"

namespace hortor_servo {
/**
 * @brief 指令包索引
 */
namespace PacketIndex {
static constexpr uint8_t kHeader1 = 0;
static constexpr uint8_t kHeader2 = 1;
static constexpr uint8_t kId = 2;
static constexpr uint8_t kLength = 3;
static constexpr uint8_t kInstruction = 4;
static constexpr uint8_t kError = 4;
static constexpr uint8_t kParameter = 5;
};  // namespace PacketIndex
namespace inst_utils {

/**
 * @brief 获取ID
 * @param buffer 指令包
 * @return ID
 */
static constexpr uint8_t GetBufferId(const uint8_t *buffer) {
  return buffer[PacketIndex::kId];
}

/**
 * @brief 获取指令包长度
 * @param buffer 指令包
 * @return 指令包长度（包括ID、长度、指令/错误和参数的总长度）
 */
static constexpr uint8_t GetLength(const uint8_t *buffer) {
  return buffer[PacketIndex::kLength];
}

/**
 * @brief 获取参数长度
 * @param buffer 指令包
 * @return 参数长度
 */
static constexpr uint8_t GetParameterSize(const uint8_t *buffer) {
  // 去掉 instruction/error 和 checksum
  return GetLength(buffer) - 2;
}

/**
 * @brief 设置参数长度
 * @param buffer 指令包
 * @param size 参数长度（不包括指令/错误和校验和）
 */
static constexpr void SetParameterSize(uint8_t *buffer, const uint8_t size) {
  buffer[PacketIndex::kLength] = size + 2;
}

/**
 * @brief 获取校验和
 * @param buffer 指令包
 * @return 校验和
 */
static constexpr uint8_t GetChecksum(const uint8_t *buffer) {
  const uint8_t parameter_size = GetParameterSize(buffer);
  return buffer[PacketIndex::kParameter + parameter_size];
}

/**
 * @brief 获取指令包总大小
 * @param buffer 指令包
 * @return 指令包总大小（包括头部、ID、长度、指令/错误、参数和校验和）
 */
static constexpr size_t GetBufferSize(const uint8_t *buffer) {
  const uint8_t length = GetLength(buffer);
  // Header(2Byte) + ID + Length = 4
  return length + 4;
}

/**
 * @brief 计算校验和
 * @param buffer 指令包
 * @return 校验和
 */
// Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)
static constexpr uint8_t CalculateChecksum(const uint8_t *buffer) {
  uint16_t checksum = 0;  // 使用16位避免溢出
  // length = ParameterSize + ID + Length
  const uint8_t length = GetParameterSize(buffer) + 2;
  for (uint8_t i = PacketIndex::kId; i < length; i++) {
    checksum += buffer[i];
  }
  return ~(static_cast<uint8_t>(checksum));
}

/**
 * @brief 校验和检查
 * @param buffer 指令包
 * @return true 如果校验和正确，false 如果校验和错误
 */
static constexpr bool CheckChecksum(const uint8_t *buffer) {
  return GetChecksum(buffer) == CalculateChecksum(buffer);
}

/**
 * @brief 设置校验和
 * @param buffer 指令包
 */
static constexpr void SetChecksum(uint8_t *buffer) {
  const uint8_t parameter_size = GetParameterSize(buffer);
  buffer[PacketIndex::kParameter + parameter_size] = CalculateChecksum(buffer);
}
}  // namespace inst_utils
}  // namespace hortor_servo
