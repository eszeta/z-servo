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

namespace hortor {
/**
 * @def CHECK
 * @brief 错误处理辅助宏
 * @param x 要检查的表达式
 *
 * 执行表达式并检查返回的错误码。如果发生错误，立即返回该错误码。
 * 用于简化连续操作的错误处理。
 */
#define CHECK(x)                               \
  do {                                         \
    hortor::Error err = (x);                   \
    if (err != hortor::Error::kOk) return err; \
  } while (0)

/**
 * @brief 错误码枚举
 *
 * 定义MT6701传感器操作可能返回的错误码，用于诊断问题。
 */
enum class Error : uint8_t {
  /**
   * @brief 成功，操作正常完成
   */
  kOk = 0,
  /**
   * @brief 一般错误，未指定具体原因
   */
  kGeneralErr = 1,
  /**
   * @brief IO错误，通信过程中发生错误
   */
  kIOErr = 3,
  /**
   * @brief 超出范围，参数值超出有效范围
   */
  kOutOfRange = 4,
  /**
   * @brief 无效参数，参数组合无效
   */
  kInvalidParameter = 5,
  /**
   * @brief 未初始化，设备未完成初始化
   */
  kUninitialized = 6,
  /**
   * @brief 无效包，包格式错误
   */
  kInvalidPacket = 7,
  /**
   * @brief 无效指令，指令码无效
   */
  kInvalidInstruction = 8,
  /**
   * @brief 数组越界
   */
  kArrayOutOfRange = 9,
  /**
   * @brief 无效状态
   */
  kInvalidState = 10,
  /**
   * @brief 未实现
   */
  kNotImplemented = 11,
  /**
   * @brief 模式不支持
   */
  kModeNotSupport = 12,
};
}  // namespace hortor