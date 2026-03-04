// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <stdint.h>

namespace hortor {

/**
 * @brief 错误码枚举
 *
 * 项目通用错误码，用于诊断问题。
 */
enum class Error : uint8_t {
  /** @brief 成功，操作正常完成 */
  kOk = 0,
  /** @brief 通用错误，未指定具体原因 */
  kErr = 1,
  /** @brief IO错误，通信过程中发生错误 */
  kIOErr = 2,
  /** @brief 越界，参数值或索引超出有效范围 */
  kOutOfRange = 3,
  /** @brief 无效参数，参数为空或组合无效 */
  kInvalidArg = 4,
  /** @brief 无效包，包格式错误 */
  kInvalidPacket = 5,
  /** @brief 无效指令，指令码无效 */
  kInvalidInstruction = 6,
  /** @brief 无效状态 */
  kInvalidState = 7,
  /** @brief 校验和错误 */
  kChecksumError = 8,
  /** @brief 不支持的操作 */
  kUnsupported = 9,
};

/**
 * @brief 判断错误码是否为成功
 */
inline bool IsOk(Error e) { return e == Error::kOk; }

}  // namespace hortor

/**
 * @def CHECK
 * @brief 错误传播宏
 *
 * 执行表达式并检查返回的错误码。如果发生错误，立即从当前函数返回该错误码。
 * 使用 __VA_ARGS__ 以兼容模板多参数调用，如 CHECK(ReadField<T, A, B>(v))。
 */
#define CHECK(...)                                   \
  do {                                               \
    ::hortor::Error _err_ = (__VA_ARGS__);           \
    if (_err_ != ::hortor::Error::kOk) return _err_; \
  } while (0)

/**
 * @def VERIFY
 * @brief 前置条件守卫宏
 *
 * 检查条件，若条件为 false，立即从当前函数返回指定错误码。
 * 用于替代 if (!cond) return err; 模式，使前置条件检查更简洁。
 */
#define VERIFY(cond, err)      \
  do {                         \
    if (!(cond)) return (err); \
  } while (0)
