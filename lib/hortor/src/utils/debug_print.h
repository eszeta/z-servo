// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file debug_print.h
 * @brief 调试输出（可选 DISABLE_DEBUG 关闭）
 */

#pragma once

#include <Arduino.h>

namespace hortor::utils {
#ifndef DISABLE_DEBUG
extern Print* debug_print;  ///< 调试输出目标（Serial 等）
#endif

/**
 * @brief 设置调试输出流
 * @param _debugPrint 输出目标（如 &Serial）
 */
void DebugEnable(Print* _debugPrint);

/**
 * @brief 打印多个参数（无换行）
 * @param args 可变参数（支持 print 的类型）
 */
template <typename... Args>
void DebugPrint(const Args&... args);

/**
 * @brief 打印多个参数并换行
 * @param args 可变参数（无参数时仅换行）
 */
template <typename... Args>
void DebugPrintln(const Args&... args);
}  // namespace hortor::utils

namespace hortor::utils {

#ifndef DISABLE_DEBUG
Print* debug_print = nullptr;

inline void DebugEnable(Print* _debugPrint) {
  debug_print = _debugPrint;
}
#else
inline void DebugEnable(Print* /*_debugPrint*/) {}
#endif

template <typename... Args>
void DebugPrint(const Args&... args) {
#ifndef DISABLE_DEBUG
  (debug_print->print(args), ...);
#endif
}

template <typename... Args>
void DebugPrintln(const Args&... args) {
#ifndef DISABLE_DEBUG
  if constexpr (sizeof...(Args) == 0) {
    debug_print->println();
  } else {
    (debug_print->print(args), ...);
    debug_print->println();
  }
#endif
}

}  // namespace hortor::utils
