// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

namespace hortor::utils {
#ifndef DISABLE_DEBUG
/**
 * @brief 调试打印
 */
Print *debug_print = nullptr;
#endif

/**
 * @brief 启用调试打印
 * @param _debugPrint 调试打印对象
 */
void DebugEnable(Print *_debugPrint) {
#ifndef DISABLE_DEBUG
  debug_print = _debugPrint;
#endif
}

/**
 * @brief 打印单个值
 * @param val 要打印的值
 */
template <typename T>
void DebugPrint(const T &val) {
#ifndef DISABLE_DEBUG
  debug_print->print(val);
#endif
}

/**
 * @brief 打印单个值并换行
 * @param val 要打印的值
 */
template <typename T>
void DebugPrintln(const T &val) {
#ifndef DISABLE_DEBUG
  debug_print->println(val);
#endif
}

/**
 * @brief 打印消息和值
 * @param msg 消息
 * @param val 值
 */
template <typename T, typename U>
void DebugPrint(const T &msg, const U &val) {
#ifndef DISABLE_DEBUG
  debug_print->print(msg);
  debug_print->print(val);
#endif
}

/**
 * @brief 打印消息和值并换行
 * @param msg 消息
 * @param val 值
 */
template <typename T, typename U>
void DebugPrintln(const T &msg, const U &val) {
#ifndef DISABLE_DEBUG
  debug_print->print(msg);
  debug_print->println(val);
#endif
}
}  // namespace hortor::utils