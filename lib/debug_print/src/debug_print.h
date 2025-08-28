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

namespace hortor_servo {
#ifndef DISABLE_DEBUG
/**
 * @brief 调试打印
 */
static Print *debug_print = nullptr;
#endif

/**
 * @brief 启用调试打印
 * @param _debugPrint 调试打印对象
 */
static void DebugEnable(Print *_debugPrint) {
#ifndef DISABLE_DEBUG
  debug_print = _debugPrint;
#endif
}

/**
 * @brief 打印单个值
 * @param val 要打印的值
 */
template <typename T>
static void DebugPrint(const T &val) {
#ifndef DISABLE_DEBUG
  debug_print->print(val);
#endif
}

/**
 * @brief 打印单个值并换行
 * @param val 要打印的值
 */
template <typename T>
static void DebugPrintln(const T &val) {
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
static void DebugPrint(const T &msg, const U &val) {
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
static void DebugPrintln(const T &msg, const U &val) {
#ifndef DISABLE_DEBUG
  debug_print->print(msg);
  debug_print->println(val);
#endif
}
}  // namespace hortor_servo