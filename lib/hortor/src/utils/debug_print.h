// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

namespace hortor::utils {
#ifndef DISABLE_DEBUG
extern Print* debug_print;
#endif

void DebugEnable(Print* _debugPrint);

template <typename T>
void DebugPrint(const T& val);
template <typename T>
void DebugPrintln(const T& val);
template <typename T, typename U>
void DebugPrint(const T& msg, const U& val);
template <typename T, typename U>
void DebugPrintln(const T& msg, const U& val);
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

template <typename T>
void DebugPrint(const T& val) {
#ifndef DISABLE_DEBUG
  debug_print->print(val);
#endif
}

template <typename T>
void DebugPrintln(const T& val) {
#ifndef DISABLE_DEBUG
  debug_print->println(val);
#endif
}

template <typename T, typename U>
void DebugPrint(const T& msg, const U& val) {
#ifndef DISABLE_DEBUG
  debug_print->print(msg);
  debug_print->print(val);
#endif
}

template <typename T, typename U>
void DebugPrintln(const T& msg, const U& val) {
#ifndef DISABLE_DEBUG
  debug_print->print(msg);
  debug_print->println(val);
#endif
}

}  // namespace hortor::utils