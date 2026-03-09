// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

namespace hortor::utils {
#ifndef DISABLE_DEBUG
extern Print* debug_print;
#endif

void DebugEnable(Print* _debugPrint);

template <typename... Args>
void DebugPrint(const Args&... args);

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
