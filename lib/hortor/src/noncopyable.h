// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

namespace hortor {
class Noncopyable {
 private:
  explicit Noncopyable(Noncopyable const&)   = delete;
  Noncopyable& operator=(Noncopyable const&) = delete;

 public:
  Noncopyable()  = default;
  ~Noncopyable() = default;
};
}  // namespace hortor