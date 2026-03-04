// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

namespace hortor {
class Noncopyable;
}

class hortor::Noncopyable {
 private:
  Noncopyable(Noncopyable const&) = delete;
  Noncopyable& operator=(Noncopyable const&) = delete;

 protected:
  Noncopyable() {}
  ~Noncopyable() {}
};
