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

#include "MA330_types.h"

namespace hortor_servo {
namespace MA330 {

using Regs = MA330Regs;

// 兼容GD32的编译环境，需要手动定义，否则会链接错误
constexpr Register Regs::kZ_L;
constexpr Register Regs::kZ_H;
constexpr Register Regs::kBCT;
constexpr Register Regs::kETX;
constexpr Register Regs::kETY;
constexpr Register Regs::kILIP;
constexpr Register Regs::kPPT_L;
constexpr Register Regs::kPPT_H;
constexpr Register Regs::kMGHT;
constexpr Register Regs::kMGLT;
constexpr Register Regs::kNPP;
constexpr Register Regs::kRD;
constexpr Register Regs::kFW;
constexpr Register Regs::kHYS;
constexpr Register Regs::kMGL_MGH;

}  // namespace MA330
}  // namespace hortor_servo