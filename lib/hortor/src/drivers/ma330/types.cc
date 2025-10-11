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

#include "types.h"

namespace hortor::drivers::MA330 {

using RegField8 = regmap::RegField8;
// 兼容GD32的编译环境，需要手动定义，否则会链接错误
constexpr RegField8 MA330Regs::kZ_L;
constexpr RegField8 MA330Regs::kZ_H;
constexpr RegField8 MA330Regs::kBCT;
constexpr RegField8 MA330Regs::kETX;
constexpr RegField8 MA330Regs::kETY;
constexpr RegField8 MA330Regs::kILIP;
constexpr RegField8 MA330Regs::kPPT_L;
constexpr RegField8 MA330Regs::kPPT_H;
constexpr RegField8 MA330Regs::kMGHT;
constexpr RegField8 MA330Regs::kMGLT;
constexpr RegField8 MA330Regs::kNPP;
constexpr RegField8 MA330Regs::kRD;
constexpr RegField8 MA330Regs::kFW;
constexpr RegField8 MA330Regs::kHYS;
constexpr RegField8 MA330Regs::kMGL_MGH;

}  // namespace hortor::drivers::MA330