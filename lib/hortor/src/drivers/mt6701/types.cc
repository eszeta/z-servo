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

namespace hortor::drivers::MT6701 {
using RegField8 = regmap::RegField8;
// 兼容GD32的编译环境，需要手动定义，否则会链接错误
constexpr RegField8 MT6701Regs::kANGLE_6;
constexpr RegField8 MT6701Regs::kANGLE_0;
constexpr RegField8 MT6701Regs::kUVM_MUX;
constexpr RegField8 MT6701Regs::kABZ_MUX;
constexpr RegField8 MT6701Regs::kDIR;
constexpr RegField8 MT6701Regs::kUVM_RES_0;
constexpr RegField8 MT6701Regs::kABZ_RES_8;
constexpr RegField8 MT6701Regs::kABZ_RES_0;
constexpr RegField8 MT6701Regs::kZERO_8;
constexpr RegField8 MT6701Regs::kZERO_0;
constexpr RegField8 MT6701Regs::kHYST_2;
constexpr RegField8 MT6701Regs::kHYST_0;
constexpr RegField8 MT6701Regs::kPULSE_WIDTH;
constexpr RegField8 MT6701Regs::kPWM_FREQ;
constexpr RegField8 MT6701Regs::kPWM_POL;
constexpr RegField8 MT6701Regs::kOUT_MODE;
constexpr RegField8 MT6701Regs::kA_STOP_8;
constexpr RegField8 MT6701Regs::kA_START_8;
constexpr RegField8 MT6701Regs::kA_START_0;
constexpr RegField8 MT6701Regs::kA_STOP_0;

}  // namespace hortor::drivers::MT6701
