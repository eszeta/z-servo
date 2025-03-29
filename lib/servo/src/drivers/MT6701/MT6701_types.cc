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

#include "MT6701_types.h"

namespace hortor_servo {
namespace MT6701 {

using Regs = MT6701Regs;

// 兼容GD32的编译环境，需要手动定义，否则会链接错误
constexpr Register Regs::kANGLE_0;
constexpr Register Regs::kANGLE_6;
constexpr Register Regs::kUVM_MUX;
constexpr Register Regs::kABZ_MUX;
constexpr Register Regs::kDIR;
constexpr Register Regs::kUVM_RES_0;
constexpr Register Regs::kABZ_RES_8;
constexpr Register Regs::kABZ_RES_0;
constexpr Register Regs::kZERO_8;
constexpr Register Regs::kZERO_0;
constexpr Register Regs::kHYST_2;
constexpr Register Regs::kHYST_0;
constexpr Register Regs::kPULSE_WIDTH;
constexpr Register Regs::kPWM_FREQ;
constexpr Register Regs::kPWM_POL;
constexpr Register Regs::kOUT_MODE;
constexpr Register Regs::kA_STOP_8;
constexpr Register Regs::kA_START_8;
constexpr Register Regs::kA_START_0;
constexpr Register Regs::kA_STOP_0;

}  // namespace MT6701
}  // namespace hortor_servo
