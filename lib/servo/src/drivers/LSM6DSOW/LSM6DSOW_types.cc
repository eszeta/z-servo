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
#include "LSM6DSOW_types.h"

namespace hortor_servo {
namespace LSM6DSOW {

using Regs = LSM6DSOWRegs;
// 兼容GD32的编译环境，需要手动定义，否则会链接错误
constexpr Register Regs::kWHO_AM_I;
constexpr Register Regs::kCTRL1_XL;
constexpr Register Regs::kODR_XL;
constexpr Register Regs::kFS_XL;
constexpr Register Regs::kLPF2_XL_EN;
constexpr Register Regs::kCTRL2_G;
constexpr Register Regs::kODR_G;
constexpr Register Regs::kFS_G;
constexpr Register Regs::kFS_125;
constexpr Register Regs::kCTRL7_G;
constexpr Register Regs::kG_HM_MODE;
constexpr Register Regs::kHP_EN_G;
constexpr Register Regs::kHPM_G;
constexpr Register Regs::kUSR_OFF_ON_OUT;
constexpr Register Regs::kCTRL8_XL;
constexpr Register Regs::kHPCF_XL;
constexpr Register Regs::kHP_REF_MODE_XL;
constexpr Register Regs::kFASTSETTL_MODE_XL;
constexpr Register Regs::kHP_SLOPE_XL_EN;
constexpr Register Regs::kLOW_PASS_ON_6D;
constexpr Register Regs::kSTATUS_REG;
constexpr Register Regs::kTDA;
constexpr Register Regs::kGDA;
constexpr Register Regs::kXLDA;
constexpr Register Regs::kOUT_TEMP;
constexpr Register Regs::kOUT_TEMP_L;
constexpr Register Regs::kOUT_TEMP_H;
constexpr Register Regs::kOUTX_G;
constexpr Register Regs::kOUTX_L_G;
constexpr Register Regs::kOUTX_H_G;
constexpr Register Regs::kOUTY_G;
constexpr Register Regs::kOUTY_L_G;
constexpr Register Regs::kOUTY_H_G;
constexpr Register Regs::kOUTZ_G;
constexpr Register Regs::kOUTZ_L_G;
constexpr Register Regs::kOUTZ_H_G;
constexpr Register Regs::kOUTX_A;
constexpr Register Regs::kOUTX_L_A;
constexpr Register Regs::kOUTX_H_A;
constexpr Register Regs::kOUTY_A;
constexpr Register Regs::kOUTY_L_A;
constexpr Register Regs::kOUTY_H_A;
constexpr Register Regs::kOUTZ_A;
constexpr Register Regs::kOUTZ_L_A;
constexpr Register Regs::kOUTZ_H_A;

}  // namespace LSM6DSOW
}  // namespace hortor_servo