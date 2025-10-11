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

namespace hortor::drivers::LSM6DSOW {
using RegField8 = regmap::RegField8;
// 兼容GD32的编译环境，需要手动定义，否则会链接错误
constexpr RegField8 LSM6DSOWRegs::kWHO_AM_I;
constexpr RegField8 LSM6DSOWRegs::kCTRL1_XL;
constexpr RegField8 LSM6DSOWRegs::kODR_XL;
constexpr RegField8 LSM6DSOWRegs::kFS_XL;
constexpr RegField8 LSM6DSOWRegs::kLPF2_XL_EN;
constexpr RegField8 LSM6DSOWRegs::kCTRL2_G;
constexpr RegField8 LSM6DSOWRegs::kODR_G;
constexpr RegField8 LSM6DSOWRegs::kFS_G;
constexpr RegField8 LSM6DSOWRegs::kFS_125;
constexpr RegField8 LSM6DSOWRegs::kCTRL7_G;
constexpr RegField8 LSM6DSOWRegs::kG_HM_MODE;
constexpr RegField8 LSM6DSOWRegs::kHP_EN_G;
constexpr RegField8 LSM6DSOWRegs::kHPM_G;
constexpr RegField8 LSM6DSOWRegs::kUSR_OFF_ON_OUT;
constexpr RegField8 LSM6DSOWRegs::kCTRL8_XL;
constexpr RegField8 LSM6DSOWRegs::kHPCF_XL;
constexpr RegField8 LSM6DSOWRegs::kHP_REF_MODE_XL;
constexpr RegField8 LSM6DSOWRegs::kFASTSETTL_MODE_XL;
constexpr RegField8 LSM6DSOWRegs::kHP_SLOPE_XL_EN;
constexpr RegField8 LSM6DSOWRegs::kLOW_PASS_ON_6D;
constexpr RegField8 LSM6DSOWRegs::kSTATUS_REG;
constexpr RegField8 LSM6DSOWRegs::kTDA;
constexpr RegField8 LSM6DSOWRegs::kGDA;
constexpr RegField8 LSM6DSOWRegs::kXLDA;
constexpr RegField8 LSM6DSOWRegs::kOUT_TEMP;
constexpr RegField8 LSM6DSOWRegs::kOUT_TEMP_L;
constexpr RegField8 LSM6DSOWRegs::kOUT_TEMP_H;
constexpr RegField8 LSM6DSOWRegs::kOUTX_G;
constexpr RegField8 LSM6DSOWRegs::kOUTX_L_G;
constexpr RegField8 LSM6DSOWRegs::kOUTX_H_G;
constexpr RegField8 LSM6DSOWRegs::kOUTY_G;
constexpr RegField8 LSM6DSOWRegs::kOUTY_L_G;
constexpr RegField8 LSM6DSOWRegs::kOUTY_H_G;
constexpr RegField8 LSM6DSOWRegs::kOUTZ_G;
constexpr RegField8 LSM6DSOWRegs::kOUTZ_L_G;
constexpr RegField8 LSM6DSOWRegs::kOUTZ_H_G;
constexpr RegField8 LSM6DSOWRegs::kOUTX_A;
constexpr RegField8 LSM6DSOWRegs::kOUTX_L_A;
constexpr RegField8 LSM6DSOWRegs::kOUTX_H_A;
constexpr RegField8 LSM6DSOWRegs::kOUTY_A;
constexpr RegField8 LSM6DSOWRegs::kOUTY_L_A;
constexpr RegField8 LSM6DSOWRegs::kOUTY_H_A;
constexpr RegField8 LSM6DSOWRegs::kOUTZ_A;
constexpr RegField8 LSM6DSOWRegs::kOUTZ_L_A;
constexpr RegField8 LSM6DSOWRegs::kOUTZ_H_A;

}  // namespace hortor::drivers::LSM6DSOW