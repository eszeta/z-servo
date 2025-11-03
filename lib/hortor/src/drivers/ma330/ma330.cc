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

#include "ma330.h"

#include "types.h"

namespace hortor::drivers::MA330 {

Error MA330::InitSPI(const Config &config) {
  CHECK(regmap_.Init(
      config.spi, config.cs_pin, SPISettings(1000000, MSBFIRST, SPI_MODE3)));
  servo::Encoder<MA330, kResolutionBits>::Init(config.direction,
                                               config.homing_offset);
  return Error::kOk;
}

Error MA330::GetRawImpl(uint16_t &out_raw) {
  CHECK(regmap_.ReadRaw(out_raw));
  return Error::kOk;
}

}  // namespace hortor::drivers::MA330
