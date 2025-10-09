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

#include "MA330.h"

#include "types.h"

namespace hortor_servo {
namespace MA330 {

MA330::MA330() : Encoder(14) {}

Error MA330::InitSPI(SPIClass *spi, const uint8_t cs_pin) {
  CHECK(spi_transport_.Init(
      spi, cs_pin, SPISettings(1000000, MSBFIRST, SPI_MODE3)));
  CHECK(spi_transport_.LinkAccessor(accessor_));
  CHECK(accessor_.Init());
  hortor_servo::Encoder::Init();
  return Error::kOk;
}

Error MA330::GetRaw(uint16_t &out_raw) {
  CHECK(accessor_.ReadRaw(out_raw));
  return Error::kOk;
}
}  // namespace MA330
}  // namespace hortor_servo
