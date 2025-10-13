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

#ifndef EEPROM_DISABLE
#include <EEPROM.h>
#endif

#include "regmap.h"
namespace hortor::servo_slave {

Error RegMap::Init() {
  CHECK(regmap::RegMapMmio::Init(table_, sizeof(table_)));
  CHECK(LoadEeprom());
  return Error::kOk;
}
Error RegMap::ServoIdImpl(uint8_t& id) { return GetId(id); }
Error RegMap::StatusImpl(uint8_t& status) { return Error::kOk; }
Error RegMap::ReturnLevelImpl(uint8_t& return_level) {
  return GetStatusReturnLevel(return_level);
}
Error RegMap::RecoveryEepromImpl() { return Error::kOk; }
Error RegMap::LoadEepromImpl() {
  // #ifndef EEPROM_DISABLE
  //   int pos = 0;
  //   for (uint8_t address = RegsBlocks::kEeprom.begin;
  //        address < RegsBlocks::kEeprom.end;
  //        address++) {
  //     regs_[address] = EEPROM.read(pos++);
  //   }
  //   for (uint8_t address = RegsBlocks::kInternalEeprom.begin;
  //        address < RegsBlocks::kInternalEeprom.end;
  //        address++) {
  //     regs_[address] = EEPROM.read(pos++);
  //   }
  // #endif
  return Error::kOk;
}
Error RegMap::StoreEepromImpl() {
  // #ifndef EEPROM_DISABLE
  //   int pos = 0;
  //   for (uint8_t address = RegsBlocks::kEeprom.begin;
  //        address < RegsBlocks::kEeprom.end;
  //        address++) {
  //     EEPROM.update(pos++, regs_[address]);
  //   }
  //   for (uint8_t address = RegsBlocks::kInternalEeprom.begin;
  //        address < RegsBlocks::kInternalEeprom.end;
  //        address++) {
  //     EEPROM.update(pos++, regs_[address]);
  //   }
  // #endif
  return Error::kOk;
}
Error RegMap::StoreEepromImpl(const uint8_t address, const uint8_t size) {
  return Error::kOk;
}
}  // namespace hortor::servo_slave
