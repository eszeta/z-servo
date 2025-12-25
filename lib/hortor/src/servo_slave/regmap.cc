// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

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
Error RegMap::RecoveryEeprom() { return Error::kOk; }
Error RegMap::LoadEeprom() {
#ifndef EEPROM_DISABLE
  int pos = 0;
  for (uint8_t address = TableBlocks::kEeprom.begin;
       address < TableBlocks::kEeprom.end;
       address++) {
    regs_[address] = EEPROM.read(pos++);
  }
#endif
  return Error::kOk;
}

Error RegMap::StoreEeprom() {
#ifndef EEPROM_DISABLE
  int pos = 0;
  for (uint8_t address = TableBlocks::kEeprom.begin;
       address < TableBlocks::kEeprom.end;
       address++) {
    EEPROM.update(pos++, regs_[address]);
  }
#endif
  return Error::kOk;
}
}  // namespace hortor::servo_slave
