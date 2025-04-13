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

#include "register_spi_transport.h"

#include "../../servo_types.h"
#include "register_accessor.h"

namespace hortor_servo {
Error RegisterSpiTransport::Init(SPIClass *spi,
                                 const int cs_pin,
                                 const SPISettings &spi_settings) {
  if (!spi_) {
    return Error::kInvalidParameter;
  }
  spi_ = spi;
  cs_pin_ = cs_pin;
  spi_settings_ = spi_settings;
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);
  return Error::kOk;
}

Error RegisterSpiTransport::LinkAccessor(RegisterAccessor &accessor) {
  accessor.SetWrite([this](const uint8_t address, const uint8_t data) {
    return Write(address, data);
  });
  accessor.SetWriteMultiple(
      [this](const uint8_t address, const uint8_t *data, const size_t size) {
        return WriteMultiple(address, data, size);
      });
  accessor.SetRead([this](const uint8_t address, uint8_t *data) {
    return Read(address, data);
  });
  accessor.SetReadMultiple(
      [this](const uint8_t address, const size_t size, uint8_t *data) {
        return ReadMultiple(address, size, data);
      });
  return Error::kOk;
}

Error RegisterSpiTransport::Write(const uint8_t address, const uint8_t data) {
  if (!spi_) {
    return Error::kInvalidParameter;
  }
  spi_->beginTransaction(spi_settings_);
  if (cs_pin_ >= 0) digitalWrite(cs_pin_, LOW);
  spi_->transfer(address);
  spi_->transfer(data);
  if (cs_pin_ >= 0) digitalWrite(cs_pin_, HIGH);
  spi_->endTransaction();
  return Error::kOk;
}

Error RegisterSpiTransport::WriteMultiple(const uint8_t address,
                                          const uint8_t *data,
                                          const size_t size) {
  if (!spi_ || !data) {
    return Error::kInvalidParameter;
  }
  spi_->beginTransaction(spi_settings_);
  if (cs_pin_ >= 0) digitalWrite(cs_pin_, LOW);
  spi_->transfer(address);
  spi_->transfer(const_cast<uint8_t *>(data), nullptr, size);
  if (cs_pin_ >= 0) digitalWrite(cs_pin_, HIGH);
  spi_->endTransaction();
  return Error::kOk;
}

Error RegisterSpiTransport::Read(const uint8_t address, uint8_t *data) {
  if (!spi_) {
    return Error::kInvalidParameter;
  }
  spi_->beginTransaction(spi_settings_);
  if (cs_pin_ >= 0) digitalWrite(cs_pin_, LOW);
  spi_->transfer(0x80 | address);
  spi_->transfer(data, 1);
  if (cs_pin_ >= 0) digitalWrite(cs_pin_, HIGH);
  spi_->endTransaction();
  return Error::kOk;
}

Error RegisterSpiTransport::ReadMultiple(const uint8_t address,
                                         const size_t size,
                                         uint8_t *data) {
  if (!spi_) {
    return Error::kInvalidParameter;
  }
  spi_->beginTransaction(spi_settings_);
  if (cs_pin_ >= 0) digitalWrite(cs_pin_, LOW);
  spi_->transfer(0x80 | address);
  spi_->transfer(data, size);
  if (cs_pin_ >= 0) digitalWrite(cs_pin_, HIGH);
  spi_->endTransaction();
  return Error::kOk;
}
}  // namespace hortor_servo
