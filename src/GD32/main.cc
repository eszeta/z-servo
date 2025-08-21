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
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "core/servo.h"
#include "debug_print.h"
#include "drivers/MA330/MA330.h"
#include "drivers/MP6515/MP6515.h"
#include "drivers/generic_current/generic_current.h"
#include "info_led.h"
#include "inst/inst.h"
#include "inst/inst_accessor.h"
#include "inst/inst_serial_adapter.h"
#include "math/math.h"

static constexpr auto kInfoLedPin = PB1;
HardwareSerial serial_debug(PB4, PB3, 0);
HardwareSerial serial_inst(PA10, PA9, 1);
SPIClass spi_sensor(DIGITAL_TO_PINNAME(PA8),
                    DIGITAL_TO_PINNAME(PA9),
                    DIGITAL_TO_PINNAME(PA10));

hortor_servo::InfoLED::InfoLED info_led;
hortor_servo::InstSerialAdapter inst_adapter;
hortor_servo::InstAccessor inst_accessor;
hortor_servo::Inst inst;
hortor_servo::MP6515::MP6515 motor_driver;
hortor_servo::MA330::MA330 angle_sensor;
hortor_servo::generic_current::GenericCurrent current_sensor;
hortor_servo::Servo servo;

// cppcheck-suppress unusedFunction
void setup() {
  serial_debug.begin(9600);
  hortor_servo::DebugEnable(&serial_debug);
  hortor_servo::DebugPrintln(F("setup"));

  info_led.Init(kInfoLedPin, hortor_servo::InfoLED::Mode::kOpenDrain);

  motor_driver.Init(PA0, PA2, PA1, PA3);
  spi_sensor.begin();
  angle_sensor.InitSPI(&spi_sensor, PA10);
  current_sensor.Init(PA3, 1000, 100);

  servo.LinkDriver(&motor_driver);
  servo.LinkAngleSensor(&angle_sensor);
  servo.LinkCurrentSense(&current_sensor);
  servo.Init();

  inst_accessor.Init();
  inst_adapter.Init(&serial_inst);

  inst.LinkAccessor(&inst_accessor);
  inst.LinkAdapter(&inst_adapter);
  inst.LinkServo(&servo);
  inst.Init();

  info_led.SetInfo(hortor_servo::InfoLED::InfoType::kOk);
}

// cppcheck-suppress unusedFunction
void loop() {
  static auto last_time = micros() - 1;
  const auto current_time = micros();
  const auto dt = (current_time - last_time) * hortor_servo::kMicroToSec;
  info_led.Process(dt);
  inst.Process(dt);
  servo.Process(dt);
  last_time = current_time;
}