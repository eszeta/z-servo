// // Copyright 2025 ES_ZETA
// // SPDX-License-Identifier: Apache-2.0

// #include <Arduino.h>
// #include <SPI.h>
// #include <Wire.h>

// #include "core/servo.h"
// #include "debug_print.h"
// #include "drivers/MA330/MA330.h"
// #include "drivers/MP6515/MP6515.h"
// #include "drivers/generic_current/generic_current.h"
// #include "info_led.h"
// #include "protocol/slave.h"
// #include "protocol/port_table_accessor.h"
// #include "protocol/i2c_port_handler.h"
// #include "protocol/serial_port_handler.h"
// #include "math/math.h"

// static constexpr auto kInfoLedPin = PB1;
// HardwareSerial serial_debug(PB4, PB3, 0);
// HardwareSerial serial_inst(PA10, PA9, 1);
// SPIClass spi_sensor(DIGITAL_TO_PINNAME(PA8),
//                     DIGITAL_TO_PINNAME(PA9),
//                     DIGITAL_TO_PINNAME(PA10));

// hortor::InfoLED::InfoLED info_led;
// hortor::InstSerialPortHandler inst_port;
// hortor::PortTableRegMap inst_accessor;
// hortor::Slave inst;
// hortor::MP6515::MP6515 motor_driver;
// hortor::MA330::MA330 angle_sensor;
// hortor::generic_current::CurrentMirror current_sensor;
// hortor::Servo servo;

// // cppcheck-suppress unusedFunction
// void setup() {
//   serial_debug.begin(9600);
//   hortor::DebugEnable(&serial_debug);
//   hortor::DebugPrintln(F("setup"));

//   info_led.Init(kInfoLedPin, hortor::InfoLED::Mode::kOpenDrain);

//   motor_driver.Init(PA0, PA2, PA1, PA3);
//   spi_sensor.begin();
//   angle_sensor.InitSPI(&spi_sensor, PA10);
//   current_sensor.Init(PA3, 1000, 100);

//   servo.LinkDriver(&motor_driver);
//   servo.LinkAngleSensor(&angle_sensor);
//   servo.LinkCurrentSense(&current_sensor);
//   servo.Init();

//   inst_accessor.Init();
//   inst_port.Init(&serial_inst);

//   inst.LinkAccessor(&inst_accessor);
//   inst.LinkPort(&inst_port);
//   // inst.LinkServo(&servo);
//   inst.Init();

//   info_led.SetInfo(hortor::InfoLED::InfoType::kOk);
// }

// // cppcheck-suppress unusedFunction
// void loop() {
//   static auto last_time = micros() - 1;
//   const auto current_time = micros();
//   const auto dt = (current_time - last_time) * hortor::kMicroToSec;
//   info_led.Process(dt);
//   inst.Process(dt);
//   servo.Process(dt);
//   last_time = current_time;
// }