// // Copyright 2025 ES_ZETA
// // SPDX-License-Identifier: Apache-2.0

// #include <Arduino.h>
// #include <Wire.h>
// #include <servo/types.h>
// #include <debug_print.h>
// #include <info_led.h>
// #include <protocol/serial_port_handler.h>
// #include <math/math.h>

// static constexpr auto kInfoLedPin = PB1;
// static constexpr auto kTargetLoopRateHz = 500;  // 目标帧率500Hz
// static constexpr auto kTargetLoopPeriodUs =
//     1000000 / kTargetLoopRateHz;  // 目标周期(微秒)

// hortor::InfoLED::InfoLED info_led{};
// HardwareSerial serial(PB4, PB3);
// TwoWire wire(PA8, PA9);
// hortor::InstSerialPortHandler inst_port{};
// hortor::InstProtocol inst_protocol{};
// hortor::InstPacket inst_packet{};

// hortor::Error Execute(hortor::InstPacket *packet);
// // cppcheck-suppress unusedFunction
// void setup() {
//   info_led.Init(kInfoLedPin, hortor::InfoLED::Mode::kOpenDrain);
//   info_led.SetInfo(hortor::InfoLED::InfoType::kOk);
//   serial.begin(115200);
//   wire.begin();
//   inst_port.Init(&serial);
// }

// // cppcheck-suppress unusedFunction
// void loop() {
//   static auto last_time = micros() - kTargetLoopPeriodUs;
//   const auto current_time = micros();
//   const auto dt = (current_time - last_time) * hortor::kMicroToSec;
//   last_time = current_time;

//   info_led.Process(dt);
//   bool is_complete = false;
//   inst_port.Process(inst_protocol, dt, inst_packet, is_complete);
//   if (is_complete) {
//     Execute(&inst_packet);
//   }

//   // 固定帧率控制
//   const auto elapsed_time = micros() - last_time;
//   if (elapsed_time < kTargetLoopPeriodUs) {
//     delayMicroseconds(kTargetLoopPeriodUs - elapsed_time);
//   }
// }

// hortor::Error Execute(hortor::InstPacket *packet) {
//   wire.beginTransmission(0x00);
//   wire.write(packet->buffer, packet->GetBufferSize());
//   wire.endTransmission(false);
//   wire.requestFrom(0x00, 128, 1);
//   while (wire.available()) {
//     uint8_t data = wire.read();
//     serial.write(data);
//   }
//   return hortor::Error::kOk;
// }