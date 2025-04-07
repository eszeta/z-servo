#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "debug_print.h"
#include "drivers/DRV8231A/DRV8231A.h"
#include "drivers/MT6701/MT6701.h"
#include "drivers/generic_current/generic_current.h"
#include "info_led.h"
#include "inst/inst.h"
#include "inst/inst_accessor.h"
#include "inst/inst_i2c_transport.h"
#include "servo.h"

static constexpr auto kInfoLedPin = PA12;

HardwareSerial serial_debug(PB4, PB3);
TwoWire wire_sensor(PA8, PA9);
TwoWire wire_inst(PB7, PA15);

hortor_servo::InfoLED::InfoLED info_led;
hortor_servo::InstI2cTransport inst_transport;
hortor_servo::InstAccessor inst_accessor;
hortor_servo::Inst inst;
hortor_servo::DRV8231A::DRV8231A motor_driver;
hortor_servo::MT6701::MT6701 angle_sensor;
hortor_servo::generic_current::GenericCurrent current_sensor;
hortor_servo::Servo servo;

void receiveEvent(int howMany) { inst_transport.OnReceive(howMany); }
void requestEvent() { inst_transport.OnRequest(); }

void setup() {
  serial_debug.begin(9600);
  hortor_servo::DebugEnable(&serial_debug);
  hortor_servo::DebugPrintln(F("setup"));

  info_led.Init(kInfoLedPin, hortor_servo::InfoLED::Mode::kOpenDrain);

  motor_driver.Init(PA0, PA2);
  wire_sensor.begin();
  angle_sensor.InitI2C(&wire_sensor);
  current_sensor.Init(PA3, 1000, 1500);

  servo.LinkDriver(&motor_driver);
  servo.LinkAngleSensor(&angle_sensor);
  servo.LinkCurrentSense(&current_sensor);
  servo.Init();

  inst_accessor.Init();
  inst_transport.Init(&wire_inst);

  inst.LinkAccessor(&inst_accessor);
  inst.LinkTransport(&inst_transport);
  inst.LinkServo(&servo);
  inst.Init();

  info_led.SetInfo(hortor_servo::InfoLED::InfoType::kOk);
  servo.SetPower(-1.0f);
}

void loop() {
  static uint32_t last_time = micros();
  const uint32_t current_time = micros();
  const uint32_t dt = current_time - last_time;
  info_led.Process(dt);
  inst.Process(dt);
  servo.Process(dt);
  last_time = current_time;

  static auto print_time = 0;
  print_time += dt;
  if (print_time > 100000) {
    print_time = 0;
    hortor_servo::DebugPrintln(servo.GetPresentVelocity());
  }
}