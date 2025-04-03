#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "debug_print.h"
#include "drivers/DRV8231A/DRV8231A.h"
#include "drivers/MA330/MA330.h"
#include "drivers/MP6515/MP6515.h"
#include "drivers/MT6701/MT6701.h"
#include "drivers/generic_current/generic_current.h"
#include "info_led.h"
#include "inst/inst.h"
#include "inst/inst_accessor.h"
#include "inst/inst_i2c_transport.h"
#include "inst/inst_serial_transport.h"
#include "servo.h"

#ifdef ARDUINO_ARCH_STM32
#include <HardwareSerial.h>
#endif

#ifdef ARDUINO_ARCH_STM32
static constexpr auto kInfoLedPin = PA12;
TwoWire wire_sensor(PA8, PA9);
TwoWire wire_inst(PB7, PA15);
HardwareSerial serial_debug(PB4, PB3);
hortor_servo::DRV8231A::DRV8231A motor_driver;
hortor_servo::MT6701::MT6701 angle_sensor;
hortor_servo::generic_current::GenericCurrent current_sensor;
hortor_servo::InstI2cTransport inst_transport;
#elif ARDUINO_ARCH_GD32
static constexpr auto kInfoLedPin = PB1;
SPIClass spi_sensor(DIGITAL_TO_PINNAME(PA8),
                    DIGITAL_TO_PINNAME(PA9),
                    DIGITAL_TO_PINNAME(PA10));
HardwareSerial serial_debug(PB4, PB3, 0);
HardwareSerial serial_inst(PA10, PA9, 1);
hortor_servo::MP6515::MP6515 motor_driver;
hortor_servo::MA330::MA330 angle_sensor;
hortor_servo::generic_current::GenericCurrent current_sensor;
hortor_servo::InstSerialTransport inst_transport;
#endif

hortor_servo::InfoLED::InfoLED info_led;

hortor_servo::InstAccessor inst_accessor;
hortor_servo::Inst inst;

hortor_servo::Servo servo;

#ifdef ARDUINO_ARCH_STM32
void receiveEvent(int howMany) { inst_transport.OnReceive(howMany); }
void requestEvent() { inst_transport.OnRequest(); }
#endif

void setup() {
  serial_debug.begin(9600);
  hortor_servo::DebugEnable(&serial_debug);
  hortor_servo::DebugPrintln(F("setup"));

  info_led.Init(kInfoLedPin, hortor_servo::InfoLED::Mode::kOpenDrain);

#ifdef ARDUINO_ARCH_STM32
  motor_driver.Init(PA0, PA2);
  current_sensor.Init(PA3, 1000, 1500);
  wire_sensor.begin();
  angle_sensor.InitI2C(&wire_sensor);
#elif ARDUINO_ARCH_GD32
  motor_driver.Init(PA0, PA2, PA1, PA3);
  current_sensor.Init(PA3, 1000, 100);
  spi_sensor.begin();
  angle_sensor.InitSPI(&spi_sensor, PA10);
#endif

  servo.LinkDriver(&motor_driver);
  servo.LinkAngleSensor(&angle_sensor);
  servo.LinkCurrentSense(&current_sensor);
  servo.Init();

  inst_accessor.Init();

#ifdef ARDUINO_ARCH_STM32
  inst_transport.Init(&wire_inst);
#elif ARDUINO_ARCH_GD32
  inst_transport.Init(&serial_inst);
#endif

  inst.LinkAccessor(&inst_accessor);
  inst.LinkTransport(&inst_transport);
  inst.LinkServo(&servo);
  inst.Init();

  info_led.SetInfo(hortor_servo::InfoLED::InfoType::kOk);
}

void loop() {
  static uint32_t last_time = micros();
  const uint32_t current_time = micros();
  const uint32_t dt = current_time - last_time;
  info_led.Process(dt);
  inst.Process(dt);
  servo.Process(dt);
  last_time = current_time;
}
