// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/* 仿真器单元测试入口。使用 Unity 驱动各模块测试组，Arduino 假对象提供
 * micros/delay 时间语义；支持环境变量 TEST_GROUP 按组过滤（如 MotorTest）。 */

#include <chrono>
#include <thread>

#include <Arduino.h>
#include <unity.h>

#include "../test_unit/test_current.h"
#include "../test_unit/test_encoder.h"
#include "../test_unit/test_motor.h"
#include "../test_unit/test_plant.h"

using namespace fakeit;

#define RUN_TEST_GROUP(TEST)                                                           \
  if (!std::getenv("TEST_GROUP") || (strcmp(#TEST, std::getenv("TEST_GROUP")) == 0)) { \
    TEST::run_tests();                                                                 \
  }

// 使用 steady_clock 返回微秒计数，替代 Arduino micros()，保证测试时间单调递增。
static unsigned long RealMicros() {
  using namespace std::chrono;
  static const auto start   = steady_clock::now();
  const auto        elapsed = duration_cast<microseconds>(steady_clock::now() - start).count();
  return static_cast<unsigned long>(elapsed & 0xFFFFFFFFUL);
}

// 重置 Arduino 假对象并绑定 micros/delay，使测试不依赖真实硬件。
void setUp(void) {
  ArduinoFakeReset();
  When(Method(ArduinoFake(Function), micros)).AlwaysDo([]() { return RealMicros(); });
  When(Method(ArduinoFake(Function), delayMicroseconds)).AlwaysDo([](unsigned int us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
  });
  When(Method(ArduinoFake(Function), delay)).AlwaysDo([](unsigned int /*ms*/) {});
}

void tearDown(void) {}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST_GROUP(MotorTest);
  RUN_TEST_GROUP(EncoderTest);
  RUN_TEST_GROUP(CurrentTest);
  RUN_TEST_GROUP(PlantTest);
  return UNITY_END();
}
