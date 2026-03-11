// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include <Arduino.h>
#include <unity.h>

#include <chrono>
#include <thread>

#include "test_bit_utils.h"
#include "test_lowpass_filter.h"
#include "test_math.h"
#include "test_pid.h"
#include "test_profile.h"
#include "test_reg_field.h"
#include "test_scheduler.h"
#include "test_timeout_limiter.h"

using namespace fakeit;

#define RUN_TEST_GROUP(TEST)                                                           \
  if (!std::getenv("TEST_GROUP") || (strcmp(#TEST, std::getenv("TEST_GROUP")) == 0)) { \
    TEST::run_tests();                                                                 \
  }

static unsigned long RealMicros() {
  using namespace std::chrono;
  static const auto start   = steady_clock::now();
  const auto        elapsed = duration_cast<microseconds>(steady_clock::now() - start).count();
  return static_cast<unsigned long>(elapsed & 0xFFFFFFFFUL);
}

void setUp(void) {
  ArduinoFakeReset();
  When(Method(ArduinoFake(Function), micros)).AlwaysDo([]() { return RealMicros(); });
  When(Method(ArduinoFake(Function), delayMicroseconds)).AlwaysDo([](unsigned int us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
  });
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST_GROUP(BitUtilsTest);
  RUN_TEST_GROUP(LowPassFilterTest);
  RUN_TEST_GROUP(MathTest);
  RUN_TEST_GROUP(PidTest);
  RUN_TEST_GROUP(ProfileTest);
  RUN_TEST_GROUP(RegFieldTest);
  RUN_TEST_GROUP(SchedulerTest);
  RUN_TEST_GROUP(TimeoutLimiterTest);
  return UNITY_END();
}
