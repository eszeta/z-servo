// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include <Arduino.h>
#include <unity.h>

#include "test_scheduler.inl"

using namespace fakeit;

#define RUN_TEST_GROUP(TEST)                                                           \
  if (!std::getenv("TEST_GROUP") || (strcmp(#TEST, std::getenv("TEST_GROUP")) == 0)) { \
    TEST::run_tests();                                                                 \
  }

void setUp(void) {
  ArduinoFakeReset();
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST_GROUP(SchedulerTest);
  return UNITY_END();
}
