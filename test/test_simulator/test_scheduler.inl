namespace SchedulerTest {

void test_empty(void) {
  int PROGMEM a = 1;
}

void run_tests(void) {
  RUN_TEST(SchedulerTest::test_empty);
}
}  // namespace SchedulerTest