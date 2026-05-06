#include <Arduino.h>
#include <unity.h>

void test_placeholder_smoke() {
    TEST_ASSERT_TRUE(true);
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_placeholder_smoke);
    UNITY_END();
}

void loop() {}

