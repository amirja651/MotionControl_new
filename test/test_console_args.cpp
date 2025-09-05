#include <unity.h>

#include "UserConsole.hpp"

using cnc::console::UserConsole;

void test_bool_parser_valid() {
  bool v = false;
  TEST_ASSERT_TRUE(
      UserConsole::Instance().Begin({}));  // idempotent safe-ish for test
  TEST_ASSERT_TRUE(UserConsole::Instance().ParseBooleanToken("on", v));
  TEST_ASSERT_TRUE(v);
  TEST_ASSERT_TRUE(UserConsole::Instance().ParseBooleanToken("OFF", v));
  TEST_ASSERT_FALSE(v);
  TEST_ASSERT_TRUE(UserConsole::Instance().ParseBooleanToken("1", v));
  TEST_ASSERT_TRUE(v);
  TEST_ASSERT_TRUE(UserConsole::Instance().ParseBooleanToken("false", v));
  TEST_ASSERT_FALSE(v);
}

void test_bool_parser_invalid() {
  bool v = true;
  TEST_ASSERT_FALSE(UserConsole::Instance().ParseBooleanToken("maybe", v));
  TEST_ASSERT_TRUE(v);  // unchanged
}

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_bool_parser_valid);
  RUN_TEST(test_bool_parser_invalid);
  return UNITY_END();
}
