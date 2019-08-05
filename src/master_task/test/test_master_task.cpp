#include "helpers.h"

#include <gtest/gtest.h>

TEST(add_one, validData) {
  const int input = 1;
  const int output_expected = 4;
  ASSERT_EQ(add_one(input), output_expected);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
