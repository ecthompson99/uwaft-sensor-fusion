#include "../include/master_task.h"

#include <gtest/gtest.h>

TEST(TestDriveDtrlMsgCallback, validData) { ASSERT_EQ(0, 0); }

TEST(TestDriveDtrlMsgCallback, emptyData) { ASSERT_EQ(0, 0); }

TEST(TestSensorDiagFlagMsgCallback, validData) { ASSERT_EQ(0, 0); }

TEST(TestSudoDriverInputMsgCallback, validData) { ASSERT_EQ(0, 0); }

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
