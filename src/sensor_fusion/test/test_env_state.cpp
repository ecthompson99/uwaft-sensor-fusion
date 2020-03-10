#include "env_state.cpp"

#include <gtest/gtesh.h>

TEST()

int main(int argc, char **argv) {
  ros::init(argc, argv, "env_state_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
