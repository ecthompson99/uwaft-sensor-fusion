#include "master_task.h"

#include <gtest/gtest.h>

TEST(DriveCtrlMsgCallback, validData) {
  ros::NodeHandle nh;
  MasterTask master_task_test(&nh);

  master_task::drive_ctrl_input_msg drive_ctrl_msg;
  drive_ctrl_msg.acc_enable = 1;
  drive_ctrl_msg.aeb_enable = 1;
  drive_ctrl_msg.lc_enable = 1;

  master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);

  master_task::can_comms_data_msg can_msg = master_task_test.get_can_comms_msg();

  ASSERT_EQ(drive_ctrl_msg.acc_enable, can_msg.acc_switch_status);
  ASSERT_EQ(drive_ctrl_msg.aeb_enable, can_msg.aeb_switch_status);
  ASSERT_EQ(drive_ctrl_msg.lc_enable, can_msg.lc_switch_status);
}

TEST(SensorDiagFlagMsgCallback, validData) {
  ros::NodeHandle nh;
  MasterTask master_task_test(&nh);

  master_task::can_comms_data_msg can_msg_expected;
  can_msg_expected.acc_valid = 1;
  can_msg_expected.aeb_valid = 2;
  can_msg_expected.lc_valid = 3;

  master_task::sensor_diagnostic_flag_msg sensor_msg;
  sensor_msg.radar_reliability[0] = 1;
  sensor_msg.radar_reliability[1] = 2;
  sensor_msg.radar_reliability[2] = 3;
  sensor_msg.radar_reliability[3] = 4;
  sensor_msg.radar_reliability[4] = 5;
  sensor_msg.radar_reliability[5] = 6;

  master_task_test.sensor_diag_flag_msg_callback(sensor_msg);

  master_task::can_comms_data_msg can_msg = master_task_test.get_can_comms_msg();

  ASSERT_EQ(can_msg_expected.acc_valid, can_msg.acc_valid);
  ASSERT_EQ(can_msg_expected.aeb_valid, can_msg.aeb_valid);
  ASSERT_EQ(can_msg_expected.lc_valid, can_msg.lc_valid);
}

TEST(SudoDriverInputMsgCallback, validData) {
  ros::NodeHandle nh;
  MasterTask master_task_test(&nh);

  master_task::sudo_driver_input_msg input_msg;
  input_msg.aeb_override = 1;
  input_msg.target_accel = 3;
  input_msg.wheel_angle = 21;

  master_task_test.sudo_driver_input_msg_callback(input_msg);

  master_task::can_comms_data_msg can_msg = master_task_test.get_can_comms_msg();

  ASSERT_EQ(input_msg.aeb_override, can_msg.aeb_override);
  ASSERT_EQ(input_msg.target_accel, can_msg.target_accel);
  ASSERT_EQ(input_msg.wheel_angle, can_msg.wheel_angle);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "master_task_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
