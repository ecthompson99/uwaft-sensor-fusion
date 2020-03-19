#include <gtest/gtest.h>
#include "sensor_fusion.h"

TEST(RawSensorObjCallback, validLogic) {
  ros::NodeHandle sensor_fusion_node_handle;
  SensorFusion sensor_fusion_test(&sensor_fusion_node_handle);

  sensor_fusion::fused_object_data_msg fused_obj_expected;
  fused_obj_expected.accel_x = 1;
  fused_obj_expected.vel_x = 2;
  fused_obj_expected.pos_x = 3;
  fused_obj_expected.pos_y = 4;

  sensor_fusion::raw_sensor_object_data_msg raw_msg;
  raw_msg.radar_num = 5;
  raw_msg.num_objects = 3;
  raw_msg.accel_x = {1, 2, 3};
  raw_msg.vel_x = {1, 2, 3};
  raw_msg.pos_x = {1, 2, 3};
  raw_msg.pos_y = {1, 2, 3};
  raw_msg.exist_prob = {1, 2, 3};
  raw_msg.valid = {1, 1, 1};

  sensor_fusion_test.raw_sensor_obj_callback(raw_msg);

  sensor_fusion::fused_object_data_msg fused_msg = sensor_fusion_test.get_fused_data_msg();

  ASSERT_EQ(fused_obj_expected.accel_x, fused_msg.accel_x);
  ASSERT_EQ(fused_obj_expected.vel_x, fused_msg.vel_x);
  ASSERT_EQ(fused_obj_expected.pos_x, fused_msg.pos_x);
  ASSERT_EQ(fused_obj_expected.pos_y, fused_msg.pos_y);
}

TEST(RawSensorObjCallback, validCallback) {
  ros::NodeHandle sensor_fusion_node_handle;
  SensorFusion sensor_fusion_test(&sensor_fusion_node_handle);

  ros::NodeHandle raw_object_node_handle;
  ros::Publisher pub = raw_object_node_handle.advertise<sensor_fusion::raw_sensor_object_data_msg>(
      "raw_sensor_object_data", MESSAGE_BUFFER_SIZE);

  sensor_fusion::fused_object_data_msg fused_obj_expected;
  fused_obj_expected.accel_x = 1;
  fused_obj_expected.vel_x = 2;
  fused_obj_expected.pos_x = 3;
  fused_obj_expected.pos_y = 4;

  sensor_fusion::raw_sensor_object_data_msg raw_msg;
  raw_msg.radar_num = 5;
  raw_msg.num_objects = 3;
  raw_msg.accel_x = {1, 2, 3};
  raw_msg.vel_x = {1, 2, 3};
  raw_msg.pos_x = {1, 2, 3};
  raw_msg.pos_y = {1, 2, 3};
  raw_msg.exist_prob = {1, 2, 3};
  raw_msg.valid = {1, 1, 1};

  pub.publish(raw_msg);
  ros::spinOnce();

  sensor_fusion::fused_object_data_msg fused_msg = sensor_fusion_test.get_fused_data_msg();

  ASSERT_EQ(fused_obj_expected.accel_x, fused_msg.accel_x);
  ASSERT_EQ(fused_obj_expected.vel_x, fused_msg.vel_x);
  ASSERT_EQ(fused_obj_expected.pos_x, fused_msg.pos_x);
  ASSERT_EQ(fused_obj_expected.pos_y, fused_msg.pos_y);
}

TEST(SensorDiagFlagMsgCallback, validLogic) {
  ros::NodeHandle sensor_fusion_node_handle;
  SensorFusion sensor_fusion_test(&sensor_fusion_node_handle);

  sensor_fusion::fused_object_data_msg fused_obj_expected;
  fused_obj_expected.accel_x = 1;
  fused_obj_expected.vel_x = 2;
  fused_obj_expected.pos_x = 3;
  fused_obj_expected.pos_y = 4;

  sensor_fusion::sensor_diagnostic_flag_msg sensor_msg;
  sensor_msg.radar_reliability[0] = 1;
  sensor_msg.radar_reliability[1] = 2;
  sensor_msg.radar_reliability[2] = 3;
  sensor_msg.radar_reliability[3] = 4;
  sensor_msg.radar_reliability[4] = 5;
  sensor_msg.radar_reliability[5] = 6;

  sensor_fusion_test.sensor_diag_flag_msg_callback(sensor_msg);

  sensor_fusion::fused_object_data_msg fused_msg = sensor_fusion_test.get_fused_data_msg();

  ASSERT_EQ(fused_obj_expected.accel_x, fused_msg.accel_x);
  ASSERT_EQ(fused_obj_expected.vel_x, fused_msg.vel_x);
  ASSERT_EQ(fused_obj_expected.pos_x, fused_msg.pos_x);
  ASSERT_EQ(fused_obj_expected.pos_y, fused_msg.pos_y);
}

TEST(SensorDiagFlagMsgCallback, validCallback) {
  ros::NodeHandle sensor_fusion_node_handle;
  SensorFusion sensor_fusion_test(&sensor_fusion_node_handle);

  ros::NodeHandle diagnostic_flags_node_handle;
  ros::Publisher pub = diagnostic_flags_node_handle.advertise<sensor_fusion::sensor_diagnostic_flag_msg>(
      "sensor_diagnostic_flag", MESSAGE_BUFFER_SIZE);

  sensor_fusion::fused_object_data_msg fused_obj_expected;
  fused_obj_expected.accel_x = 1;
  fused_obj_expected.vel_x = 2;
  fused_obj_expected.pos_x = 3;
  fused_obj_expected.pos_y = 4;

  sensor_fusion::sensor_diagnostic_flag_msg sensor_msg;
  sensor_msg.radar_reliability[0] = 1;
  sensor_msg.radar_reliability[1] = 2;
  sensor_msg.radar_reliability[2] = 3;
  sensor_msg.radar_reliability[3] = 4;
  sensor_msg.radar_reliability[4] = 5;
  sensor_msg.radar_reliability[5] = 6;

  pub.publish(sensor_msg);
  ros::spinOnce();

  sensor_fusion::fused_object_data_msg fused_msg = sensor_fusion_test.get_fused_data_msg();

  ASSERT_EQ(fused_obj_expected.accel_x, fused_msg.accel_x);
  ASSERT_EQ(fused_obj_expected.vel_x, fused_msg.vel_x);
  ASSERT_EQ(fused_obj_expected.pos_x, fused_msg.pos_x);
  ASSERT_EQ(fused_obj_expected.pos_y, fused_msg.pos_y);
}

bool fused_obj_cb_called = false;
void test_fused_obj_cb(const sensor_fusion::fused_object_data_msg& fused_data_msg) { fused_obj_cb_called = true; }

TEST(PublishFusedData, validLogic) {
  ros::NodeHandle sensor_fusion_node_handle;
  SensorFusion sensor_fusion_test(&sensor_fusion_node_handle);

  ros::NodeHandle fused_data_node_handle;
  ros::Subscriber sub = fused_data_node_handle.subscribe("fused_data", MESSAGE_BUFFER_SIZE, &test_fused_obj_cb);

  sensor_fusion_test.publish_fused_data();
  ros::spinOnce();

  ASSERT_EQ(fused_obj_cb_called, false);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensor_fusion_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
