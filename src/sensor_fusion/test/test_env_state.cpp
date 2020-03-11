#include <gtest/gtesh.h>
#include "env_state.h"

TEST(FiltObjCallback, validLogic){
  ros::NodeHandle env_state_node_handle;
  EnvironmentState env_state_test(&env_state_node_handle);

  sensor_fusion::object_output_msg object_output_expected;
  object_output_expected.obj_id = 1;
  object_output_expected.obj_dx = 12;
  object_output_expected.obj_lane = 2;
  object_output_expected.obj_vx = 23;
  object_output_expected.obj_dy = 46;
  object_output_expected.obj_ax = 45;
  object_output_expected.obj_path = 1;
  object_output_expected.obj_vy = 67;
  object_output_expected.obj_timestamp = 134;
  object_output_expected.object_track_num = 1;

  sensor_fusion::filtered_object_msg filtered_msg;
  filtered_msg.obj_id = 1;
  filtered_msg.obj_dx = 12;
  filtered_msg.obj_lane = 2;
  filtered_msg.obj_vx = 23;
  filtered_msg.obj_dy = 46;
  filtered_msg.obj_ax = 45;
  filtered_msg.obj_path = 1;
  filtered_msg.obj_vy = 67;
  filtered_msg.obj_timestamp = 134;

  env_state_test.filtered_object_callback(filtered_msg);

  sensor_fusion::object_output_msg object_output = env_state_test.get_object_output_msg();

  ASSERT_EQ(object_output_expected.obj_id, object_output.obj_id);
  ASSERT_EQ(object_output_expected.obj_dx, object_output.obj_dx);
  ASSERT_EQ(object_output_expected.obj_lane, object_output.obj_lane);
  ASSERT_EQ(object_output_expected.obj_vx, object_output.obj_vx);
  ASSERT_EQ(object_output_expected.obj_dy, object_output.obj_dy);
  ASSERT_EQ(object_output_expected.obj_ax, object_output.obj_ax);
  ASSERT_EQ(object_output_expected.obj_path, object_output.obj_path);
  ASSERT_EQ(object_output_expected.obj_vy, object_output.obj_vy);
  ASSERT_EQ(object_output_expected.obj_timestamp, object_output.obj_timestamp);
  ASSERT_EQ(object_output_expected.object_track_num, object_output.object_track_num);


}

TEST(FiltObjCallback, validCallback){}

TEST(AddObject, validLogic){}
TEST(UpdateObject, validLogic){}
TEST(CheckTimestamp, validLogic){}
TEST(UpdateEnvState, validLogic){}
TEST(FindTargetObjects, validLogic){}

bool object_output_cb_called = false;
void test_output_obj_cb(const sensor_fusion::object_output_msg& object_output_msg){object_output_cb_called = true;}

TEST(PublishOutputObj, validLogic){
  ros::NodeHandle env_state_node_handle;
  EnvironmentState env_state_test(&env_state_node_handle);

  ros::NodeHandle output_obj_node_handle;
  ros::Subscriber sub = output_obj_node_handle.subscribe("object_output", MESSAGE_BUFFER_SIZE, &test_output_obj_cb);

  env_state_test.publish_object_output();
  ros::spinOnce(object_output_cb_called, false);

  ASSERT_EQ(test_output_obj_cb, false);

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "env_state_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
