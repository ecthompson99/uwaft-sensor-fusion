#include <gtest/gtest.h>
#include "env_state.h"

TEST(FiltObjCallback, validLogic){
  ros::NodeHandle env_state_node_handle;
  EnvironmentState env_state_test(&env_state_node_handle);

  sensor_fusion::object_output_msg output_msg_expected;
  output_msg_expected.obj_id = 1;
  output_msg_expected.obj_dx = 12;
  output_msg_expected.obj_lane = 2;
  output_msg_expected.obj_vx = 23;
  output_msg_expected.obj_dy = 46;
  output_msg_expected.obj_ax = 45;
  output_msg_expected.obj_path = 1;
  output_msg_expected.obj_vy = 67;
  output_msg_expected.obj_timestamp = 134;

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

  sensor_fusion::object_output_msg output_msg = env_state_test.get_object_output_msg();

  ASSERT_EQ(output_msg_expected.obj_id, output_msg.obj_id);
  ASSERT_EQ(output_msg_expected.obj_dx, output_msg.obj_dx);
  ASSERT_EQ(output_msg_expected.obj_lane, output_msg.obj_lane);
  ASSERT_EQ(output_msg_expected.obj_vx, output_msg.obj_vx);
  ASSERT_EQ(output_msg_expected.obj_dy, output_msg.obj_dy);
  ASSERT_EQ(output_msg_expected.obj_ax, output_msg.obj_ax);
  ASSERT_EQ(output_msg_expected.obj_path, output_msg.obj_path);
  ASSERT_EQ(output_msg_expected.obj_vy, output_msg.obj_vy);
  ASSERT_EQ(output_msg_expected.obj_timestamp, output_msg.obj_timestamp);
}

TEST(FiltObjCallback, validCallback){}

TEST(AddObject, validLogic) {
  ros::NodeHandle env_state_node_handle;
  EnvironmentState env_state_test(&env_state_node_handle);

  ObjectState new_object_1;
  new_object_1.obj_id = 4;
  new_object_1.obj_dx = 10;
  new_object_1.obj_dy = 15;
  new_object_1.obj_lane = 1;
  new_object_1.obj_vx = 4;
  new_object_1.obj_vy = 6;
  new_object_1.obj_path = 0;
  new_object_1.obj_ax = 12;
  new_object_1.obj_timestamp = 100;

  ObjectState new_object_2;
  new_object_1.obj_id = 5;
  new_object_1.obj_dx = 15;
  new_object_1.obj_dy = 6;
  new_object_1.obj_lane = 2;
  new_object_1.obj_vx = 43;
  new_object_1.obj_vy = 64;
  new_object_1.obj_path = 1;
  new_object_1.obj_ax = 15;
  new_object_1.obj_timestamp = 200;

  ObjectState new_object_3;
  new_object_1.obj_id = 6;
  new_object_1.obj_dx = 40;
  new_object_1.obj_dy = 45;
  new_object_1.obj_lane = 3;
  new_object_1.obj_vx = 29;
  new_object_1.obj_vy = 26;
  new_object_1.obj_path = 0;
  new_object_1.obj_ax = 21;
  new_object_1.obj_timestamp = 300;

  std::vector<ObjectState> trackedObjects;
  ObjectState tracked_object_1;
  tracked_object_1.obj_id = 1;
  tracked_object_1.obj_dx = 12;
  tracked_object_1.obj_lane = 2;
  tracked_object_1.obj_vx = 23;
  tracked_object_1.obj_dy = 46;
  tracked_object_1.obj_ax = 45;
  tracked_object_1.obj_path = 1;
  tracked_object_1.obj_vy = 67;
  tracked_object_1.obj_timestamp = 134;

  EnvironmentState::trackedObjects.push_back(tracked_object_1);

  ASSERT_EQ(EnvironmentState::trackedObjects.size() = 1);
  EnvironmentState::env_state_test.add_object(new_object_1);
  ASSERT_EQ(EnvironmentState::trackedObjects.size(), 2);
  ASSERT_EQ(EnvironmentState::trackedObjects[1].obj_id, 4)
  EnvironmentState::env_state_test.add_object(new_object_2);
  ASSERT_EQ(EnvironmentState::trackedObjects.size(), 3);
  ASSERT_EQ(EnvironmentState::trackedObjects[1].obj_id, 5)
  EnvironmentState::env_state_test.add_object(new_object_3);
  ASSERT_EQ(EnvironmentState::trackedObjects.size(), 4);
  ASSERT_EQ(EnvironmentState::trackedObjects[1].obj_id, 6)
}

TEST(UpdateObject, validLogic) {
ros::NodeHandle env_state_node_handle;
  EnvironmentState env_state_test(&env_state_node_handle);

  ObjectState new_object_1;
  new_object_1.obj_id = 4;
  new_object_1.obj_dx = 10;
  new_object_1.obj_dy = 15;
  new_object_1.obj_lane = 1;
  new_object_1.obj_vx = 4;
  new_object_1.obj_vy = 6;
  new_object_1.obj_path = 0;
  new_object_1.obj_ax = 12;
  new_object_1.obj_timestamp = 100;

  ObjectState new_object_2;
  new_object_1.obj_id = 5;
  new_object_1.obj_dx = 15;
  new_object_1.obj_dy = 6;
  new_object_1.obj_lane = 2;
  new_object_1.obj_vx = 43;
  new_object_1.obj_vy = 64;
  new_object_1.obj_path = 1;
  new_object_1.obj_ax = 15;
  new_object_1.obj_timestamp = 200;

  std::vector<ObjectState> trackedObjects;
  ObjectState tracked_object_1;
  tracked_object_1.obj_id = 1;
  tracked_object_1.obj_dx = 12;
  tracked_object_1.obj_lane = 2;
  tracked_object_1.obj_vx = 23;
  tracked_object_1.obj_dy = 46;
  tracked_object_1.obj_ax = 45;
  tracked_object_1.obj_path = 1;
  tracked_object_1.obj_vy = 67;
  tracked_object_1.obj_timestamp = 134;

  ObjectState tracked_object_2;
  tracked_object_2.obj_id = 6;
  tracked_object_2.obj_dx = 40;
  tracked_object_2.obj_dy = 45;
  tracked_object_2.obj_lane = 3;
  tracked_object_2.obj_vx = 29;
  tracked_object_2.obj_vy = 26;
  tracked_object_2.obj_path = 0;
  tracked_object_2.obj_ax = 21;
  tracked_object_2.obj_timestamp = 300;

  EnvironmentState::trackedObjects.push_back(tracked_object_1);
  EnvironmentState::trackedObjects.push_back(tracked_object_2);

  ASSERT_EQ(EnvironmentState::trackedObjects.size(), 2);
  EnvironmentState::env_state_test.update_object(new_object_1, 0);
  ASSERT_EQ(EnvironmentState::trackedObjects[0].obj_id, 4)
  EnvironmentState::env_state_test.update_object(new_object_2, 1);
  ASSERT_EQ(EnvironmentState::trackedObjects[1].obj_id, 5)
}

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
  ros::spinOnce();

  ASSERT_EQ(object_output_cb_called, false);

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "env_state_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
