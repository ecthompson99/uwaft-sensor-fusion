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
  
  //void set_constructor(uint8_t set_obj_id, double set_obj_dx, uint8_t set_obj_lane,
  //double set_obj_vx, double set_obj_dy, double set_obj_ax, bool set_obj_path, 
  //double set_obj_vy, double set_obj_timestamp){

  ObjectState new_object_1(4, 10, 1, 4, 15, 12, 0, 6, 100);
  ObjectState new_object_2(5, 15, 2, 43, 6, 15, 1, 64, 200);
  ObjectState new_object_3(6, 40, 3, 29, 45, 21, 0, 26, 300);

  std::vector<ObjectState> trackedObjects;
  
  ObjectState tracked_object_1(1, 12, 2, 23, 46, 45, 1, 67, 134);

  trackedObjects.push_back(tracked_object_1);

  ASSERT_EQ(trackedObjects.size(), 1);
  EnvironmentState::env_state_test.add_object(new_object_1);
  ASSERT_EQ(trackedObjects.size(), 2);
  ASSERT_EQ(trackedObjects[1].get_obj_id(), 4);
  trackedObjects.add_object(new_object_2);
  ASSERT_EQ(trackedObjects.size(), 3);
  ASSERT_EQ(trackedObjects[1].get_obj_id(), 5);
  trackedObjects.add_object(new_object_3);
  ASSERT_EQ(trackedObjects.size(), 4);
  ASSERT_EQ(trackedObjects[1].get_obj_id(), 6);
}

TEST(UpdateObject, validLogic) {
  ros::NodeHandle env_state_node_handle;
  EnvironmentState env_state_test(&env_state_node_handle);

  //void set_constructor(uint8_t set_obj_id, double set_obj_dx, uint8_t set_obj_lane,
  //double set_obj_vx, double set_obj_dy, double set_obj_ax, bool set_obj_path, 
  //double set_obj_vy, double set_obj_timestamp){

  ObjectState new_object_1(4, 10, 1, 4, 15, 12, 0, 6, 100);
  ObjectState new_object_2(5, 15, 2, 43, 6, 15, 1, 64, 200);

  std::vector<ObjectState> trackedObjects;
  
  ObjectState tracked_object_1(1, 12, 2, 23, 46, 45, 1, 67, 134);
  ObjectState tracked_object_2(6, 40, 3, 29, 45, 21, 0, 26, 300);

  trackedObjects.push_back(tracked_object_1);
  trackedObjects.push_back(tracked_object_2);
  ASSERT_EQ(trackedObjects[0].get_obj_id(), 1);
  ASSERT_EQ(trackedObjects[1].get_obj_id(), 6);

  ASSERT_EQ(trackedObjects.size(), 2);
  env_state_test.update_object(new_object_1, 0);
  ASSERT_EQ(trackedObjects[0].get_obj_id(), 4);
  env_state_test.update_object(new_object_2, 1);
  ASSERT_EQ(trackedObjects[1].get_obj_id(), 5);
}

TEST(CheckTimestamp, validLogic){
  ros::NodeHandle env_state_node_handle;
  EnvironmentState env_state_test(&env_state_node_handle);

  //void set_constructor(uint8_t set_obj_id, double set_obj_dx, uint8_t set_obj_lane,
  //double set_obj_vx, double set_obj_dy, double set_obj_ax, bool set_obj_path, 
  //double set_obj_vy, double set_obj_timestamp){
  
  ObjectState new_object_1(4, 10, 1, 4, 15, 12, 0, 6, 4000);
  ObjectState new_object_2(3, 56, 2, 3, 56, 14, 1, 67, 50000);

  std::vector<ObjectState> trackedObjects;
  
  ObjectState tracked_object_1(1, 12, 2, 23, 46, 45, 1, 67, 134);
  ObjectState tracked_object_2(6, 40, 0, 29, 45, 21, 0, 26, 1500);
  ObjectState tracked_object_2(5, 10, 1, 27, 34, 87, 1, 90, 3000);

  ASSERT_EQ(trackedObjects.size(), 0);
  trackedObjects.push_back(tracked_object_1);
  ASSERT_EQ(trackedObjects.size(), 1);
  trackedObjects.push_back(tracked_object_2);
  ASSERT_EQ(trackedObjects.size(), 2);
  trackedObjects.push_back(tracked_object_3);
  ASSERT_EQ(trackedObjects.size(), 3);

  ASSERT_EQ(trackedObjects[0].get_obj_id(), 1);
  ASSERT_EQ(trackedObjects[1].get_obj_id(), 6);
  ASSERT_EQ(trackedObjects[2].get_obj_id(), 5);

  env_state_test.check_timestamp(new_object_1);
  ASSERT_EQ(trackedObjects.size(), 3);

  env_state_test.check_timestamp(new_object_2);
  ASSERT_EQ(trackedObjects.size(), 0);

}
TEST(UpdateEnvState, validLogic) {
  ros::NodeHandle env_state_node_handle;
  EnvironmentState env_state_test(&env_state_node_handle);

  ObjectState new_object_1(1, 10, 1, 4, 15, 11, 0, 6, 100);
  ObjectState new_object_2(2, 15, 2, 43, 6, 15, 1, 64, 200);
  ObjectState new_object_3(1, 33, 1, 23, 9, 14, 0, 8, 300);
  ObjectState new_object_4(3, 45, 2, 3, 16, 35, 1, 5, 400);
  ObjectState new_object_5(4, 12, 1, 6, 18, 12, 0, 4, 500);
  ObjectState new_object_6(2, 11, 1, 19, 32, 5, 1, 6, 600);

  env_state_test.update_env_state(new_object_1); //add new_object_1
  ASSERT_EQ(EnvironmentState::trackedObjects.size(), 1); 
  ASSERT_EQ(EnvironmentState::trackedObjects[0].get_obj_id(), 1);

  env_state_test.update_env_state(new_object_2); //add new_object_2
  ASSERT_EQ(EnvironmentState::trackedObjects.size(), 2);
  ASSERT_EQ(EnvironmentState::trackedObjects[1].get_obj_id(), 1);
  
  env_state_test.update_env_state(new_object_3); //update first object
  ASSERT_EQ(EnvironmentState::trackedObjects.size(), 2);
  ASSERT_EQ(EnvironmentState::trackedObjects[0].get_obj_timestamp(), 300);
  
  env_state_test.update_env_state(new_object_4); //add new_object_4 (ID:3)
  ASSERT_EQ(EnvironmentState::trackedObjects.size(), 3);
  ASSERT_EQ(EnvironmentState::trackedObjects[2].get_obj_id(), 3);
  
  env_state_test.update_env_state(new_object_5); //add new_object_5 (ID:4)
  ASSERT_EQ(EnvironmentState::trackedObjects.size(), 4);
  ASSERT_EQ(EnvironmentState::trackedObjects[3].get_obj_id(), 4);
  
  env_state_test.update_env_state(new_object_6); //update second object
  ASSERT_EQ(EnvironmentState::trackedObjects.size(), 4);
  ASSERT_EQ(EnvironmentState::trackedObjects[1].get_obj_timestamp(), 600);

}

TEST(FindTargetObjects, validLogic){
  ros::NodeHandle env_state_node_handle;
  EnvironmentState env_state_test(&env_state_node_handle);
  
  ObjectState new_object_1(4, 10, 1, 4, 15, 12, 0, 6, 100);
  ObjectState new_object_2(5, 15, 2, 43, 6, 15, 1, 64, 200);
  ObjectState new_object_3(1, 24, 0, 67, 3, 623, 1, 45, 1200);

  std::vector<ObjectState> trackedObjects;
  ObjectState targetObjects[3] = {};

  ObjectState tracked_object_3(7, 89, 1, 29, 45, 21, 0, 26, 300);
  ObjectState tracked_object_1(5, 11, 2, 23, 46, 45, 1, 67, 134);
  ObjectState tracked_object_2(6, 12, 0, 78, 45, 21, 0, 26, 300);

  env_state_test.push_back(tracked_object_1);
  env_state_test.push_back(tracked_object_2);
  env_state_test.push_back(tracked_object_3);
  ASSERT_EQ(trackedObjects.size(), 3);
  
  ASSERT_EQ(targetObjects[0], 0);
  ASSERT_EQ(targetObjects[1], 0);
  ASSERT_EQ(targetObjects[2], 0);

  env_state_test.find_target_objects(new_object_1);
  ASSERT_EQ(targetObjects[1], new_object_1);
  env_state_test.find_target_objects(new_object_2);
  ASSERT_EQ(targetObjects[2], new_object_2);
  env_state_test.find_target_objects(new_object_3);
  ASSERT_EQ(targetObjects[0], 0);
}

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
