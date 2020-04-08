#include <gtest/gtest.h>
#include "env_state.h"
//#include "object_state.h"

//TEST(FiltObjCallback, validLogic){
//  ros::NodeHandle env_state_node_handle;
//  EnvironmentState env_state_test(&env_state_node_handle);

//  sensor_fusion::object_output_msg output_msg_expected;
//  output_msg_expected.obj_id = 1;
//  output_msg_expected.obj_dx = 12;
//  output_msg_expected.obj_lane = 2;
//  output_msg_expected.obj_vx = 23;
//  output_msg_expected.obj_dy = 46;
//  output_msg_expected.obj_ax = 45;
//  output_msg_expected.obj_path = 1;
//  output_msg_expected.obj_vy = 67;
//  output_msg_expected.obj_timestamp = 134;

//  sensor_fusion::filtered_object_msg filtered_msg;
//  filtered_msg.obj_id = 1;
//  filtered_msg.obj_dx = 12;
//  filtered_msg.obj_lane = 2;
//  filtered_msg.obj_vx = 23;
//  filtered_msg.obj_dy = 46;
//  filtered_msg.obj_ax = 45;
//  filtered_msg.obj_path = 1;
//  filtered_msg.obj_vy = 67;
//  filtered_msg.obj_timestamp = 134;

//  env_state_test.filtered_object_callback(filtered_msg);

//  sensor_fusion::object_output_msg output_msg = env_state_test.get_object_output_msg();

//  ASSERT_EQ(output_msg_expected.obj_id, output_msg.obj_id);
//  ASSERT_EQ(output_msg_expected.obj_dx, output_msg.obj_dx);
//  ASSERT_EQ(output_msg_expected.obj_lane, output_msg.obj_lane);
//  ASSERT_EQ(output_msg_expected.obj_vx, output_msg.obj_vx);
//  ASSERT_EQ(output_msg_expected.obj_dy, output_msg.obj_dy);
//  ASSERT_EQ(output_msg_expected.obj_ax, output_msg.obj_ax);
//  ASSERT_EQ(output_msg_expected.obj_path, output_msg.obj_path);
//  ASSERT_EQ(output_msg_expected.obj_vy, output_msg.obj_vy);
//  ASSERT_EQ(output_msg_expected.obj_timestamp, output_msg.obj_timestamp);
//}

//TEST(FiltObjCallback, validCallback){
//  ros::NodeHandle env_state_node_handle;
//  EnvironmentState env_state_test(&env_state_node_handle);

//  ros::NodeHandle filtered_object_node_handle;
//  ros::Publisher pub = filtered_object_node_handle.advertise<sensor_fusion::filtered_object_msg>(
//    "kf_dummy_data", MESSAGE_BUFFER_SIZE);

//  sensor_fusion::object_output_msg output_msg_expected;
//  output_msg_expected.obj_id = 1;
//  output_msg_expected.obj_dx = 12;
//  output_msg_expected.obj_lane = 2;
//  output_msg_expected.obj_vx = 23;
//  output_msg_expected.obj_dy = 46;
//  output_msg_expected.obj_ax = 45;
//  output_msg_expected.obj_path = 1;
//  output_msg_expected.obj_vy = 67;
//  output_msg_expected.obj_timestamp = 134;

//  sensor_fusion::filtered_object_msg filtered_msg;
//  filtered_msg.obj_id = 1;
//  filtered_msg.obj_dx = 12;
//  filtered_msg.obj_lane = 2;
//  filtered_msg.obj_vx = 23;
//  filtered_msg.obj_dy = 46;
//  filtered_msg.obj_ax = 45;
//  filtered_msg.obj_path = 1;
//  filtered_msg.obj_vy = 67;
//  filtered_msg.obj_timestamp = 134;

//  pub.publish(filtered_msg);
//  ros::spinOnce();

//  sensor_fusion::object_output_msg output_msg = env_state_test.get_object_output_msg();

//  ASSERT_EQ(output_msg_expected.obj_id, output_msg.obj_id);
//  ASSERT_EQ(output_msg_expected.obj_dx, output_msg.obj_dx);
//  ASSERT_EQ(output_msg_expected.obj_lane, output_msg.obj_lane);
//  ASSERT_EQ(output_msg_expected.obj_vx, output_msg.obj_vx);
//  ASSERT_EQ(output_msg_expected.obj_dy, output_msg.obj_dy);
//  ASSERT_EQ(output_msg_expected.obj_ax, output_msg.obj_ax);
//  ASSERT_EQ(output_msg_expected.obj_path, output_msg.obj_path);
//  ASSERT_EQ(output_msg_expected.obj_vy, output_msg.obj_vy);
//  ASSERT_EQ(output_msg_expected.obj_timestamp, output_msg.obj_timestamp);
//}

TEST(AddObject, validLogic) {
  ros::NodeHandle env_state_node_handle;
  EnvironmentState env_state_test(&env_state_node_handle);

  ObjectState new_object_1(4, 10, 1, 4, 15, 12, 0, 6, 100);
  ObjectState new_object_2(5, 15, 2, 43, 6, 15, 1, 64, 200);
  ObjectState new_object_3(6, 40, 3, 29, 45, 21, 0, 26, 300);

  ASSERT_EQ(env_state_test.trackedObjects.size(), 0);
  env_state_test.add_object(new_object_1); // add object_1
  ASSERT_EQ(env_state_test.trackedObjects.size(), 1);
  ASSERT_EQ(env_state_test.trackedObjects[0].get_obj_id(), 4);

  env_state_test.add_object(new_object_2); // add object_2
  ASSERT_EQ(env_state_test.trackedObjects.size(), 2);
  ASSERT_EQ(env_state_test.trackedObjects[1].get_obj_id(), 5);

  env_state_test.add_object(new_object_3); // add object_3
  ASSERT_EQ(env_state_test.trackedObjects.size(), 3);
  ASSERT_EQ(env_state_test.trackedObjects[2].get_obj_id(), 6);
}

TEST(UpdateObject, validLogic) {
  ros::NodeHandle env_state_node_handle;
  EnvironmentState env_state_test(&env_state_node_handle);

  ObjectState new_object_1(4, 10, 1, 4, 15, 12, 0, 6, 100);
  ObjectState new_object_2(5, 15, 2, 43, 6, 15, 1, 64, 200);
  
  ObjectState tracked_object_1(1, 12, 2, 23, 46, 45, 1, 67, 134);
  ObjectState tracked_object_2(6, 40, 3, 29, 45, 21, 0, 26, 300);

  env_state_test.add_object(tracked_object_1); // add object
  env_state_test.add_object(tracked_object_2); // add object
  ASSERT_EQ(env_state_test.trackedObjects[0].get_obj_id(), 1);
  ASSERT_EQ(env_state_test.trackedObjects[1].get_obj_id(), 6);

  ASSERT_EQ(env_state_test.trackedObjects.size(), 2);
  env_state_test.update_object(new_object_1, 0); // replace tracked_object_1
  ASSERT_EQ(env_state_test.trackedObjects[0].get_obj_id(), 4);
  env_state_test.update_object(new_object_2, 1); // replace tracked_object_2
  ASSERT_EQ(env_state_test.trackedObjects[1].get_obj_id(), 5);
  ASSERT_EQ(env_state_test.trackedObjects.size(), 2);
}

TEST(CheckTimestamp, validLogic){
  ros::NodeHandle env_state_node_handle;
  EnvironmentState env_state_test(&env_state_node_handle);

  ObjectState new_object_1(4, 10, 1, 4, 15, 12, 0, 6, 35);
  ObjectState new_object_2(3, 56, 2, 3, 56, 14, 1, 67, 37);
  ObjectState new_object_3(7, 96, 1, 5, 76, 34, 0, 77, 100);
  
  ObjectState tracked_object_1(1, 12, 2, 23, 46, 45, 1, 67, 25);
  ObjectState tracked_object_2(6, 40, 0, 29, 45, 21, 0, 26, 30);
  ObjectState tracked_object_3(5, 10, 1, 27, 34, 87, 1, 90, 28);

  ASSERT_EQ(env_state_test.trackedObjects.size(), 0);
  env_state_test.add_object(tracked_object_1);
  ASSERT_EQ(env_state_test.trackedObjects.size(), 1);
  env_state_test.add_object(tracked_object_2);
  ASSERT_EQ(env_state_test.trackedObjects.size(), 2);
  env_state_test.add_object(tracked_object_3);
  ASSERT_EQ(env_state_test.trackedObjects.size(), 3);

  ASSERT_EQ(env_state_test.trackedObjects[0].get_obj_id(), 1);
  ASSERT_EQ(env_state_test.trackedObjects[1].get_obj_id(), 6);
  ASSERT_EQ(env_state_test.trackedObjects[2].get_obj_id(), 5);

  env_state_test.check_timestamp(new_object_1); // not erase any of the tracked objects
  ASSERT_EQ(env_state_test.trackedObjects.size(), 3);

  env_state_test.check_timestamp(new_object_2); // erase tracked_object_1 (37 - 25 = 12> 10)
	ASSERT_EQ(env_state_test.trackedObjects.size(), 2);
	ASSERT_EQ(env_state_test.trackedObjects[0].get_obj_id(), 6);

  env_state_test.check_timestamp(new_object_3); // erase tracked_object_2 & 3
  ASSERT_EQ(env_state_test.trackedObjects.size(), 0);
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
  ASSERT_EQ(env_state_test.trackedObjects.size(), 1); 
  ASSERT_EQ(env_state_test.trackedObjects[0].get_obj_id(), 1);

  env_state_test.update_env_state(new_object_2); //add new_object_2
  ASSERT_EQ(env_state_test.trackedObjects.size(), 2);
  ASSERT_EQ(env_state_test.trackedObjects[1].get_obj_id(), 2);
  
  env_state_test.update_env_state(new_object_3); //update first object
  ASSERT_EQ(env_state_test.trackedObjects.size(), 2);
  ASSERT_EQ(env_state_test.trackedObjects[0].get_obj_timestamp(), 300);
  
  env_state_test.update_env_state(new_object_4); //add new_object_4 (ID:3)
  ASSERT_EQ(env_state_test.trackedObjects.size(), 3);
  ASSERT_EQ(env_state_test.trackedObjects[2].get_obj_id(), 3);
  
  env_state_test.update_env_state(new_object_5); //add new_object_5 (ID:4)
  ASSERT_EQ(env_state_test.trackedObjects.size(), 4);
  ASSERT_EQ(env_state_test.trackedObjects[3].get_obj_id(), 4);
  
  env_state_test.update_env_state(new_object_6); //update second object
  ASSERT_EQ(env_state_test.trackedObjects.size(), 4);
  ASSERT_EQ(env_state_test.trackedObjects[1].get_obj_timestamp(), 600);

}

TEST(FindTargetObjects, validLogic){
  ros::NodeHandle env_state_node_handle;
  EnvironmentState env_state_test(&env_state_node_handle);

  ObjectState new_object_1(4, 10, 1, 4, 15, 12, 0, 6, 100);
  ObjectState new_object_2(5, 15, 2, 43, 6, 15, 1, 64, 200);
  ObjectState new_object_3(1, 24, 0, 67, 3, 623, 1, 45, 1200);
  ObjectState new_object_4(9, 2, 0, 67, 3, 623, 1, 45, 1200);

  ObjectState target_object_1(7, 89, 1, 29, 45, 21, 0, 26, 300);
  ObjectState target_object_2(5, 11, 2, 23, 46, 45, 1, 67, 134);
  ObjectState target_object_3(6, 12, 0, 78, 45, 21, 0, 26, 300);

  env_state_test.targetObjects[0] = target_object_3;// add objects in increasing order of lanes (0, 1, 2)
  ASSERT_EQ(env_state_test.targetObjects[0].get_obj_id(), 6);
	env_state_test.targetObjects[1] = target_object_1;
  ASSERT_EQ(env_state_test.targetObjects[1].get_obj_id(), 7);
	env_state_test.targetObjects[2] = target_object_2;
  ASSERT_EQ(env_state_test.targetObjects[2].get_obj_id(), 5);

  env_state_test.find_target_objects(new_object_1); // 10 dx < 89 dx (lane 1) -> update target object
  ASSERT_EQ(env_state_test.targetObjects[1].get_obj_id(), 4);

  env_state_test.find_target_objects(new_object_2); // 15 dx > 11 dx (lane 2) and ids are same -> update target object
  ASSERT_EQ(env_state_test.targetObjects[2].get_obj_id(), 5);
  env_state_test.find_target_objects(new_object_3); // 24 dx > 12 dx (lane 0) but ids are different -> don't update

	//ASSERT_EQ(env_state_test.targetObjects[0], NULL);

  env_state_test.find_target_objects(new_object_4); // 2 dx < 12 dx (lane 0) -> update target object
  ASSERT_EQ(env_state_test.targetObjects[0].get_obj_id(), 9);
}

//bool object_output_cb_called = false;
//void test_output_obj_cb(const sensor_fusion::object_output_msg& object_output_msg){object_output_cb_called = true;}

//TEST(PublishOutputObj, validLogic){
//  ros::NodeHandle env_state_node_handle;
//  EnvironmentState env_state_test(&env_state_node_handle);

//  ros::NodeHandle output_obj_node_handle;
//  ros::Subscriber sub = output_obj_node_handle.subscribe("object_output", MESSAGE_BUFFER_SIZE, &test_output_obj_cb);

//  env_state_test.publish_object_output();
//  ros::spinOnce();

//  ASSERT_EQ(object_output_cb_called, false);

//}


int main(int argc, char **argv) {
  ros::init(argc, argv, "env_state_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
