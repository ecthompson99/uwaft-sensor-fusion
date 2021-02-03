#include <gtest/gtest.h>
#include "env_state.h"

TEST(FiltObjCallback, validLogic){
    ros::NodeHandle env_state_node_handle;
    EnvironmentState env_state_test(&env_state_node_handle);

    common::tracked_output_msg output_msg_expected;
    output_msg_expected.obj_id = 4;
    output_msg_expected.obj_dx = 2;
    output_msg_expected.obj_lane = 0;
    output_msg_expected.obj_vx = 2;
    output_msg_expected.obj_dy = 2;
    output_msg_expected.obj_ax = 2;
    output_msg_expected.obj_path = 0;
    output_msg_expected.obj_vy = 2;
    output_msg_expected.obj_timestamp = 2;

    common::filtered_object_msg filtered_msg;
    filtered_msg.obj_id = 4; // add as new object
    filtered_msg.obj_dx = 2;
    filtered_msg.obj_lane = 0;
    filtered_msg.obj_vx = 2;
    filtered_msg.obj_dy = 2;
    filtered_msg.obj_ax = 2;
    filtered_msg.obj_path = 0;
    filtered_msg.obj_vy = 2;
    filtered_msg.obj_timestamp = 2; // < 5 so keep

    env_state_test.global_clk = 1;
    env_state_test.first_timestamp = 0;
    
    env_state_test.filtered_object_callback(filtered_msg);

    common::tracked_output_msg output_msg = env_state_test.get_tracked_output_msg();

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

TEST(FiltObjCallback, validCallback){
    ros::NodeHandle env_state_node_handle;
    EnvironmentState env_state_test(&env_state_node_handle);

    ros::NodeHandle filtered_object_node_handle;
    ros::Publisher pub = filtered_object_node_handle.advertise<common::filtered_object_msg>(
    "filtered_obj", MESSAGE_BUFFER_SIZE);

    common::tracked_output_msg output_msg_expected;
    output_msg_expected.obj_id = 1;
    output_msg_expected.obj_dx = 2;
    output_msg_expected.obj_lane = 0;
    output_msg_expected.obj_vx = 2;
    output_msg_expected.obj_dy = 2;
    output_msg_expected.obj_ax = 2;
    output_msg_expected.obj_path = 0;
    output_msg_expected.obj_vy = 2;
    output_msg_expected.obj_timestamp = 2;


    pub.publish(output_msg_expected);
    ros::spinOnce();

    env_state_test.first_timestamp = 0;
    env_state_test.global_clk = 1;
    
    common::tracked_output_msg output_msg = env_state_test.get_tracked_output_msg();

    ASSERT_EQ(output_msg_expected.obj_id, output_msg.obj_id);
    ASSERT_EQ(output_msg_expected.obj_dx, output_msg.obj_dx);
    ASSERT_EQ(output_msg_expected.obj_lane, output_msg.obj_lane);
    ASSERT_EQ(output_msg_expected.obj_vx, output_msg.obj_vx);
    ASSERT_EQ(output_msg_expected.obj_dy, output_msg.obj_dy);
    ASSERT_EQ(output_msg_expected.obj_ax, output_msg.obj_ax);
    ASSERT_EQ(output_msg_expected.obj_path, output_msg.obj_path);
}

TEST(AddObject, validLogic) {
    ros::NodeHandle env_state_node_handle;
    EnvironmentState env_state_test(&env_state_node_handle);

    ObjectState new_object_1(4, 10, 1, 4, 15, 12, 0, 6, 100, 1);
    ObjectState new_object_2(5, 15, 2, 43, 6, 15, 1, 64, 200, 1);
    ObjectState new_object_3(6, 40, 3, 29, 45, 21, 0, 26, 300, 1);

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

    ObjectState new_object_1(4, 10, 1, 4, 15, 12, 0, 6, 100, 1);
    ObjectState new_object_2(5, 15, 2, 43, 6, 15, 1, 64, 200, 1);

    ObjectState tracked_object_1(1, 12, 2, 23, 46, 45, 1, 67, 134, 1);
    ObjectState tracked_object_2(6, 40, 3, 29, 45, 21, 0, 26, 300, 1);

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

    ObjectState new_object_1(4, 10, 1, 4, 15, 12, 0, 6, 26, 1);
    ObjectState new_object_2(3, 56, 2, 3, 56, 14, 1, 67, 31, 1);
    ObjectState new_object_3(7, 96, 1, 5, 76, 34, 0, 77, 100, 1);

    ObjectState tracked_object_1(1, 12, 2, 23, 46, 45, 1, 67, 25, 1);
    ObjectState tracked_object_2(6, 40, 0, 29, 45, 21, 0, 26, 26, 1);
    ObjectState tracked_object_3(5, 10, 1, 27, 34, 87, 1, 90, 26, 1);

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

    ObjectState new_object_1(1, 10, 1, 4, 15, 11, 0, 6, 100, 1);
    ObjectState new_object_2(2, 15, 2, 43, 6, 15, 1, 64, 200, 1);
    ObjectState new_object_3(1, 33, 1, 23, 9, 14, 0, 8, 300, 1);
    ObjectState new_object_4(3, 45, 2, 3, 16, 35, 1, 5, 400, 1);
    ObjectState new_object_5(4, 12, 1, 6, 18, 12, 0, 4, 500, 1);
    ObjectState new_object_6(2, 11, 1, 19, 32, 5, 1, 6, 600, 1);

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

TEST(FindTargetObject, validLogic){
    ros::NodeHandle env_state_node_handle;
    EnvironmentState env_state_test(&env_state_node_handle);

    ObjectState new_object_1(7, 50, 1, 4, 15, 12, 0, 6, 100, 1);
    ObjectState new_object_2(7, 8, 2, 43, 6, 15, 1, 64, 200, 1);
    ObjectState new_object_3(7, 100, 1, 67, 3, 623, 1, 45, 1200, 1);
    ObjectState new_object_4(7, 24, 1, 67, 3, 623, 1, 45, 1200, 1);

    ASSERT_EQ(env_state_test.target1, 0);
    ASSERT_EQ(env_state_test.target.get_obj_dx(), 255);
    ASSERT_EQ(env_state_test.target.get_obj_id(), 0);

    env_state_test.find_target_object(new_object_1); 
    ASSERT_EQ(env_state_test.target.get_obj_id(), new_object_1.get_obj_id());
    ASSERT_EQ(env_state_test.target.get_obj_dx(), new_object_1.get_obj_dx());

    env_state_test.target1 = 1; // not first time
    // different lane (don't update)
    env_state_test.find_target_object(new_object_2); 
    ASSERT_EQ(env_state_test.target.get_obj_lane(), 1);
    ASSERT_EQ(new_object_2.get_obj_lane(), 2);
    ASSERT_EQ(env_state_test.target.get_obj_lane(), new_object_1.get_obj_lane());
    ASSERT_EQ(env_state_test.target.get_obj_dx(), new_object_1.get_obj_dx());

    // same lane and id, target larger dx (udate)
    env_state_test.find_target_object(new_object_3); 
    ASSERT_EQ(env_state_test.target.get_obj_id(), new_object_3.get_obj_id());
    ASSERT_EQ(env_state_test.target.get_obj_dx(), new_object_3.get_obj_dx());

    // same lane and id, target smaller dx (update)
    env_state_test.find_target_object(new_object_4); 
    ASSERT_EQ(env_state_test.target.get_obj_id(), new_object_4.get_obj_id());
    ASSERT_EQ(env_state_test.target.get_obj_dx(), new_object_4.get_obj_dx());

}

bool target_output_cb_called = false;
void test_target_obj_cb(const common::target_output_msg& target_output_msg){target_output_cb_called = true;}

TEST(PubTargetObj, validLogic){
    ros::NodeHandle env_state_node_handle;
    EnvironmentState env_state_test(&env_state_node_handle);

    ros::NodeHandle target_obj_node_handle;
    ros::Subscriber sub = target_obj_node_handle.subscribe("target_obj", MESSAGE_BUFFER_SIZE, &test_target_obj_cb);

    ObjectState new_target(0, 255, 0, 0, 0, 0, 0, 0, 0, 0);
    env_state_test.target = new_target;
    env_state_test.publish_target_obj();
    ros::spinOnce();

    ASSERT_EQ(target_output_cb_called, false);
    ASSERT_EQ(env_state_test.target.get_obj_id(), 0);
    ASSERT_EQ(env_state_test.target.get_obj_dx(), 255);
}

bool tracked_output_cb_called = false;
void test_tracked_obj_cb(const common::tracked_output_msg& tracked_output_msg){tracked_output_cb_called = true;}

TEST(PubTrackedObj, validLogic){
    ros::NodeHandle env_state_node_handle;
    EnvironmentState env_state_test(&env_state_node_handle);

    ros::NodeHandle tracked_obj_node_handle;
    ros::Subscriber sub = tracked_obj_node_handle.subscribe("tracked_obj", MESSAGE_BUFFER_SIZE, &test_tracked_obj_cb);

    env_state_test.publish_tracked_obj();
    ros::spinOnce();

    ASSERT_EQ(tracked_output_cb_called, false);

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "env_state_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
