#include "env_state.h"
#include <gtest/gtesh.h>

TEST(filtered_object_callback, validLogic){
  

}
TEST(filtered_object_callback, validCallback){


}

TEST(add_object, validLogic){}
TEST(update_object, validLogic){}
TEST(check_timestamp, validLogic){}
TEST(update_env_state, validLogic){}
TEST(find_target_object, validLogic){}

TEST(publish_obj_msg, validLogic){}
TEST(publish_obj_msg, validCallback){}




int main(int argc, char **argv) {
  ros::init(argc, argv, "env_state_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
