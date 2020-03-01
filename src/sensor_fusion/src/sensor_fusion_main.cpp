#include "ros/ros.h"
#include "env_state.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "sensor_fusion"); 
  ros::NodeHandle env_state_node_handle;
  EnvironmentState env_state(&env_state_node_handle);

  while (ros::ok()) {
    env_state.publish_object_output();
    ros::spinOnce();
  }

  return 0;
}


