#include "ros/ros.h"
#include "env_state.h"
#include "string.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "sensor_fusion"); 
  ros::NodeHandle env_state_node_handle;
  EnvironmentState env_state(&env_state_node_handle);

  while (ros::ok()) {
    env_state.publish_target_obj();
		env_state.publish_tracked_obj();
    ros::spinOnce();
  }

  return 0;
}


