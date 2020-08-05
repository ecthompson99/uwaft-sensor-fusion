#include "env_state.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "env_state");
    ros::NodeHandle env_state_node_handle;
    EnvironmentState env_state = EnvironmentState(&env_state_node_handle);
	// while (ros::ok()){
	// 	env_state.publish_target_obj();
	// 	env_state.publish_tracked_obj();
	// 	ros::Rate(10).sleep();
	// 	ros::spinOnce();
	// }

	return 0;
}

