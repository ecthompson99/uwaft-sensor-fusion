#include "env_state.h"

EnvironmentState::EnvironmentState(ros::NodeHandle* node_handle) : env_state_node_handle(node_handle) {
  filtered_object_sub = env_state_node_handle->subscribe("filtered_object", MESSAGE_BUFFER_SIZE,
                                                            &EnvironmentState::filtered_object_callback, this);


  object_output_pub = env_state_node_handle->advertise<sensor_fusion::object_output_msg>("object_output",
                                                                                             MESSAGE_BUFFER_SIZE);
}

EnvironmentState::~EnvironmentState() {}

void EnvironmentState::publish_object_output() { object_output_pub.publish(object_output_msg); }

void EnvironmentState::filtered_object_callback(const sensor_fusion::filtered_object_msg& filtered_msg) {
 
}


sensor_fusion::object_output_msg EnvironmentState::get_object_output_msg() { return object_output_msg; }
