#include "env_state.h"
#include "string.h"

using namespace std;

EnvironmentState::EnvironmentState(ros::NodeHandle* node_handle) : env_state_node_handle(node_handle) {
  filtered_object_sub = env_state_node_handle->subscribe("filtered_object", MESSAGE_BUFFER_SIZE,
                                                            &EnvironmentState::filtered_object_callback, this);

  object_output_pub = env_state_node_handle->advertise<sensor_fusion::object_output_msg>("object_output",
                                                                                             MESSAGE_BUFFER_SIZE);
}

EnvironmentState::~EnvironmentState() {}

void EnvironmentState::publish_object_output() { object_output_pub.publish(object_output_msg); }

void EnvironmentState::filtered_object_callback(const sensor_fusion::filtered_object_msg& filtered_msg) {
    
    track_env_state(filtered_msg);

    // TODO:
    object_output_msg.obj_id = 1;
    object_output_msg.obj_dx = 2.2;
    object_output_msg.obj_lane = 1;
    object_output_msg.obj_vx = 3.3;
    object_output_msg.obj_dy = 5.6;
    object_output_msg.obj_ax = 12.4;
    object_output_msg.obj_in_lane = 1;
    object_output_msg.obj_vy = 16.7;
    
    cout  << "obj_id: "<< object_output_msg.obj_id << "\n"
                    << "obj_dx " << object_output_msg.obj_dx << "\n"
                    << "obj_lane " << object_output_msg.obj_lane << "\n"
                    << "obj_vx " << object_output_msg.obj_vx << "\n"
                    << "obj_dy" << object_output_msg.obj_dy << "\n"
                    << "obj_ax " << object_output_msg.obj_ax << "\n"
                    << "obj_in_lane " << object_output_msg.obj_in_lane << "\n"
                    << "obj_vy" << object_output_msg.obj_vy << "\n";


}


sensor_fusion::object_output_msg EnvironmentState::get_object_output_msg() { return object_output_msg; }

void EnvironmentState::add_object(const sensor_fusion::filtered_object_msg& filtered_msg) {
  EnvironmentState::trackedObjects.std::push_back(filtered_msg);
}

void EnvironmentState::update_object(const sensor_fusion::filtered_object_msg& filtered_msg, int index) {
  EnvironmentState::trackedObjects[index] = filtered_msg;
}

void EnvironmentState::track_env_state(const sensor_fusion::filtered_object_msg& filtered_msg) {
  
  for (int index = 0; index < EnvironmentState::trackedObjects.size(); index++){
    if (filtered_msg.obj_id == EnvironmentState::trackedObjects[index].get_obj_id())
      update_object(filtered_msg, index);
  }
  add_object(filtered_msg);
  
}


