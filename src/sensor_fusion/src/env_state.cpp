#include "env_state.h"
// #include "string.h"

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
    ObjectState tracked_msg;
    tracked_msg.copy_info(filtered_msg); // copy constructor
    check_timestamp(tracked_msg); // removes outdated state vector
    update_env_state(tracked_msg); // update id of objects in state vector

    // // TODO:
    // object_output_msg.obj_id = 1;
    // object_output_msg.obj_dx = 2.2;
    // object_output_msg.obj_lane = 1;
    // object_output_msg.obj_vx = 3.3;
    // object_output_msg.obj_dy = 5.6;
    // object_output_msg.obj_ax = 12.4;
    // object_output_msg.obj_path = 1;
    // object_output_msg.obj_vy = 16.7;
    
    // cout  << "obj_id: "<< object_output_msg.obj_id << "\n"
    //                 << "obj_dx " << object_output_msg.obj_dx << "\n"
    //                 << "obj_lane " << object_output_msg.obj_lane << "\n"
    //                 << "obj_vx " << object_output_msg.obj_vx << "\n"
    //                 << "obj_dy" << object_output_msg.obj_dy << "\n"
    //                 << "obj_ax " << object_output_msg.obj_ax << "\n"
    //                 << "obj_path " << object_output_msg.obj_path << "\n"
    //                 << "obj_vy" << object_output_msg.obj_vy << "\n";


}


sensor_fusion::object_output_msg EnvironmentState::get_object_output_msg() { return object_output_msg; }

void EnvironmentState::add_object(const ObjectState& tracked_msg) {
  EnvironmentState::trackedObjects.push_back(tracked_msg); // error b/c trying to insert different data type to vector type ObjectState
}

void EnvironmentState::update_object(const ObjectState& tracked_msg, int index) {
  EnvironmentState::trackedObjects[index] = tracked_msg; // error from filtered_object_msg not same as ObjectState type
}

void EnvironmentState::check_timestamp(const ObjectState& tracked_msg) {
  
  for (int index = 0; index < EnvironmentState::trackedObjects.size(); index++){
    if ((tracked_msg.get_obj_timestamp() - EnvironmentState::trackedObjects[index].get_obj_timestamp())>10000){
      // removes tracked object from state vectors
      EnvironmentState::trackedObjects.erase(index);
    }  
}

void EnvironmentState::update_env_state(const ObjectState& tracked_msg) {
  
  for (int index = 0; index < EnvironmentState::trackedObjects.size(); index++){
    // if object is found in the state vector (has been tracked), update it's ID
    if (tracked_msg.get_obj_id() == EnvironmentState::trackedObjects[index].get_obj_id())
      update_object(tracked_msg, index);
  }
  // if object has not been tracked, add the object to state vector
  add_object(tracked_msg);
  
}

  
}


