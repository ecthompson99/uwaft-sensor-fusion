#ifndef __ENV_STATE_H__
#define __ENV_STATE_H__

#include "ros/ros.h"
#include "object_state.h"
#include "sensor_fusion/filtered_object_msg.h" // sub
#include "sensor_fusion/object_output_msg.h"  // pub
#include <vector>


static const uint8_t MESSAGE_BUFFER_SIZE = 10;

class EnvironmentState {
 public:
  std::vector<ObjectState> trackedObjects;
  ObjectState targetObjects[3] = {};
  
  EnvironmentState(ros::NodeHandle* env_state_node_handle);
  virtual ~EnvironmentState();
  void publish_object_output();
  void filtered_object_callback(const sensor_fusion::filtered_object_msg& filtered_msg);
  sensor_fusion::object_output_msg get_object_output_msg();

  void add_object(const ObjectState& tracked_msg);
  void update_object(const ObjectState& tracked_msg, int index);
  void check_timestamp(const ObjectState& tracked_msg);
  void update_env_state(const ObjectState& tracked_msg); 
  void find_target_objects(const ObjectState& tracked_msg);

  //bool env_state_vector_service_callback(sensor_fusion::env_state_srv::Request &request, sensor_fusion::env_state_srv::Response &response);

  private:
  ros::NodeHandle* env_state_node_handle;
  ros::Subscriber filtered_object_sub;
  ros::Publisher object_output_pub;
  sensor_fusion::object_output_msg object_output_msg;
  ros::ServiceServer my_service;
};

#endif  // __ENV_STATE_H__