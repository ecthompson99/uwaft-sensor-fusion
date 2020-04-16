#ifndef __ENV_STATE_H__
#define __ENV_STATE_H__

#include "ros/ros.h"
#include "object_state.h"
#include "sensor_fusion/filtered_object_msg.h" // sub
#include "sensor_fusion/target_output_msg.h"  // pub
#include "sensor_fusion/tracked_output_msg.h"  // pub
#include <vector>
#include "sensor_fusion/env_state_srv.h"  //service



static const uint8_t MESSAGE_BUFFER_SIZE = 10;

class EnvironmentState {
 public:
  std::vector<ObjectState> trackedObjects;
  ObjectState targetObjects[3];
  
  EnvironmentState(ros::NodeHandle* env_state_node_handle);
  virtual ~EnvironmentState();
  void publish_target_obj();
  void publish_tracked_obj();
  void filtered_object_callback(const sensor_fusion::filtered_object_msg& filtered_msg);
  sensor_fusion::target_output_msg get_target_output_msg();
  sensor_fusion::tracked_output_msg get_tracked_output_msg();

  void add_object(const ObjectState& tracked_msg);
  void update_object(const ObjectState& tracked_msg, int index);
  void check_timestamp(const ObjectState& tracked_msg);
  void update_env_state(const ObjectState& tracked_msg); 
  void find_target_objects(const ObjectState& tracked_msg);

    //FOR SERVICE
  bool env_state_srv_callback(sensor_fusion::env_state_srv::Request &req, sensor_fusion::env_state_srv::Response &res);

  private:
  ros::NodeHandle* env_state_node_handle;
  ros::Subscriber filtered_object_sub;
  ros::Publisher target_obj_pub;
	ros::Publisher tracked_obj_pub;
  ros::ServiceServer service;
};

#endif  // __ENV_STATE_H__
