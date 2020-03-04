#ifndef __ENV_STATE_H__
#define __ENV_STATE_H__

#include "ros/ros.h"

#include "sensor_fusion/filtered_object_msg.h" // sub
#include "sensor_fusion/object_output_msg.h"  // pub


static const uint8_t MESSAGE_BUFFER_SIZE = 10;

class EnvironmentState {
 public:
  EnvironmentState(ros::NodeHandle* env_state_node_handle);
  virtual ~EnvironmentState();

  void publish_object_output();
  void filtered_object_callback(const sensor_fusion::filtered_object_msg& filtered_msg);
  sensor_fusion::object_output_msg get_object_output_msg();

  // TODO:
  bool is_equal(const sensor_fusion::filtered_object_msg& filtered_msg); // compare input msg to state objects
  bool is_in_state_array(); 
  void add_object();


 private:
  ros::NodeHandle* env_state_node_handle;
  ros::Subscriber filtered_object_sub;
  ros::Publisher object_output_pub;
  sensor_fusion::object_output_msg object_output_msg;
};

#endif  // __ENV_STATE_H__