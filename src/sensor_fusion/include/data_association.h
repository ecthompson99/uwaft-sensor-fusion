#ifndef __DATA_ASSOCIATION_H__
#define __DATA_ASSOCIATION_H__

#include "ros/ros.h"
#include "sensor_fusion/raw_sensor_object_data_msg.h"  // sub
#include "object_state.h"


static const uint8_t MESSAGE_BUFFER_SIZE = 10;

class DataAssociation {
 public:
  DataAssociation(ros::NodeHandle* node_handle);
  void delete_potential_objects();

  friend class ObjectState;
  
  std::vector<int> toRemove;   //builds up a vector of indicies to be removed because removing random indicies while iterating  a second time will skip

 private:
  ros::NodeHandle* node_handle;
  ros::Subscriber sensor_data_obj_sub;
  ros::Publisher sensor_data_obj_pub;
  std::vector<ObjectState> potential_objs; // should be vector of objects

  const std::string KALMAN_FILTER_TOPIC = "kalman_filter";
  const std::string SENSOR_DATA_TOPIC = "raw_sensor_object_data";
  const int TOL = 5;
  void sensor_data_obj_callback(const sensor_fusion::raw_sensor_object_data_msg& sensor_data);
  bool objects_match(ObjectState obj, ObjectState sensor_data);  //both of type confirmedObjsContainer - post-conversion

};

#endif  // __DATA_ASSOCIATION_H__
