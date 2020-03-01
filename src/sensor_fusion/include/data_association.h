#ifndef __DATA_ASSOCIATION_H__
#define __DATA_ASSOCIATION_H__

#include "ros/ros.h"
#include "sensor_fusion/raw_sensor_object_data_msg.h"  // sub

static const uint8_t MESSAGE_BUFFER_SIZE = 10;

class DataAssociation {
 public:
  DataAssociation(ros::NodeHandle* node_handle);
  void delete_potential_objects();

 private:
  ros::NodeHandle* node_handle;
  ros::Subscriber sensor_data_obj_sub;
  ros::Publisher sensor_data_obj_pub;
  std::vector<int> potential_objs; // should be vector of objects

  const std::string KALMAN_FILTER_TOPIC = "kalman_filter";
  const std::string SENSOR_DATA_TOPIC = "raw_sensor_object_data";

  void sensor_data_obj_callback(const sensor_fusion::raw_sensor_object_data_msg& sensor_data);
};

#endif  // __DATA_ASSOCIATION_H__
