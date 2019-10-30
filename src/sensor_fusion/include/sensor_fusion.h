#ifndef __SENSOR_FUSION_H__
#define __SENSOR_FUSION_H__

#include "ros/ros.h"

#include "sensor_fusion/fused_object_data_msg.h"       // pub
#include "sensor_fusion/raw_sensor_object_data_msg.h"  // sub
#include "sensor_fusion/sensor_diagnostic_flag_msg.h"  // sub

static const uint8_t MESSAGE_BUFFER_SIZE = 10;

class SensorFusion {
 public:
  SensorFusion(ros::NodeHandle* sensor_fusion_node_handle);
  virtual ~SensorFusion();

  void publish_fused_data();
  void raw_sensor_obj_callback(const sensor_fusion::raw_sensor_object_data_msg& raw_msg);
  void sensor_diag_flag_msg_callback(const sensor_fusion::sensor_diagnostic_flag_msg& sensor_msg);

  sensor_fusion::fused_object_data_msg get_fused_data_msg();

 private:
  ros::NodeHandle* sensor_fusion_node_handle;
  ros::Subscriber raw_sensor_obj_sub;
  ros::Subscriber sensor_diag_flag_sub;
  ros::Publisher fused_obj_pub;
  sensor_fusion::fused_object_data_msg fused_data_msg;
};

#endif  // __SENSOR_FUSION_H__
