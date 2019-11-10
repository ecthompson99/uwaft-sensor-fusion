#ifndef __can_tx_rx_H__
#define __can_tx_rx_H__

#include "ros/ros.h"
#include "can_tx_rx/can_comms_data_msg.h"
#include "can_tx_rx/drive_ctrl_input_msg.h"
#include "can_tx_rx/sensor_diagnostic_data_msg.h"
#include "can_tx_rx/raw_sensor_object_data_msg.h"

static const uint8_t CAN_MESSAGE_BUFFER_SIZE = 100;

class Can_Tx_Rx_Class {
 public:
  Can_Tx_Rx_Class(ros::NodeHandle* nodeHandle);
  void can_comms_msg_callback(const can_tx_rx::can_comms_data_msg& can_comms_msg);
  void publish_drive_ctrl_input();
  void publish_raw_sensor_object();
  void publish_sensor_diag();

 private:
  ros::NodeHandle* can_tx_rx_node_handle;
  ros::Subscriber can_comms_sub;
  ros::Publisher sensor_diag_pub;
  ros::Publisher raw_sensor_obj_pub;
  ros::Publisher drive_ctrl_input_pub;
};

#endif  // __can_tx_rx_H__
