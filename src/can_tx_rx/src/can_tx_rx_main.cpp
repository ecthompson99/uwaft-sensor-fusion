#include "ros/ros.h"
#include "can_tx_rx/can_tx_rx.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "can_tx_rx");
  ros::NodeHandle can_tx_rx_feeder_handle;
  Can_Tx_Rx_Class Can_Tx_Rx_Class(&can_tx_rx_feeder_handle);
  while (ros::ok()) {
    Can_Tx_Rx_Class.publish_drive_ctrl_input();
    Can_Tx_Rx_Class.publish_raw_sensor_object();
    Can_Tx_Rx_Class.publish_sensor_diag();
    ros::spinOnce();
  }
  return 0;
}
