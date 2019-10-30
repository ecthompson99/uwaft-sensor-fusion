#include "ros/ros.h"
#include "can_tx_rx/can_tx_rx.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "can_tx_rx_feeder_test");
  ros::NodeHandle can_tx_rx_feeder_node_handle;
  ros::Publisher can_comms_pub = can_tx_rx_feeder_node_handle.advertise<can_tx_rx::can_comms_data_msg>("can_comms_data", CAN_MESSAGE_BUFFER_SIZE);
  can_tx_rx::can_comms_data_msg can_comms_msg;
  while (ros::ok()) {
    can_comms_msg.acc_switch_status = true;
    can_comms_msg.aeb_switch_status = true;
    can_comms_msg.lc_switch_status = true;
    can_comms_msg.target_accel = 0.1;
    can_comms_msg.acc_valid= true;
    can_comms_msg.hold_target_speed= true;
    can_comms_msg.speed_setpoint= 10.2;
    can_comms_msg.aeb_valid = true;
    can_comms_msg.aeb_override = true;
    can_comms_msg.lc_valid = true;
    can_comms_msg.wheel_angle = 0.75;
    can_comms_msg.alive_rolling_counter = 3;
    can_comms_pub.publish(can_comms_msg);
    ROS_INFO_STREAM("Streaming test data");
    ros::spinOnce();
  }
  return 0;
}
