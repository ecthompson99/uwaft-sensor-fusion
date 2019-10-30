#include "ros/ros.h"
#include "can_tx_rx/can_tx_rx.h"

Can_Tx_Rx_Class::Can_Tx_Rx_Class(ros::NodeHandle* nodeHandle) : can_tx_rx_node_handle(nodeHandle) {
  can_comms_sub =
  can_tx_rx_node_handle->subscribe("can_comms_data", CAN_MESSAGE_BUFFER_SIZE, &Can_Tx_Rx_Class::can_comms_msg_callback, this);
  sensor_diag_pub = can_tx_rx_node_handle->advertise<can_tx_rx::sensor_diagnostic_data_msg>("sensor_diagnostic_data", CAN_MESSAGE_BUFFER_SIZE);
  raw_sensor_obj_pub = can_tx_rx_node_handle->advertise<can_tx_rx::raw_sensor_object_data_msg>("raw_sensor_object_data", CAN_MESSAGE_BUFFER_SIZE);
  drive_ctrl_input_pub = can_tx_rx_node_handle->advertise<can_tx_rx::drive_ctrl_input_msg>("drive_ctrl_input", CAN_MESSAGE_BUFFER_SIZE);
}

can_tx_rx::drive_ctrl_input_msg drive_ctrl_msg;
can_tx_rx::raw_sensor_object_data_msg raw_sensor_obj_msg;
can_tx_rx::sensor_diagnostic_data_msg sensor_diag_msg;
can_tx_rx::can_comms_data_msg can_comms_data;

void Can_Tx_Rx_Class::can_comms_msg_callback(const can_tx_rx::can_comms_data_msg& can_comms_msg) {
    ROS_INFO_STREAM("acc_switch_status:" << int(can_comms_msg.acc_switch_status));
    ROS_INFO_STREAM("acc_switch_status: " << int(can_comms_msg.aeb_switch_status));
    ROS_INFO_STREAM("lc_switch_status: " << int(can_comms_msg.lc_switch_status));
    ROS_INFO_STREAM("target_accel: " << can_comms_msg.target_accel);
    ROS_INFO_STREAM("acc_valid: " << int(can_comms_msg.acc_valid));
    ROS_INFO_STREAM("hold_target_speed: " << int(can_comms_msg.hold_target_speed));
    ROS_INFO_STREAM("speed setpoint: " << can_comms_msg.speed_setpoint);
    ROS_INFO_STREAM("aeb_valid: " << int(can_comms_msg.aeb_valid));
    ROS_INFO_STREAM("aeb_override: " << int(can_comms_msg.aeb_override));
    ROS_INFO_STREAM("lc_valid: " << int(can_comms_msg.lc_valid));
    ROS_INFO_STREAM("wheel angle: " << can_comms_msg.wheel_angle);
    ROS_INFO_STREAM("alive_rolling_counter: " << can_comms_msg.alive_rolling_counter);
    ROS_INFO_STREAM("--- ");
    can_comms_data = can_comms_msg;
}

void Can_Tx_Rx_Class::publish_drive_ctrl_input() {
  drive_ctrl_msg.acc_enable = false;
  drive_ctrl_msg.aeb_enable = true;
  drive_ctrl_msg.lc_enable = false;
  drive_ctrl_msg.acc_speed_set_point = 120.3;
  drive_ctrl_msg.acc_dist_set_point= 56.3;
  Can_Tx_Rx_Class::drive_ctrl_input_pub.publish(drive_ctrl_msg);
}

void Can_Tx_Rx_Class::publish_raw_sensor_object() {
  raw_sensor_obj_msg.radar_num = 2;
  raw_sensor_obj_msg.num_objects = 3 + ros::Time::now().toSec();
  raw_sensor_obj_msg.accel_x= {0.23, 0.13, 0.14, 13.2};
  raw_sensor_obj_msg.vel_x= {0.32, 0.31, 0.41, 2.31};
  raw_sensor_obj_msg.pos_x= {0.34, 0.45, 0.56, 0.67, 0.78};
  raw_sensor_obj_msg.pos_y= {0.43, 0.54, 0.65, 0.76, 0.87};
  raw_sensor_obj_msg.exist_prob = {0.34, 0.45, 0.56, 0.67, 0.78};
  raw_sensor_obj_msg.valid = {true, false, false, true, false};
  Can_Tx_Rx_Class::raw_sensor_obj_pub.publish(raw_sensor_obj_msg);
}

void Can_Tx_Rx_Class::publish_sensor_diag() {
  static int counter = 0;
  sensor_diag_msg.starter_consistency = 5;
  sensor_diag_msg.time_stamp = ros::Time::now().toSec();
  sensor_diag_msg.ender_consistency= 2.4;
  sensor_diag_msg.counter = counter;
  sensor_diag_msg.checksum= 0.56;
  sensor_diag_msg.itc_info= 17;
  sensor_diag_msg.hw_fail = false;
  sensor_diag_msg.sgu_fail = true;
  sensor_diag_msg.horizontal_misalign = 1.5;
  sensor_diag_msg.absorb_blind = 5.1;
  sensor_diag_msg.distort_blind = 8.2;
  sensor_diag_msg.message_counter = counter + 3;
  sensor_diag_msg.message_crc = counter - 2;
  Can_Tx_Rx_Class::sensor_diag_pub.publish(sensor_diag_msg);
  counter++;
}
