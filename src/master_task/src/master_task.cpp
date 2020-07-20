#include "master_task.h"

MasterTask::MasterTask(ros::NodeHandle* nodeHandle) : nh(nodeHandle) {
  drive_ctrl_sub =
      nh->subscribe("drive_control_input", MASTER_MESSAGE_BUFFER_SIZE, &MasterTask::drive_ctrl_msg_callback, this);
  acc_sub = 
      nh->subscribe("acc_output", MASTER_MESSAGE_BUFFER_SIZE, &MasterTask::acc_output_msg_callback, this);
  aeb_sub = 
      nh->subscribe("aeb_output", MASTER_MESSAGE_BUFFER_SIZE, &MasterTask::aeb_output_msg_callback, this);
  lcc_sub = 
      nh->subscribe("lcc_output", MASTER_MESSAGE_BUFFER_SIZE, &MasterTask::lcc_output_msg_callback, this);
  sensor_diagnostic_CH2_client = 
      nh->serviceClient<common::sensor_diagnostic_flag_CH2>("sensor_diagnostic_CH2");
  sensor_diagnostic_CH3_client = 
      nh->serviceClient<common::sensor_diagnostic_flag_CH3>("sensor_diagnostic_CH3");
  sensor_diagnostic_CH4_client = 
      nh->serviceClient<common::sensor_diagnostic_flag_CH4>("sensor_diagnostic_CH4");
  
  master_task_pub = nh->advertise<common::can_comms_data_msg>("can_comms_data", MASTER_MESSAGE_BUFFER_SIZE);
}

MasterTask::~MasterTask() {}

void MasterTask::publish_can_comms_msg() { master_task_pub.publish(can_comms_msg); }

void MasterTask::drive_ctrl_msg_callback(const common::drive_ctrl_input_msg& drive_ctrl_msg) {

  ROS_INFO_STREAM("\n"
                  << "ACC enabled? " << unsigned(drive_ctrl_msg.acc_enable) << "\n"
                  << "AEB enabled? " << unsigned(drive_ctrl_msg.aeb_enable) << "\n"
                  << "LC enabled? " << unsigned(drive_ctrl_msg.lc_enable) << "\n"
                  << "ACC speed set point " << drive_ctrl_msg.acc_speed_set_point << "\n"
                  << "ACC dist set point " << drive_ctrl_msg.acc_dist_set_point << "\n");
}



common::can_comms_data_msg MasterTask::get_can_comms_msg() { return can_comms_msg; }
