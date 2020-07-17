#include "master_task.h"

MasterTask::MasterTask(ros::NodeHandle* nodeHandle) : nh(nodeHandle) {
  drive_ctrl_sub =
      nh->subscribe("drive_control_input", MASTER_MESSAGE_BUFFER_SIZE, &MasterTask::drive_ctrl_msg_callback, this);

  //add controller pubs
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
