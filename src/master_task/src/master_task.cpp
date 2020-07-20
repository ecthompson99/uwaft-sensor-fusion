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
  sensor_diagnostic_CH2_server = 
      nh->advertiseService<common::sensor_diagnostic_flag_CH2>("sensor_diagnostic_CH2");
  sensor_diagnostic_CH3_server = 
      nh->advertiseService<common::sensor_diagnostic_flag_CH3>("sensor_diagnostic_CH3");
  sensor_diagnostic_CH4_server = 
      nh->advertiseService<common::sensor_diagnostic_flag_CH4>("sensor_diagnostic_CH4");
  
  master_task_pub = nh->advertise<common::can_comms_data_msg>("can_comms_data", MASTER_MESSAGE_BUFFER_SIZE);
}

MasterTask::~MasterTask() {}

void MasterTask::publish_can_comms_msg() { master_task_pub.publish(can_comms_msg); }

void MasterTask::drive_ctrl_msg_callback(const common::drive_ctrl_input_msg& drive_ctrl_msg) 
{

  ROS_INFO_STREAM("\n"
                  << "ACC allowed? " << unsigned(drive_ctrl_msg.acc_allowed) << "\n"
                  << "AEB allowed? " << unsigned(drive_ctrl_msg.aeb_allowed) << "\n"
                  << "LC allowed? " << unsigned(drive_ctrl_msg.lcc_allowed) << "\n"
                  << "ACC speed set point " << drive_ctrl_msg.acc_speed_set_point << "\n"
                  << "Vehicle speed " << drive_ctrl_msg.veh_spd << "\n");
}

void MasterTask::acc_output_msg_callback(const common::acc_output_msg& acc_msg)
{
    ROS_INFO_STREAM("\n"
                  << "ACC fault? " << unsigned(acc_msg.acc_fault) << "\n"
                  << "ACC acceleration is " << acc_msg.acc_accel << "\n");
}

void MasterTask::aeb_output_msg_callback(const common::aeb_output_msg& aeb_msg)
{
    ROS_INFO_STREAM("\n"
                  << "AEB fault? " << unsigned(aeb_msg.aeb_fault) << "\n"
                  << "AEB engaged? " << unsigned(aeb_msg.aeb_engaged) << "\n"
                  << "AEB acceleration is " << aeb_msg.aeb_accel << "\n");
}

void MasterTask::lcc_output_msg_callback(const common::lcc_output_msg& lcc_msg)
{
    ROS_INFO_STREAM("\n"
                  << "LCC fault? " << unsigned(lcc_msg.lcc_fault) << "\n"
                  << "LCC steer is " << lcc_msg.lcc_steer << "\n");
}

void MasterTask::sensor_diagnostic_callback(common::sensor_diagnostic_flag_CH2::Request &req_CH2, common::sensor_diagnostic_flag_CH3::Request &req_CH3, common::sensor_diagnostic_flag_CH4::Request &req_CH4)
{
    ROS_INFO_STREAM("\n"
                  << "Front radar fault? " << unsigned(req_CH2.front_radar) << "\n"
                  << "Left radar fault? " << unsigned(req_CH3.left_corner_radar) << "\n"
                  << "Right radar fault? " << unsigned(req_CH3.right_corner_radar) << "\n"
                  << "Mobileye fault? " << unsigned(req_CH4.mobileye) << "\n");
}
common::can_comms_data_msg MasterTask::get_can_comms_msg() { return can_comms_msg; }
