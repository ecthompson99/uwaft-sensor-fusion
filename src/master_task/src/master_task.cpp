#include "master_task.h"

MasterTask::MasterTask(ros::NodeHandle* nodeHandle) : nh(nodeHandle) {
  drive_ctrl_sub =
      nh->subscribe("drive_control_input", MASTER_MESSAGE_BUFFER_SIZE, &MasterTask::drive_ctrl_msg_callback, this);

  sensor_diag_flag_sub = nh->subscribe("sensor_diagnostic_flag", MASTER_MESSAGE_BUFFER_SIZE,
                                       &MasterTask::sensor_diag_flag_msg_callback, this);

  sudo_driver_input_sub =
      nh->subscribe("sudo_driver_input", MASTER_MESSAGE_BUFFER_SIZE, &MasterTask::sudo_driver_input_msg_callback, this);

  master_task_pub = nh->advertise<master_task::can_comms_data_msg>("can_comms_data", MASTER_MESSAGE_BUFFER_SIZE);
}

MasterTask::~MasterTask() {}

void MasterTask::publish_can_comms_msg() { master_task_pub.publish(can_comms_msg); }

void MasterTask::drive_ctrl_msg_callback(const master_task::drive_ctrl_input_msg& drive_ctrl_msg) {
  can_comms_msg.acc_switch_status = drive_ctrl_msg.acc_enable;
  can_comms_msg.aeb_switch_status = drive_ctrl_msg.aeb_enable;
  can_comms_msg.lc_switch_status = drive_ctrl_msg.lc_enable;
  can_comms_msg.speed_setpoint = drive_ctrl_msg.acc_speed_set_point;

  ROS_INFO_STREAM("\n"
                  << "ACC enabled? " << drive_ctrl_msg.acc_enable << "\n"
                  << "AEB enabled? " << drive_ctrl_msg.aeb_enable << "\n"
                  << "LC enabled? " << drive_ctrl_msg.lc_enable << "\n"
                  << "ACC speed set point " << drive_ctrl_msg.acc_speed_set_point << "\n"
                  << "ACC dist set point " << drive_ctrl_msg.acc_dist_set_point << "\n");
}

void MasterTask::sensor_diag_flag_msg_callback(const master_task::sensor_diagnostic_flag_msg& sensor_msg) {
  // temporary workaround for array
  int first = sensor_msg.radar_reliability[0];
  int second = sensor_msg.radar_reliability[1];
  int third = sensor_msg.radar_reliability[2];
  int fourth = sensor_msg.radar_reliability[3];
  int fifth = sensor_msg.radar_reliability[4];
  int sixth = sensor_msg.radar_reliability[5];

  // temporary for unit tests
  if ((first == 1) && (second == 2)) {
    can_comms_msg.acc_valid = 1;
  }
  if ((third == 3) && (fourth == 4)) {
    can_comms_msg.aeb_valid = 2;
  }
  if ((fifth == 5) && (sixth == 6)) {
    can_comms_msg.lc_valid = 3;
  }

  ROS_INFO_STREAM("\n"
                  << "1st sensor " << first << "\n"
                  << "2nd sensor " << second << "\n"
                  << "3rd sensor " << third << "\n"
                  << "4th sensor " << fourth << "\n"
                  << "5th sensor " << fifth << "\n"
                  << "6th sensor " << sixth << "\n");
}

void MasterTask::sudo_driver_input_msg_callback(const master_task::sudo_driver_input_msg& input_msg) {
  can_comms_msg.aeb_override = input_msg.aeb_override;
  can_comms_msg.target_accel = input_msg.target_accel;
  can_comms_msg.wheel_angle = input_msg.wheel_angle;

  ROS_INFO_STREAM("\n"
                  << "AEB override " << input_msg.aeb_override << "\n"
                  << "Target Acceleration " << input_msg.target_accel << "\n"
                  << "Wheel angle " << input_msg.wheel_angle << "\n");
}

master_task::can_comms_data_msg MasterTask::get_can_comms_msg() { return can_comms_msg; }
