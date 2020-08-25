#ifndef __MASTER_TASK_H__
#define __MASTER_TASK_H__

#include "ros/ros.h"

#include "common/can_comms_data_msg.h"
#include "common/drive_ctrl_input_msg.h"
#include "common/sensor_diagnostic_flag_CH2.h"
#include "common/sensor_diagnostic_flag_CH3.h"
#include "common/sensor_diagnostic_flag_CH4.h"
#include "common/acc_output_msg.h"
#include "common/aeb_output_msg.h"
#include "common/lcc_output_msg.h"
#include <thread>
#include <time.h>

static const uint8_t MASTER_MESSAGE_BUFFER_SIZE = 10;

class MasterTask {
 public:
  MasterTask(ros::NodeHandle* nodeHandle);
  virtual ~MasterTask();

  void publish_can_comms_msg();

  void drive_ctrl_msg_callback(const common::drive_ctrl_input_msg& drive_ctrl_msg);
  void acc_output_msg_callback(const common::acc_output_msg& acc_msg);
  void aeb_output_msg_callback(const common::aeb_output_msg& aeb_msg);
  void lcc_output_msg_callback(const common::lcc_output_msg& lcc_msg);
  bool sensor_diagnostic_callback_CH2(common::sensor_diagnostic_flag_CH2::Request &req_CH2, common::sensor_diagnostic_flag_CH2::Response &res_CH2);
  bool sensor_diagnostic_callback_CH3(common::sensor_diagnostic_flag_CH3::Request &req_CH3, common::sensor_diagnostic_flag_CH3::Response &res_CH3);
  bool sensor_diagnostic_callback_CH4(common::sensor_diagnostic_flag_CH4::Request &req_CH4, common::sensor_diagnostic_flag_CH4::Response &res_CH4);
  void CAV_1_5();
  void CAV_1_6();
  void ACC_1_1();
  void ACC_15();
  void ACC_17();
  void ACC_20();
  void ACC_4();
  void AEB_13();
  void AEB_17();
  void AEB_22();
  void AEB_23();
  void AEB_24();
  void AEB_26();
  void AEB_14();
  void CAV_2_2();
  void LCC_1();
  void LCC_10();
  void LCC_11();
  void LCC_3();
  common::can_comms_data_msg get_can_comms_msg();

 protected:
 private:
  ros::NodeHandle* nh;
  ros::Subscriber drive_ctrl_sub;
  ros::Subscriber acc_sub;
  ros::Subscriber aeb_sub;
  ros::Subscriber lcc_sub;
  ros::Publisher master_task_pub;
  ros::ServiceServer sensor_diagnostic_CH2_server;
  ros::ServiceServer sensor_diagnostic_CH3_server;
  ros::ServiceServer sensor_diagnostic_CH4_server;
  common::can_comms_data_msg can_comms_msg;

  bool ACC_ACTIVATION;
  bool AEB_ACTIVATION;
  bool LCC_ACTIVATION;

  bool ACC_ALLOWED;
  bool AEB_ALLOWED;
  bool LCC_ALLOWED;

  uint64_t alive_rolling_counter_MABx;
  uint64_t alive_rolling_counter_Jetson;

  double acc_speed_set_point;
  int acc_gap_level;
  double VEHICLE_SPEED;
  double STEERING_ANGLE;
  //acc topic 
  bool ACC_FAULT;
  double ACC_ACCEL;
  //aeb topic
  bool AEB_FAULT;
  bool AEB_ENGAGED;
  double AEB_ACCEL;
  //lcc topic
  bool LCC_FAULT;
  double LCC_STEER;
  //sensor_diagnostic_flag_CH2
  bool FRONT_RADAR; //change to zero after unit testing!!
  //sensor_diagnostic_flag_CH3
  bool LEFT_CORNER_RADAR;
  bool RIGHT_CORNER_RADAR;
  //sensor_diagnostic_flag_CH4
  bool MOBILEYE;

  double buffer_time = 0.200;  // ACC_1_1

  bool initial_OFF_request = true;
  bool initial_OFF_request_2 = true;
  bool initial_OFF_request_3 = true;
  bool initial_OFF_request_4 = true;
  bool initial_OFF_request_5 = true;
  bool initial_OFF_request_6 = true;
  bool initial_OFF_request_7 = true;

  ros::WallTime begin1;
  ros::WallTime begin3;
  ros::WallTime begin4;
  ros::WallTime begin5;
  ros::WallTime begin6;
  ros::WallTime begin7;
  ros::WallTime begin8;

  double prev_vel; //ego vehicle previous velocity, used to calculate acceleration of ego vehicle
  double curr_vel; //ego vehicle current velocity
  double ego_accel;
  ros::WallTime prev_time;
};

#endif  // __MASTER_TASK_H__
