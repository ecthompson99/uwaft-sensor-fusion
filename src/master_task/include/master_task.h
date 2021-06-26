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
  
  // If the related diagnostic functions for CAV features are non-functional, the CAV features shall be turned off.
  void ACC_1_1();
  // If AEB is not functioning, ACC shall be in the OFF state
  void ACC_4();
  // A requested turn larger than 15 degrees shall instantly turn off ACC
  void ACC_15();
  // To ensure enable data is correct, 500 ms of a continuously active Enable signal must pass between the enabling of the ACC and the actual ACC engagement
  void ACC_16();
  // not in RTM???
  void ACC_17();
  // If the output ACC enable signal is in one state but  over a 300 ms period the initial signal is in the opposite state for more than 120 ms cumulatively, a signal error message shall be sent to the Jetson and the ACC shall be disabled until this condition has cleared
  void ACC_18();
  // ACC Allowed signal shall prevent the use of the ACC algorithm if it is not active
  void ACC_20();

  // AEB requests shall always negate ACC requests
  void AEB_13();
  // AEB shall not be active if traveling under 15 km/h
  void AEB_22();
  // To ensure enable data is correct, 500 ms of a continuously active Enable signal must pass between the enabling of the AEB and the actual AEB engagement
  void AEB_24();
  // AEB Allowed signal shall prevent the use of the AEB algorithm if it is not active
  void AEB_26();
  
  // If Mobileye appears non-functional, LCC, ACC and AEB shall be set to the OFF state
  void CAV_1_5();
  // If front MRR appears non-functional, ACC and AEB shall be set to the OFF state
  void CAV_1_6();
  // Automated braking commands cannot exceed 0.5 g deceleration
  void CAV_2_2();

  // CAVS rolling counter shall be sent every CAN frame
  void INT_1();
  // If the HSC alive rolling counter is not functioning as intended for 50 ms, all signals being sent to the HSC shall be set to zero/default values until the rolling counter is functioning properly for 100 ms
  void INT_2();
  // If the Jetson alive rolling counter is not functioning as intended for 50 ms, all signals being sent to the CAVS controller shall be set to zero/default values until the rolling counter is functioning properly for 100 ms
  void INT_7();

  // Steering angle limit shall be hard-coded into each system it is needed in
  void LCC_1();
  // Vehicle acceleration past 2.5 m/s^2 shall temporarily disable LCC until acceleration is below 2.5 m/s^2	
  void LCC_3();
  // To ensure enable data is correct, 500 ms of a continuously active Enable signal must pass between the enabling of the LCC and the actual LCC engagement
  void LCC_10();
  // To avoid accidental disengagement of the function, the LCC enable switch shall be OFF for a continuous 200 ms period
  void LCC_11();
  // If the output LCC enable signal is in one state but  over a 300 ms period the initial signal is in the opposite state for more than 120 ms cumulatively, a signal error message shall be sent to the Jetson and LCC shall be disabled until this condition has cleared
  void LCC_12();

  // helper function for INT_2 AND INT_7 that checks that INT_2 has been called within a certain time period
  void check_rolling_counters_called(); 
  
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

  uint64_t alive_rolling_counter_Jetson;
  uint64_t prev_rolling_counter_Jetson;
  bool first_RC_Jetson = true;
  ros::SteadyTime prev_time_rc_jetson;
  ros::SteadyTime prev_time_correct_jetson = ros::SteadyTime(0, 0);
  bool initial_Jetson_alive = true;

  uint64_t alive_rolling_counter_MABx; 
  uint64_t prev_rolling_counter_MABx; 
  bool first_RC = true;
  ros::SteadyTime prev_time_rc;
  ros::SteadyTime prev_time_correct_mabx = ros::SteadyTime(0, 0);
  bool initial_HSC_alive = true;

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
  bool initial_OFF_request_8 = true;

  ros::SteadyTime rc_time_init;
  ros::SteadyTime rc_time_init_jetson;
  ros::SteadyTime acc_period_start = ros::SteadyTime::now();
  ros::SteadyTime lcc_period_start = ros::SteadyTime::now();
  ros::SteadyTime ten_sec_start_acc = ros::SteadyTime::now();
  ros::SteadyTime ten_sec_start_lcc = ros::SteadyTime::now();
  ros::SteadyTime begin1;
  ros::SteadyTime begin3;
  ros::SteadyTime begin4;
  ros::SteadyTime begin5;
  ros::SteadyTime begin6;
  ros::SteadyTime begin7;
  ros::SteadyTime begin8;
  ros::SteadyTime begin9;

  unsigned int opposite_counter = 0;
  unsigned int opposite_counter_lcc = 0;

  double prev_vel; //ego vehicle previous velocity, used to calculate acceleration of ego vehicle
  double curr_vel; //ego vehicle current velocity
  double ego_accel;
  ros::SteadyTime prev_time;

  // constants
  unsigned int MAX_VEHICLE_SPEED = 15; // AEB_22 
  unsigned long PERIOD_MAX = 300; // ACC_18 and LCC_12
  unsigned long OPPOSITE_MAX = 12; // ACC_18 and LCC_12
  unsigned int MAX_LCC_STEER = 15; // ACC_15
};

#endif  // __MASTER_TASK_H__