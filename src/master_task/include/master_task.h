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
#include "std_srvs/Empty.h"

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
};

#endif  // __MASTER_TASK_H__
