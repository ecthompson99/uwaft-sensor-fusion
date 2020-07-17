#ifndef __MASTER_TASK_H__
#define __MASTER_TASK_H__

#include "ros/ros.h"

#include "common/can_comms_data_msg.h"
#include "common/drive_ctrl_input_msg.h"
#include "common/sensor_diagnostic_flag_msg.h"
#include "common/sudo_driver_input_msg.h"//update!!

static const uint8_t MASTER_MESSAGE_BUFFER_SIZE = 10;

class MasterTask {
 public:
  MasterTask(ros::NodeHandle* nodeHandle);
  virtual ~MasterTask();

  void publish_can_comms_msg();

  void drive_ctrl_msg_callback(const common::drive_ctrl_input_msg& drive_ctrl_msg);

  common::can_comms_data_msg get_can_comms_msg();

 protected:
 private:
  ros::NodeHandle* nh;
  ros::Subscriber drive_ctrl_sub;
  ros::Subscriber sensor_diag_flag_sub;//delete
  ros::Subscriber sudo_driver_input_sub;//delete
  ros::Publisher master_task_pub;
  common::can_comms_data_msg can_comms_msg;
};

#endif  // __MASTER_TASK_H__
