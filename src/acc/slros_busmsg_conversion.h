#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <common/acc_output_msg.h>
#include <common/drive_ctrl_input_msg.h>
#include <common/target_output_msg.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include "ACC_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(common::acc_output_msg* msgPtr, SL_Bus_ACC_common_acc_output_msg const* busPtr);
void convertToBus(SL_Bus_ACC_common_acc_output_msg* busPtr, common::acc_output_msg const* msgPtr);

void convertFromBus(common::drive_ctrl_input_msg* msgPtr, SL_Bus_ACC_common_drive_ctrl_input_msg const* busPtr);
void convertToBus(SL_Bus_ACC_common_drive_ctrl_input_msg* busPtr, common::drive_ctrl_input_msg const* msgPtr);

void convertFromBus(common::target_output_msg* msgPtr, SL_Bus_ACC_common_target_output_msg const* busPtr);
void convertToBus(SL_Bus_ACC_common_target_output_msg* busPtr, common::target_output_msg const* msgPtr);

void convertFromBus(ros::Time* msgPtr, SL_Bus_ACC_ros_time_Time const* busPtr);
void convertToBus(SL_Bus_ACC_ros_time_Time* busPtr, ros::Time const* msgPtr);

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_ACC_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_ACC_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr);


#endif
