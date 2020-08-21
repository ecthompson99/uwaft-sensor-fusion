#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block ACC/drive_ctrl_input
extern SimulinkSubscriber<common::drive_ctrl_input_msg, SL_Bus_ACC_common_drive_ctrl_input_msg> Sub_ACC_140;

// For Block ACC/target_output
extern SimulinkSubscriber<common::target_output_msg, SL_Bus_ACC_common_target_output_msg> Sub_ACC_139;

// For Block ACC/Publish
extern SimulinkPublisher<common::acc_output_msg, SL_Bus_ACC_common_acc_output_msg> Pub_ACC_136;

void slros_node_init(int argc, char** argv);

#endif
