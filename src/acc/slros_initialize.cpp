#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "ACC";

// For Block ACC/drive_ctrl_input
SimulinkSubscriber<common::drive_ctrl_input_msg, SL_Bus_ACC_common_drive_ctrl_input_msg> Sub_ACC_145;

// For Block ACC/target_output
SimulinkSubscriber<common::target_output_msg, SL_Bus_ACC_common_target_output_msg> Sub_ACC_144;

// For Block ACC/Publish
SimulinkPublisher<common::acc_output_msg, SL_Bus_ACC_common_acc_output_msg> Pub_ACC_141;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

