#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_ACC_common_acc_output_msg and common::acc_output_msg

void convertFromBus(common::acc_output_msg* msgPtr, SL_Bus_ACC_common_acc_output_msg const* busPtr)
{
  const std::string rosMessageType("common/acc_output_msg");

  msgPtr->acc_accel =  busPtr->AccAccel;
  msgPtr->acc_fault =  busPtr->AccFault;
  convertFromBus(&msgPtr->header, &busPtr->Header);
}

void convertToBus(SL_Bus_ACC_common_acc_output_msg* busPtr, common::acc_output_msg const* msgPtr)
{
  const std::string rosMessageType("common/acc_output_msg");

  busPtr->AccAccel =  msgPtr->acc_accel;
  busPtr->AccFault =  msgPtr->acc_fault;
  convertToBus(&busPtr->Header, &msgPtr->header);
}


// Conversions between SL_Bus_ACC_common_drive_ctrl_input_msg and common::drive_ctrl_input_msg

void convertFromBus(common::drive_ctrl_input_msg* msgPtr, SL_Bus_ACC_common_drive_ctrl_input_msg const* busPtr)
{
  const std::string rosMessageType("common/drive_ctrl_input_msg");

  msgPtr->acc_activation =  busPtr->AccActivation;
  msgPtr->acc_allowed =  busPtr->AccAllowed;
  msgPtr->acc_gap_level =  busPtr->AccGapLevel;
  msgPtr->acc_speed_set_point =  busPtr->AccSpeedSetPoint;
  msgPtr->aeb_activation =  busPtr->AebActivation;
  msgPtr->aeb_allowed =  busPtr->AebAllowed;
  msgPtr->alive_rolling_counter_Jetson = (uint64_t) busPtr->AliveRollingCounterJetson;
  msgPtr->alive_rolling_counter_MABx = (uint64_t) busPtr->AliveRollingCounterMABx;
  convertFromBus(&msgPtr->header, &busPtr->Header);
  msgPtr->lcc_activation =  busPtr->LccActivation;
  msgPtr->lcc_allowed =  busPtr->LccAllowed;
  msgPtr->str_ang =  busPtr->StrAng;
  msgPtr->veh_spd =  busPtr->VehSpd;
}

void convertToBus(SL_Bus_ACC_common_drive_ctrl_input_msg* busPtr, common::drive_ctrl_input_msg const* msgPtr)
{
  const std::string rosMessageType("common/drive_ctrl_input_msg");

  busPtr->AccActivation =  msgPtr->acc_activation;
  busPtr->AccAllowed =  msgPtr->acc_allowed;
  busPtr->AccGapLevel =  msgPtr->acc_gap_level;
  busPtr->AccSpeedSetPoint =  msgPtr->acc_speed_set_point;
  busPtr->AebActivation =  msgPtr->aeb_activation;
  busPtr->AebAllowed =  msgPtr->aeb_allowed;
  busPtr->AliveRollingCounterJetson = (real_T) msgPtr->alive_rolling_counter_Jetson;
  busPtr->AliveRollingCounterMABx = (real_T) msgPtr->alive_rolling_counter_MABx;
  convertToBus(&busPtr->Header, &msgPtr->header);
  busPtr->LccActivation =  msgPtr->lcc_activation;
  busPtr->LccAllowed =  msgPtr->lcc_allowed;
  busPtr->StrAng =  msgPtr->str_ang;
  busPtr->VehSpd =  msgPtr->veh_spd;
}


// Conversions between SL_Bus_ACC_common_target_output_msg and common::target_output_msg

void convertFromBus(common::target_output_msg* msgPtr, SL_Bus_ACC_common_target_output_msg const* busPtr)
{
  const std::string rosMessageType("common/target_output_msg");

  convertFromBus(&msgPtr->header, &busPtr->Header);
  msgPtr->obj_ax =  busPtr->ObjAx;
  msgPtr->obj_dx =  busPtr->ObjDx;
  msgPtr->obj_dy =  busPtr->ObjDy;
  msgPtr->obj_id =  busPtr->ObjId;
  msgPtr->obj_lane =  busPtr->ObjLane;
  msgPtr->obj_path =  busPtr->ObjPath;
  msgPtr->obj_timestamp =  busPtr->ObjTimestamp;
  msgPtr->obj_track_num =  busPtr->ObjTrackNum;
  msgPtr->obj_vx =  busPtr->ObjVx;
  msgPtr->obj_vy =  busPtr->ObjVy;
}

void convertToBus(SL_Bus_ACC_common_target_output_msg* busPtr, common::target_output_msg const* msgPtr)
{
  const std::string rosMessageType("common/target_output_msg");

  convertToBus(&busPtr->Header, &msgPtr->header);
  busPtr->ObjAx =  msgPtr->obj_ax;
  busPtr->ObjDx =  msgPtr->obj_dx;
  busPtr->ObjDy =  msgPtr->obj_dy;
  busPtr->ObjId =  msgPtr->obj_id;
  busPtr->ObjLane =  msgPtr->obj_lane;
  busPtr->ObjPath =  msgPtr->obj_path;
  busPtr->ObjTimestamp =  msgPtr->obj_timestamp;
  busPtr->ObjTrackNum =  msgPtr->obj_track_num;
  busPtr->ObjVx =  msgPtr->obj_vx;
  busPtr->ObjVy =  msgPtr->obj_vy;
}


// Conversions between SL_Bus_ACC_ros_time_Time and ros::Time

void convertFromBus(ros::Time* msgPtr, SL_Bus_ACC_ros_time_Time const* busPtr)
{
  const std::string rosMessageType("ros_time/Time");

  msgPtr->sec =  busPtr->Sec;
  msgPtr->nsec =  busPtr->Nsec;
}

void convertToBus(SL_Bus_ACC_ros_time_Time* busPtr, ros::Time const* msgPtr)
{
  const std::string rosMessageType("ros_time/Time");

  busPtr->Sec =  msgPtr->sec;
  busPtr->Nsec =  msgPtr->nsec;
}


// Conversions between SL_Bus_ACC_std_msgs_Header and std_msgs::Header

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_ACC_std_msgs_Header const* busPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromBusVariablePrimitiveArray(msgPtr->frame_id, busPtr->FrameId, busPtr->FrameId_SL_Info);
  msgPtr->seq =  busPtr->Seq;
  convertFromBus(&msgPtr->stamp, &busPtr->Stamp);
}

void convertToBus(SL_Bus_ACC_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToBusVariablePrimitiveArray(busPtr->FrameId, busPtr->FrameId_SL_Info, msgPtr->frame_id, slros::EnabledWarning(rosMessageType, "frame_id"));
  busPtr->Seq =  msgPtr->seq;
  convertToBus(&busPtr->Stamp, &msgPtr->stamp);
}

