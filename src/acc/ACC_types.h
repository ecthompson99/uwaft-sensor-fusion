//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: ACC_types.h
//
// Code generated for Simulink model 'ACC'.
//
// Model version                  : 1.0
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Sun Jun 27 14:22:40 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_ACC_types_h_
#define RTW_HEADER_ACC_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ACC_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ACC_ros_time_Time_

// MsgType=ros_time/Time
typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_ACC_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ACC_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ACC_std_msgs_Header_

// MsgType=std_msgs/Header
typedef struct {
  uint32_T Seq;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=FrameId_SL_Info:TruncateAction=warn 
  uint8_T FrameId[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=FrameId
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;

  // MsgType=ros_time/Time
  SL_Bus_ACC_ros_time_Time Stamp;
} SL_Bus_ACC_std_msgs_Header;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ACC_common_acc_output_msg_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ACC_common_acc_output_msg_

// MsgType=common/acc_output_msg
typedef struct {
  boolean_T AccFault;
  real_T AccAccel;

  // MsgType=std_msgs/Header
  SL_Bus_ACC_std_msgs_Header Header;
} SL_Bus_ACC_common_acc_output_msg;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ACC_common_drive_ctrl_input_msg_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ACC_common_drive_ctrl_input_msg_

// MsgType=common/drive_ctrl_input_msg
typedef struct {
  boolean_T AccActivation;
  boolean_T AebActivation;
  boolean_T LccActivation;
  boolean_T AccAllowed;
  boolean_T AebAllowed;
  boolean_T LccAllowed;

  // Int64Type=uint64
  real_T AliveRollingCounterMABx;

  // Int64Type=uint64
  real_T AliveRollingCounterJetson;
  real_T AccSpeedSetPoint;
  uint8_T AccGapLevel;
  real_T VehSpd;
  real_T StrAng;

  // MsgType=std_msgs/Header
  SL_Bus_ACC_std_msgs_Header Header;
} SL_Bus_ACC_common_drive_ctrl_input_msg;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ACC_common_target_output_msg_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ACC_common_target_output_msg_

// MsgType=common/target_output_msg
typedef struct {
  // Int64Type=uint64
  real_T ObjId;
  real_T ObjDx;
  uint8_T ObjLane;
  real_T ObjVx;
  real_T ObjDy;
  real_T ObjAx;
  boolean_T ObjPath;
  real_T ObjVy;
  real_T ObjTimestamp;
  uint8_T ObjTrackNum;

  // MsgType=std_msgs/Header
  SL_Bus_ACC_std_msgs_Header Header;
} SL_Bus_ACC_common_target_output_msg;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_ECf3ZcX7QZBVymoYyKADRB_
#define DEFINED_TYPEDEF_FOR_struct_ECf3ZcX7QZBVymoYyKADRB_

typedef struct {
  boolean_T acc_fault;
  boolean_T aeb_fault;
  boolean_T lcc_fault;
  boolean_T cav_alive;
  boolean_T radar_fault;
  boolean_T mobileye_fault;
  real_T veh_spd_thr;
  real_T acc_accel_max;
  real_T acc_accel_min_over;
  real_T accel_min_under;
  real_T acc_near_time;
  real_T acc_near_dist;
  real_T acc_med_time;
  real_T acc_med_dist;
  real_T acc_far_time;
  real_T acc_far_dist;
  real_T aeb_decel;
  real_T lcc_str;
  real_T lcc_max;
  real_T lcc_min;
} struct_ECf3ZcX7QZBVymoYyKADRB;

#endif

#ifndef typedef_robotics_slros_internal_block_T
#define typedef_robotics_slros_internal_block_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} robotics_slros_internal_block_T;

#endif                                 //typedef_robotics_slros_internal_block_T

#ifndef typedef_robotics_slros_internal_blo_e_T
#define typedef_robotics_slros_internal_blo_e_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} robotics_slros_internal_blo_e_T;

#endif                                 //typedef_robotics_slros_internal_blo_e_T

// Parameters (default storage)
typedef struct P_ACC_T_ P_ACC_T;

// Forward declaration for rtModel
typedef struct tag_RTM_ACC_T RT_MODEL_ACC_T;

#endif                                 // RTW_HEADER_ACC_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
