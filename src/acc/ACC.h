//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: ACC.h
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
#ifndef RTW_HEADER_ACC_h_
#define RTW_HEADER_ACC_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef ACC_COMMON_INCLUDES_
# define ACC_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "slros_initialize.h"
#endif                                 // ACC_COMMON_INCLUDES_

#include "ACC_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block signals (default storage)
typedef struct {
  real_T Bc[98];
  real_T b_Mv[98];
  real_T cTol[98];
  real_T varargin_1[98];
  real_T vseq[62];
  real_T rseq[60];
  SL_Bus_ACC_common_target_output_msg In1;// '<S41>/In1'
  SL_Bus_ACC_common_drive_ctrl_input_msg In1_p;// '<S40>/In1'
  SL_Bus_ACC_common_target_output_msg b_varargout_2;
  SL_Bus_ACC_common_drive_ctrl_input_msg b_varargout_2_m;
  int16_T iAnew[98];
  int16_T iC[98];
  SL_Bus_ACC_common_acc_output_msg BusAssign;// '<Root>/BusAssign'
  boolean_T iAout[98];                 // '<S20>/optimizer'
  real_T RLinv[9];
  real_T D[9];
  real_T b_H[9];
  real_T U[9];
  real_T TL[9];
  real_T QQ[9];
  real_T RR[9];
  real_T c_A[9];
  real_T Opt[6];
  real_T Rhs[6];
  real_T xk[4];
  real_T xest[4];                      // '<S20>/optimizer'
  real_T f[3];
  real_T zopt[3];
  real_T r[3];
  real_T AcRow[3];
  real_T b_Ac[3];
  real_T work[3];
  real_T y_innov[2];
  real_T ymax_incr[2];
  real_T ymin_incr[2];
  real_T ymax_scale[2];                // '<S20>/ymax_scale'
  real_T ymin_scale[2];                // '<S20>/ymin_scale'
  real_T dv0[2];
  real_T ACC_Accel_Out;                // '<S6>/ACC Diagnostics'
  real_T umax_incr;
  real_T umin_incr;
  real_T DelBound;
  real_T lead_velocity;                // '<S1>/Sum6'
  real_T umax_scale;                   // '<S20>/umax_scale'
  real_T umin_scale;                   // '<S20>/umin_scale'
  real_T Switch2;                      // '<S12>/Switch2'
  real_T set_velocity;
  real_T Merge1;
  real_T safe_distance;
  real_T Switch1;
  real_T rMin;
  real_T Xnorm0;
  real_T cMin;
  real_T cVal;
  real_T t;
  real_T atmp;
  real_T xnorm;
  real_T tau_idx_0;
  real_T tau_idx_1;
  real_T temp;
  real_T c;
  real_T scale;
  int32_T i;
  int32_T rseq_tmp;
  int32_T j;
  int32_T b_k;
  int32_T idx;
  int32_T e_k;
  int32_T i_c;
  int32_T i0;
  int32_T Opt_tmp;
  int32_T U_tmp;
  int32_T i_k;
  int32_T b_i;
  int32_T c_i;
  int32_T e_i;
  int32_T f_i;
  boolean_T RateLimiter;               // '<S6>/ACC Diagnostics'
} B_ACC_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slros_internal_block_T obj; // '<S3>/SinkBlock'
  robotics_slros_internal_blo_e_T obj_f;// '<S5>/SourceBlock'
  robotics_slros_internal_blo_e_T obj_p;// '<S4>/SourceBlock'
  real_T last_mv_DSTATE;               // '<S20>/last_mv'
  real_T Delay_DSTATE;                 // '<S6>/Delay'
  real_T last_x_PreviousInput[4];      // '<S20>/last_x'
  real_T PrevY;                        // '<S6>/Rate Limiter'
  int32_T chartAbsoluteTimeCounter;    // '<S6>/ACC Diagnostics'
  int32_T durationLastReferenceTick_1; // '<S6>/ACC Diagnostics'
  int32_T durationLastReferenceTick_1_h;// '<S6>/ACC Diagnostics'
  int32_T durationLastReferenceTick_1_d;// '<S6>/ACC Diagnostics'
  int32_T durationLastReferenceTick_1_n;// '<S6>/ACC Diagnostics'
  uint8_T is_active_c14_ACC;           // '<S6>/ACC Diagnostics'
  uint8_T is_c14_ACC;                  // '<S6>/ACC Diagnostics'
  uint8_T is_ACC;                      // '<S6>/ACC Diagnostics'
  uint8_T is_Standard;                 // '<S6>/ACC Diagnostics'
  uint8_T is_ACC_Normal;               // '<S6>/ACC Diagnostics'
  boolean_T Memory_PreviousInput[98];  // '<S20>/Memory'
  boolean_T condWasTrueAtLastTimeStep_1;// '<S6>/ACC Diagnostics'
  boolean_T condWasTrueAtLastTimeStep_1_c;// '<S6>/ACC Diagnostics'
  boolean_T condWasTrueAtLastTimeStep_1_c0;// '<S6>/ACC Diagnostics'
  boolean_T condWasTrueAtLastTimeStep_1_d;// '<S6>/ACC Diagnostics'
} DW_ACC_T;

// Parameters (default storage)
struct P_ACC_T_ {
  struct_ECf3ZcX7QZBVymoYyKADRB data;  // Variable: data
                                       //  Referenced by:
                                       //    '<S7>/Constant'
                                       //    '<S7>/Constant1'
                                       //    '<S7>/Constant2'
                                       //    '<S7>/Constant3'
                                       //    '<S7>/Constant4'
                                       //    '<S7>/Constant6'
                                       //    '<S8>/Constant2'
                                       //    '<S8>/Constant3'
                                       //    '<S8>/Constant4'
                                       //    '<S8>/Switch1'

  real_T CompareToConstant_const;      // Mask Parameter: CompareToConstant_const
                                       //  Referenced by: '<S16>/Constant'

  SL_Bus_ACC_common_target_output_msg Out1_Y0;// Computed Parameter: Out1_Y0
                                              //  Referenced by: '<S41>/Out1'

  SL_Bus_ACC_common_target_output_msg Constant_Value;// Computed Parameter: Constant_Value
                                                     //  Referenced by: '<S5>/Constant'

  SL_Bus_ACC_common_drive_ctrl_input_msg Out1_Y0_a;// Computed Parameter: Out1_Y0_a
                                                   //  Referenced by: '<S40>/Out1'

  SL_Bus_ACC_common_drive_ctrl_input_msg Constant_Value_p;// Computed Parameter: Constant_Value_p
                                                          //  Referenced by: '<S4>/Constant'

  SL_Bus_ACC_common_acc_output_msg Constant_Value_e;// Computed Parameter: Constant_Value_e
                                                    //  Referenced by: '<S2>/Constant'

  real_T NotuseACCoutputconstant_Value;// Expression: 1
                                       //  Referenced by: '<S1>/Not use ACC output constant'

  real_T UseACCoutputconstant_Value;   // Expression: 0
                                       //  Referenced by: '<S1>/Use ACC output constant'

  real_T last_x_InitialCondition[4];   // Expression: lastx+xoff
                                       //  Referenced by: '<S20>/last_x'

  real_T last_mv_InitialCondition;     // Expression: lastu+uoff
                                       //  Referenced by: '<S20>/last_mv'

  real_T kmhtoms_Gain;                 // Expression: 1/3.6
                                       //  Referenced by: '<S1>/km//h to m//s'

  real_T umin_scale_Gain;              // Expression: RMVscale
                                       //  Referenced by: '<S20>/umin_scale'

  real_T umax_scale_Gain;              // Expression: RMVscale
                                       //  Referenced by: '<S20>/umax_scale'

  real_T Minimumvelocityconstant_Value;// Expression: acc.MinVelocity
                                       //  Referenced by: '<S1>/Minimum velocity constant'

  real_T ymin_scale_Gain[2];           // Expression: RYscale
                                       //  Referenced by: '<S20>/ymin_scale'

  real_T Unconstrained_Value;          // Expression: 0
                                       //  Referenced by: '<S1>/Unconstrained'

  real_T Maximumvelocityconstant_Value;// Expression: acc.MaxVelocity
                                       //  Referenced by: '<S1>/Maximum velocity constant'

  real_T ymax_scale_Gain[2];           // Expression: RYscale
                                       //  Referenced by: '<S20>/ymax_scale'

  real_T E_zero_Value;                 // Expression: zeros(1,1)
                                       //  Referenced by: '<S10>/E_zero'

  real_T umin_scale4_Gain;             // Expression: MVscale(:,ones(1,max(nCC,1)))'
                                       //  Referenced by: '<S20>/umin_scale4'

  real_T F_zero_Value[2];              // Expression: zeros(1,2)
                                       //  Referenced by: '<S10>/F_zero'

  real_T ymin_scale1_Gain[2];          // Expression: Yscale(:,ones(1,max(nCC,1)))'
                                       //  Referenced by: '<S20>/ymin_scale1'

  real_T G_zero_Value;                 // Expression: zeros(1,1)
                                       //  Referenced by: '<S10>/G_zero'

  real_T S_zero_Value;                 // Expression: zeros(1,1)
                                       //  Referenced by: '<S10>/S_zero'

  real_T ymin_scale2_Gain;             // Expression: MDscale(:,ones(1,max(nCC,1)))'
                                       //  Referenced by: '<S20>/ymin_scale2'

  real_T Constant1_Value;              // Expression: 1
                                       //  Referenced by: '<S1>/Constant1'

  real_T extmv_zero_Value;             // Expression: zeros(1,1)
                                       //  Referenced by: '<S10>/ext.mv_zero'

  real_T extmv_scale_Gain;             // Expression: RMVscale
                                       //  Referenced by: '<S20>/ext.mv_scale'

  real_T mvtarget_zero_Value;          // Expression: zeros(1,1)
                                       //  Referenced by: '<S10>/mv.target_zero'

  real_T extmv_scale1_Gain;            // Expression: RMVscale
                                       //  Referenced by: '<S20>/ext.mv_scale1'

  real_T ywt_zero_Value[2];            // Expression: zeros(2,1)
                                       //  Referenced by: '<S10>/y.wt_zero'

  real_T uwt_zero_Value;               // Expression: zeros(1,1)
                                       //  Referenced by: '<S10>/u.wt_zero'

  real_T duwt_zero_Value;              // Expression: zeros(1,1)
                                       //  Referenced by: '<S10>/du.wt_zero'

  real_T ecrwt_zero_Value;             // Expression: zeros(1,1)
                                       //  Referenced by: '<S10>/ecr.wt_zero'

  real_T umin_scale1_Gain;             // Expression: MVscale
                                       //  Referenced by: '<S20>/umin_scale1'

  real_T Delay_InitialCondition;       // Expression: 0.0
                                       //  Referenced by: '<S6>/Delay'

  real_T RateLimiter_RisingLim;        // Computed Parameter: RateLimiter_RisingLim
                                       //  Referenced by: '<S6>/Rate Limiter'

  real_T RateLimiter_FallingLim;       // Computed Parameter: RateLimiter_FallingLim
                                       //  Referenced by: '<S6>/Rate Limiter'

  real_T RateLimiter_IC;               // Expression: 0
                                       //  Referenced by: '<S6>/Rate Limiter'

  uint32_T Delay_DelayLength;          // Computed Parameter: Delay_DelayLength
                                       //  Referenced by: '<S6>/Delay'

  boolean_T Memory_InitialCondition[98];// Expression: iA
                                        //  Referenced by: '<S20>/Memory'

};

// Real-time Model Data Structure
struct tag_RTM_ACC_T {
  const char_T *errorStatus;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_ACC_T ACC_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
extern B_ACC_T ACC_B;

// Block states (default storage)
extern DW_ACC_T ACC_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void ACC_initialize(void);
  extern void ACC_step(void);
  extern void ACC_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_ACC_T *const ACC_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S12>/Data Type Duplicate' : Unused code path elimination
//  Block '<S12>/Data Type Propagation' : Unused code path elimination
//  Block '<S17>/Compare' : Unused code path elimination
//  Block '<S17>/Constant' : Unused code path elimination
//  Block '<S18>/Compare' : Unused code path elimination
//  Block '<S18>/Constant' : Unused code path elimination
//  Block '<S19>/Compare' : Unused code path elimination
//  Block '<S19>/Constant' : Unused code path elimination
//  Block '<S1>/Constant' : Unused code path elimination
//  Block '<S1>/DataTypeConversion_atrack' : Unused code path elimination
//  Block '<S21>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S22>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S23>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S24>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S25>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S26>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S27>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S28>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S29>/Vector Dimension Check' : Unused code path elimination
//  Block '<S30>/Vector Dimension Check' : Unused code path elimination
//  Block '<S31>/Vector Dimension Check' : Unused code path elimination
//  Block '<S32>/Vector Dimension Check' : Unused code path elimination
//  Block '<S33>/Vector Dimension Check' : Unused code path elimination
//  Block '<S34>/Vector Dimension Check' : Unused code path elimination
//  Block '<S35>/Vector Dimension Check' : Unused code path elimination
//  Block '<S36>/Vector Dimension Check' : Unused code path elimination
//  Block '<S37>/Vector Dimension Check' : Unused code path elimination
//  Block '<S38>/Vector Dimension Check' : Unused code path elimination
//  Block '<S20>/constant' : Unused code path elimination
//  Block '<S20>/umin_scale2' : Unused code path elimination
//  Block '<S20>/umin_scale3' : Unused code path elimination
//  Block '<S20>/umin_scale5' : Unused code path elimination
//  Block '<S20>/ym_zero' : Unused code path elimination
//  Block '<S1>/DataTypeConversion_L0' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_amax' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_amin' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_dmin' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_optsgn' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_reldist' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_vego' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_vlead' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_vset' : Eliminate redundant data type conversion
//  Block '<S20>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S20>/Data Type Conversion10' : Eliminate redundant data type conversion
//  Block '<S20>/Data Type Conversion11' : Eliminate redundant data type conversion
//  Block '<S20>/Data Type Conversion12' : Eliminate redundant data type conversion
//  Block '<S20>/Data Type Conversion13' : Eliminate redundant data type conversion
//  Block '<S20>/Data Type Conversion2' : Eliminate redundant data type conversion
//  Block '<S20>/Data Type Conversion3' : Eliminate redundant data type conversion
//  Block '<S20>/Data Type Conversion4' : Eliminate redundant data type conversion
//  Block '<S20>/Data Type Conversion5' : Eliminate redundant data type conversion
//  Block '<S20>/Data Type Conversion6' : Eliminate redundant data type conversion
//  Block '<S20>/Data Type Conversion7' : Eliminate redundant data type conversion
//  Block '<S20>/Data Type Conversion8' : Eliminate redundant data type conversion
//  Block '<S20>/Data Type Conversion9' : Eliminate redundant data type conversion
//  Block '<S20>/E Conversion' : Eliminate redundant data type conversion
//  Block '<S20>/F Conversion' : Eliminate redundant data type conversion
//  Block '<S20>/G Conversion' : Eliminate redundant data type conversion
//  Block '<S20>/Reshape' : Reshape block reduction
//  Block '<S20>/Reshape1' : Reshape block reduction
//  Block '<S20>/Reshape2' : Reshape block reduction
//  Block '<S20>/Reshape3' : Reshape block reduction
//  Block '<S20>/Reshape4' : Reshape block reduction
//  Block '<S20>/Reshape5' : Reshape block reduction
//  Block '<S20>/S Conversion' : Eliminate redundant data type conversion
//  Block '<S20>/mo or x Conversion' : Eliminate redundant data type conversion


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'ACC'
//  '<S1>'   : 'ACC/ACC'
//  '<S2>'   : 'ACC/Blank'
//  '<S3>'   : 'ACC/Publish'
//  '<S4>'   : 'ACC/drive_ctrl_input'
//  '<S5>'   : 'ACC/target_output'
//  '<S6>'   : 'ACC/ACC/ACC  Saturation'
//  '<S7>'   : 'ACC/ACC/ACC Gap Level'
//  '<S8>'   : 'ACC/ACC/ACC Limits'
//  '<S9>'   : 'ACC/ACC/ACC_Fault Diagnostics'
//  '<S10>'  : 'ACC/ACC/MPC'
//  '<S11>'  : 'ACC/ACC/ACC  Saturation/ACC Diagnostics'
//  '<S12>'  : 'ACC/ACC/ACC  Saturation/Saturation Dynamic'
//  '<S13>'  : 'ACC/ACC/ACC Gap Level/Near'
//  '<S14>'  : 'ACC/ACC/ACC Gap Level/Time_Far'
//  '<S15>'  : 'ACC/ACC/ACC Gap Level/Time_Medium'
//  '<S16>'  : 'ACC/ACC/ACC_Fault Diagnostics/Compare To Constant'
//  '<S17>'  : 'ACC/ACC/ACC_Fault Diagnostics/Compare To Constant1'
//  '<S18>'  : 'ACC/ACC/ACC_Fault Diagnostics/Compare To Constant2'
//  '<S19>'  : 'ACC/ACC/ACC_Fault Diagnostics/Compare To Constant3'
//  '<S20>'  : 'ACC/ACC/MPC/MPC'
//  '<S21>'  : 'ACC/ACC/MPC/MPC/MPC Matrix Signal Check'
//  '<S22>'  : 'ACC/ACC/MPC/MPC/MPC Matrix Signal Check1'
//  '<S23>'  : 'ACC/ACC/MPC/MPC/MPC Matrix Signal Check2'
//  '<S24>'  : 'ACC/ACC/MPC/MPC/MPC Preview Signal Check'
//  '<S25>'  : 'ACC/ACC/MPC/MPC/MPC Preview Signal Check1'
//  '<S26>'  : 'ACC/ACC/MPC/MPC/MPC Preview Signal Check2'
//  '<S27>'  : 'ACC/ACC/MPC/MPC/MPC Preview Signal Check3'
//  '<S28>'  : 'ACC/ACC/MPC/MPC/MPC Preview Signal Check4'
//  '<S29>'  : 'ACC/ACC/MPC/MPC/MPC Scalar Signal Check'
//  '<S30>'  : 'ACC/ACC/MPC/MPC/MPC Scalar Signal Check1'
//  '<S31>'  : 'ACC/ACC/MPC/MPC/MPC Vector Signal Check'
//  '<S32>'  : 'ACC/ACC/MPC/MPC/MPC Vector Signal Check1'
//  '<S33>'  : 'ACC/ACC/MPC/MPC/MPC Vector Signal Check11'
//  '<S34>'  : 'ACC/ACC/MPC/MPC/MPC Vector Signal Check2'
//  '<S35>'  : 'ACC/ACC/MPC/MPC/MPC Vector Signal Check3'
//  '<S36>'  : 'ACC/ACC/MPC/MPC/MPC Vector Signal Check4'
//  '<S37>'  : 'ACC/ACC/MPC/MPC/MPC Vector Signal Check5'
//  '<S38>'  : 'ACC/ACC/MPC/MPC/MPC Vector Signal Check6'
//  '<S39>'  : 'ACC/ACC/MPC/MPC/optimizer'
//  '<S40>'  : 'ACC/drive_ctrl_input/Enabled Subsystem'
//  '<S41>'  : 'ACC/target_output/Enabled Subsystem'

#endif                                 // RTW_HEADER_ACC_h_

//
// File trailer for generated code.
//
// [EOF]
//
