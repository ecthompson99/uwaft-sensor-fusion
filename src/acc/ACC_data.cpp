//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: ACC_data.cpp
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
#include "ACC.h"
#include "ACC_private.h"

// Block parameters (default storage)
P_ACC_T ACC_P = {
  // Variable: data
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

  {
    0,
    0,
    0,
    1,
    0,
    0,
    20.0,
    5.0,
    -3.5,
    -5.0,
    1.4,
    10.0,
    1.8,
    12.5,
    2.3,
    15.0,
    0.0,
    0.0,
    180.0,
    180.0
  },

  // Mask Parameter: CompareToConstant_const
  //  Referenced by: '<S16>/Constant'

  0.0,

  // Computed Parameter: Out1_Y0
  //  Referenced by: '<S41>/Out1'

  {
    0.0,                               // ObjId
    0.0,                               // ObjDx
    0U,                                // ObjLane
    0.0,                               // ObjVx
    0.0,                               // ObjDy
    0.0,                               // ObjAx
    false,                             // ObjPath
    0.0,                               // ObjVy
    0.0,                               // ObjTimestamp
    0U,                                // ObjTrackNum

    {
      0U,                              // Seq

      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                // FrameId

      {
        0U,                            // CurrentLength
        0U                             // ReceivedLength
      },                               // FrameId_SL_Info

      {
        0.0,                           // Sec
        0.0                            // Nsec
      }                                // Stamp
    }                                  // Header
  },

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S5>/Constant'

  {
    0.0,                               // ObjId
    0.0,                               // ObjDx
    0U,                                // ObjLane
    0.0,                               // ObjVx
    0.0,                               // ObjDy
    0.0,                               // ObjAx
    false,                             // ObjPath
    0.0,                               // ObjVy
    0.0,                               // ObjTimestamp
    0U,                                // ObjTrackNum

    {
      0U,                              // Seq

      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                // FrameId

      {
        0U,                            // CurrentLength
        0U                             // ReceivedLength
      },                               // FrameId_SL_Info

      {
        0.0,                           // Sec
        0.0                            // Nsec
      }                                // Stamp
    }                                  // Header
  },

  // Computed Parameter: Out1_Y0_a
  //  Referenced by: '<S40>/Out1'

  {
    false,                             // AccActivation
    false,                             // AebActivation
    false,                             // LccActivation
    false,                             // AccAllowed
    false,                             // AebAllowed
    false,                             // LccAllowed
    0.0,                               // AliveRollingCounterMABx
    0.0,                               // AliveRollingCounterJetson
    0.0,                               // AccSpeedSetPoint
    0U,                                // AccGapLevel
    0.0,                               // VehSpd
    0.0,                               // StrAng

    {
      0U,                              // Seq

      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                // FrameId

      {
        0U,                            // CurrentLength
        0U                             // ReceivedLength
      },                               // FrameId_SL_Info

      {
        0.0,                           // Sec
        0.0                            // Nsec
      }                                // Stamp
    }                                  // Header
  },

  // Computed Parameter: Constant_Value_p
  //  Referenced by: '<S4>/Constant'

  {
    false,                             // AccActivation
    false,                             // AebActivation
    false,                             // LccActivation
    false,                             // AccAllowed
    false,                             // AebAllowed
    false,                             // LccAllowed
    0.0,                               // AliveRollingCounterMABx
    0.0,                               // AliveRollingCounterJetson
    0.0,                               // AccSpeedSetPoint
    0U,                                // AccGapLevel
    0.0,                               // VehSpd
    0.0,                               // StrAng

    {
      0U,                              // Seq

      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                // FrameId

      {
        0U,                            // CurrentLength
        0U                             // ReceivedLength
      },                               // FrameId_SL_Info

      {
        0.0,                           // Sec
        0.0                            // Nsec
      }                                // Stamp
    }                                  // Header
  },

  // Computed Parameter: Constant_Value_e
  //  Referenced by: '<S2>/Constant'

  {
    false,                             // AccFault
    0.0,                               // AccAccel

    {
      0U,                              // Seq

      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                // FrameId

      {
        0U,                            // CurrentLength
        0U                             // ReceivedLength
      },                               // FrameId_SL_Info

      {
        0.0,                           // Sec
        0.0                            // Nsec
      }                                // Stamp
    }                                  // Header
  },

  // Expression: 1
  //  Referenced by: '<S1>/Not use ACC output constant'

  1.0,

  // Expression: 0
  //  Referenced by: '<S1>/Use ACC output constant'

  0.0,

  // Expression: lastx+xoff
  //  Referenced by: '<S20>/last_x'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: lastu+uoff
  //  Referenced by: '<S20>/last_mv'

  0.0,

  // Expression: 1/3.6
  //  Referenced by: '<S1>/km//h to m//s'

  0.27777777777777779,

  // Expression: RMVscale
  //  Referenced by: '<S20>/umin_scale'

  0.1,

  // Expression: RMVscale
  //  Referenced by: '<S20>/umax_scale'

  0.1,

  // Expression: acc.MinVelocity
  //  Referenced by: '<S1>/Minimum velocity constant'

  0.0,

  // Expression: RYscale
  //  Referenced by: '<S20>/ymin_scale'

  { 0.02, 0.02 },

  // Expression: 0
  //  Referenced by: '<S1>/Unconstrained'

  0.0,

  // Expression: acc.MaxVelocity
  //  Referenced by: '<S1>/Maximum velocity constant'

  50.0,

  // Expression: RYscale
  //  Referenced by: '<S20>/ymax_scale'

  { 0.02, 0.02 },

  // Expression: zeros(1,1)
  //  Referenced by: '<S10>/E_zero'

  0.0,

  // Expression: MVscale(:,ones(1,max(nCC,1)))'
  //  Referenced by: '<S20>/umin_scale4'

  10.0,

  // Expression: zeros(1,2)
  //  Referenced by: '<S10>/F_zero'

  { 0.0, 0.0 },

  // Expression: Yscale(:,ones(1,max(nCC,1)))'
  //  Referenced by: '<S20>/ymin_scale1'

  { 50.0, 50.0 },

  // Expression: zeros(1,1)
  //  Referenced by: '<S10>/G_zero'

  0.0,

  // Expression: zeros(1,1)
  //  Referenced by: '<S10>/S_zero'

  0.0,

  // Expression: MDscale(:,ones(1,max(nCC,1)))'
  //  Referenced by: '<S20>/ymin_scale2'

  50.0,

  // Expression: 1
  //  Referenced by: '<S1>/Constant1'

  1.0,

  // Expression: zeros(1,1)
  //  Referenced by: '<S10>/ext.mv_zero'

  0.0,

  // Expression: RMVscale
  //  Referenced by: '<S20>/ext.mv_scale'

  0.1,

  // Expression: zeros(1,1)
  //  Referenced by: '<S10>/mv.target_zero'

  0.0,

  // Expression: RMVscale
  //  Referenced by: '<S20>/ext.mv_scale1'

  0.1,

  // Expression: zeros(2,1)
  //  Referenced by: '<S10>/y.wt_zero'

  { 0.0, 0.0 },

  // Expression: zeros(1,1)
  //  Referenced by: '<S10>/u.wt_zero'

  0.0,

  // Expression: zeros(1,1)
  //  Referenced by: '<S10>/du.wt_zero'

  0.0,

  // Expression: zeros(1,1)
  //  Referenced by: '<S10>/ecr.wt_zero'

  0.0,

  // Expression: MVscale
  //  Referenced by: '<S20>/umin_scale1'

  10.0,

  // Expression: 0.0
  //  Referenced by: '<S6>/Delay'

  0.0,

  // Computed Parameter: RateLimiter_RisingLim
  //  Referenced by: '<S6>/Rate Limiter'

  0.08,

  // Computed Parameter: RateLimiter_FallingLim
  //  Referenced by: '<S6>/Rate Limiter'

  -0.16,

  // Expression: 0
  //  Referenced by: '<S6>/Rate Limiter'

  0.0,

  // Computed Parameter: Delay_DelayLength
  //  Referenced by: '<S6>/Delay'

  1U,

  // Expression: iA
  //  Referenced by: '<S20>/Memory'

  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

//
// File trailer for generated code.
//
// [EOF]
//
