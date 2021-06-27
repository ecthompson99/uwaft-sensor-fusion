//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: ACC.cpp
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

// Named constants for Chart: '<S6>/ACC Diagnostics'
#define ACC_IN_ACC                     ((uint8_T)1U)
#define ACC_IN_ACC_Normal              ((uint8_T)1U)
#define ACC_IN_ACC_OverSpd             ((uint8_T)2U)
#define ACC_IN_ACC_Zeroed              ((uint8_T)3U)
#define ACC_IN_FullBrakes              ((uint8_T)1U)
#define ACC_IN_HalfBrakes              ((uint8_T)2U)
#define ACC_IN_Init                    ((uint8_T)2U)
#define ACC_IN_NO_ACTIVE_CHILD         ((uint8_T)0U)
#define ACC_IN_RateLimited             ((uint8_T)2U)
#define ACC_IN_Standard                ((uint8_T)3U)

// Named constants for MATLAB Function: '<S20>/optimizer'
#define ACC_RMDscale                   (0.02)
#define ACC_enable_value               (0.0)
#define ACC_nv                         (2.0)
#define ACC_ny                         (2.0)
#define ACC_voff                       (0.4)

// Block signals (default storage)
B_ACC_T ACC_B;

// Block states (default storage)
DW_ACC_T ACC_DW;

// Real-time model
RT_MODEL_ACC_T ACC_M_;
RT_MODEL_ACC_T *const ACC_M = &ACC_M_;

// Forward declaration for local functions
static void ACC_Unconstrained(const real_T b_Hinv[9], const real_T f[3], real_T
  x[3], int16_T n);
static real_T ACC_norm(const real_T x[3]);
static void ACC_abs(const real_T x[3], real_T y[3]);
static void ACC_abs_h(const real_T x[98], real_T y[98]);
static real_T ACC_xnrm2(int32_T n, const real_T x[9], int32_T ix0);
static void ACC_xgemv(int32_T m, int32_T n, const real_T b_A[9], int32_T ia0,
                      const real_T x[9], int32_T ix0, real_T y[3]);
static void ACC_xgerc(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
                      real_T y[3], real_T b_A[9], int32_T ia0);
static void ACC_qr(const real_T b_A[9], real_T Q[9], real_T R[9]);
static real_T ACC_KWIKfactor(const real_T b_Ac[294], const int16_T iC[98],
  int16_T nA, const real_T b_Linv[9], real_T RLinv[9], real_T D[9], real_T b_H[9],
  int16_T n);
static void ACC_DropConstraint(int16_T kDrop, int16_T iA[98], int16_T *nA,
  int16_T iC[98]);
static int16_T ACC_rdivide_helper(int16_T x, int16_T y);
static void ACC_qpkwik(const real_T b_Linv[9], const real_T b_Hinv[9], const
  real_T f[3], const real_T b_Ac[294], const real_T b[98], int16_T iA[98],
  int16_T b_maxiter, real_T FeasTol, real_T x[3], real_T lambda[98], real_T
  *status);
static void ACC_Standard(const real_T *set_velocity, const real_T *Merge1, const
  real_T *safe_distance, real_T *umin_scale1, real_T *Switch1);
static real_T ACC_mod(real_T x);
static void matlabCodegenHandle_matlabCod_e(robotics_slros_internal_blo_e_T *obj);
static void matlabCodegenHandle_matlabCodeg(robotics_slros_internal_block_T *obj);

// Function for MATLAB Function: '<S20>/optimizer'
static void ACC_Unconstrained(const real_T b_Hinv[9], const real_T f[3], real_T
  x[3], int16_T n)
{
  int32_T i;
  int32_T i_0;
  for (i = 1; i - 1 < n; i++) {
    i_0 = (int16_T)i;
    x[(int16_T)i - 1] = (-b_Hinv[i_0 - 1] * f[0] + -b_Hinv[i_0 + 2] * f[1]) +
      -b_Hinv[i_0 + 5] * f[2];
  }
}

// Function for MATLAB Function: '<S20>/optimizer'
static real_T ACC_norm(const real_T x[3])
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

// Function for MATLAB Function: '<S20>/optimizer'
static void ACC_abs(const real_T x[3], real_T y[3])
{
  y[0] = fabs(x[0]);
  y[1] = fabs(x[1]);
  y[2] = fabs(x[2]);
}

// Function for MATLAB Function: '<S20>/optimizer'
static void ACC_abs_h(const real_T x[98], real_T y[98])
{
  int32_T k;
  for (k = 0; k < 98; k++) {
    y[k] = fabs(x[k]);
  }
}

// Function for MATLAB Function: '<S20>/optimizer'
static real_T ACC_xnrm2(int32_T n, const real_T x[9], int32_T ix0)
{
  real_T y;
  int32_T kend;
  real_T absxk;
  real_T t;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      ACC_B.scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = fabs(x[k - 1]);
        if (absxk > ACC_B.scale) {
          t = ACC_B.scale / absxk;
          y = y * t * t + 1.0;
          ACC_B.scale = absxk;
        } else {
          t = absxk / ACC_B.scale;
          y += t * t;
        }
      }

      y = ACC_B.scale * sqrt(y);
    }
  }

  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T a;
  a = fabs(u0);
  y = fabs(u1);
  if (a < y) {
    a /= y;
    y *= sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = sqrt(y * y + 1.0) * a;
  } else {
    if (!rtIsNaN(y)) {
      y = a * 1.4142135623730951;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S20>/optimizer'
static void ACC_xgemv(int32_T m, int32_T n, const real_T b_A[9], int32_T ia0,
                      const real_T x[9], int32_T ix0, real_T y[3])
{
  int32_T ix;
  int32_T b_iy;
  int32_T b;
  int32_T iac;
  int32_T d;
  int32_T ia;
  if ((m != 0) && (n != 0)) {
    for (b_iy = 0; b_iy < n; b_iy++) {
      y[b_iy] = 0.0;
    }

    b_iy = 0;
    b = (n - 1) * 3 + ia0;
    for (iac = ia0; iac <= b; iac += 3) {
      ix = ix0;
      ACC_B.c = 0.0;
      d = (iac + m) - 1;
      for (ia = iac; ia <= d; ia++) {
        ACC_B.c += b_A[ia - 1] * x[ix - 1];
        ix++;
      }

      y[b_iy] += ACC_B.c;
      b_iy++;
    }
  }
}

// Function for MATLAB Function: '<S20>/optimizer'
static void ACC_xgerc(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
                      real_T y[3], real_T b_A[9], int32_T ia0)
{
  int32_T jA;
  int32_T jy;
  int32_T ix;
  int32_T j;
  int32_T b;
  int32_T ijA;
  if (!(alpha1 == 0.0)) {
    jA = ia0 - 1;
    jy = 0;
    for (j = 0; j < n; j++) {
      if (y[jy] != 0.0) {
        ACC_B.temp = y[jy] * alpha1;
        ix = ix0;
        b = m + jA;
        for (ijA = jA; ijA < b; ijA++) {
          b_A[ijA] += b_A[ix - 1] * ACC_B.temp;
          ix++;
        }
      }

      jy++;
      jA += 3;
    }
  }
}

// Function for MATLAB Function: '<S20>/optimizer'
static void ACC_qr(const real_T b_A[9], real_T Q[9], real_T R[9])
{
  int32_T knt;
  int32_T lastc;
  int32_T coltop;
  int32_T b_coltop;
  int32_T exitg1;
  boolean_T exitg2;
  memcpy(&ACC_B.c_A[0], &b_A[0], 9U * sizeof(real_T));
  ACC_B.work[0] = 0.0;
  ACC_B.work[1] = 0.0;
  ACC_B.work[2] = 0.0;
  ACC_B.atmp = ACC_B.c_A[0];
  ACC_B.tau_idx_0 = 0.0;
  ACC_B.xnorm = ACC_xnrm2(2, ACC_B.c_A, 2);
  if (ACC_B.xnorm != 0.0) {
    ACC_B.xnorm = rt_hypotd_snf(ACC_B.c_A[0], ACC_B.xnorm);
    if (ACC_B.c_A[0] >= 0.0) {
      ACC_B.xnorm = -ACC_B.xnorm;
    }

    if (fabs(ACC_B.xnorm) < 1.0020841800044864E-292) {
      knt = -1;
      do {
        knt++;
        for (lastc = 1; lastc < 3; lastc++) {
          ACC_B.c_A[lastc] *= 9.9792015476736E+291;
        }

        ACC_B.xnorm *= 9.9792015476736E+291;
        ACC_B.atmp *= 9.9792015476736E+291;
      } while (!(fabs(ACC_B.xnorm) >= 1.0020841800044864E-292));

      ACC_B.xnorm = rt_hypotd_snf(ACC_B.atmp, ACC_xnrm2(2, ACC_B.c_A, 2));
      if (ACC_B.atmp >= 0.0) {
        ACC_B.xnorm = -ACC_B.xnorm;
      }

      ACC_B.tau_idx_0 = (ACC_B.xnorm - ACC_B.atmp) / ACC_B.xnorm;
      ACC_B.atmp = 1.0 / (ACC_B.atmp - ACC_B.xnorm);
      for (lastc = 1; lastc < 3; lastc++) {
        ACC_B.c_A[lastc] *= ACC_B.atmp;
      }

      for (lastc = 0; lastc <= knt; lastc++) {
        ACC_B.xnorm *= 1.0020841800044864E-292;
      }

      ACC_B.atmp = ACC_B.xnorm;
    } else {
      ACC_B.tau_idx_0 = (ACC_B.xnorm - ACC_B.c_A[0]) / ACC_B.xnorm;
      ACC_B.atmp = 1.0 / (ACC_B.c_A[0] - ACC_B.xnorm);
      for (knt = 1; knt < 3; knt++) {
        ACC_B.c_A[knt] *= ACC_B.atmp;
      }

      ACC_B.atmp = ACC_B.xnorm;
    }
  }

  ACC_B.c_A[0] = 1.0;
  if (ACC_B.tau_idx_0 != 0.0) {
    knt = 3;
    lastc = 0;
    while ((knt > 0) && (ACC_B.c_A[lastc + 2] == 0.0)) {
      knt--;
      lastc--;
    }

    lastc = 2;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      coltop = (lastc - 1) * 3 + 4;
      b_coltop = coltop;
      do {
        exitg1 = 0;
        if (b_coltop <= (coltop + knt) - 1) {
          if (ACC_B.c_A[b_coltop - 1] != 0.0) {
            exitg1 = 1;
          } else {
            b_coltop++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    knt = 0;
    lastc = 0;
  }

  if (knt > 0) {
    ACC_xgemv(knt, lastc, ACC_B.c_A, 4, ACC_B.c_A, 1, ACC_B.work);
    ACC_xgerc(knt, lastc, -ACC_B.tau_idx_0, 1, ACC_B.work, ACC_B.c_A, 4);
  }

  ACC_B.c_A[0] = ACC_B.atmp;
  ACC_B.atmp = ACC_B.c_A[4];
  ACC_B.tau_idx_1 = 0.0;
  ACC_B.xnorm = ACC_xnrm2(1, ACC_B.c_A, 6);
  if (ACC_B.xnorm != 0.0) {
    ACC_B.xnorm = rt_hypotd_snf(ACC_B.c_A[4], ACC_B.xnorm);
    if (ACC_B.c_A[4] >= 0.0) {
      ACC_B.xnorm = -ACC_B.xnorm;
    }

    if (fabs(ACC_B.xnorm) < 1.0020841800044864E-292) {
      knt = -1;
      do {
        knt++;
        for (lastc = 5; lastc < 6; lastc++) {
          ACC_B.c_A[lastc] *= 9.9792015476736E+291;
        }

        ACC_B.xnorm *= 9.9792015476736E+291;
        ACC_B.atmp *= 9.9792015476736E+291;
      } while (!(fabs(ACC_B.xnorm) >= 1.0020841800044864E-292));

      ACC_B.xnorm = rt_hypotd_snf(ACC_B.atmp, ACC_xnrm2(1, ACC_B.c_A, 6));
      if (ACC_B.atmp >= 0.0) {
        ACC_B.xnorm = -ACC_B.xnorm;
      }

      ACC_B.tau_idx_1 = (ACC_B.xnorm - ACC_B.atmp) / ACC_B.xnorm;
      ACC_B.atmp = 1.0 / (ACC_B.atmp - ACC_B.xnorm);
      for (lastc = 5; lastc < 6; lastc++) {
        ACC_B.c_A[lastc] *= ACC_B.atmp;
      }

      for (lastc = 0; lastc <= knt; lastc++) {
        ACC_B.xnorm *= 1.0020841800044864E-292;
      }

      ACC_B.atmp = ACC_B.xnorm;
    } else {
      ACC_B.tau_idx_1 = (ACC_B.xnorm - ACC_B.c_A[4]) / ACC_B.xnorm;
      ACC_B.atmp = 1.0 / (ACC_B.c_A[4] - ACC_B.xnorm);
      for (knt = 5; knt < 6; knt++) {
        ACC_B.c_A[knt] *= ACC_B.atmp;
      }

      ACC_B.atmp = ACC_B.xnorm;
    }
  }

  ACC_B.c_A[4] = 1.0;
  if (ACC_B.tau_idx_1 != 0.0) {
    knt = 2;
    lastc = 3;
    while ((knt > 0) && (ACC_B.c_A[lastc + 2] == 0.0)) {
      knt--;
      lastc--;
    }

    lastc = 1;
    b_coltop = 8;
    do {
      exitg1 = 0;
      if (b_coltop <= knt + 7) {
        if (ACC_B.c_A[b_coltop - 1] != 0.0) {
          exitg1 = 1;
        } else {
          b_coltop++;
        }
      } else {
        lastc = 0;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  } else {
    knt = 0;
    lastc = 0;
  }

  if (knt > 0) {
    ACC_xgemv(knt, lastc, ACC_B.c_A, 8, ACC_B.c_A, 5, ACC_B.work);
    ACC_xgerc(knt, lastc, -ACC_B.tau_idx_1, 5, ACC_B.work, ACC_B.c_A, 8);
  }

  ACC_B.c_A[4] = ACC_B.atmp;
  R[0] = ACC_B.c_A[0];
  for (knt = 1; knt + 1 < 4; knt++) {
    R[knt] = 0.0;
  }

  ACC_B.work[0] = 0.0;
  for (knt = 0; knt < 2; knt++) {
    R[knt + 3] = ACC_B.c_A[knt + 3];
  }

  for (knt = 2; knt + 1 < 4; knt++) {
    R[knt + 3] = 0.0;
  }

  ACC_B.work[1] = 0.0;
  for (knt = 0; knt < 3; knt++) {
    R[knt + 6] = ACC_B.c_A[knt + 6];
  }

  ACC_B.work[2] = 0.0;
  ACC_B.c_A[8] = 1.0;
  for (knt = 0; knt < 2; knt++) {
    ACC_B.c_A[7 - knt] = 0.0;
  }

  ACC_B.c_A[4] = 1.0;
  if (ACC_B.tau_idx_1 != 0.0) {
    knt = 2;
    lastc = 5;
    while ((knt > 0) && (ACC_B.c_A[lastc] == 0.0)) {
      knt--;
      lastc--;
    }

    lastc = 1;
    coltop = 7;
    do {
      exitg1 = 0;
      if (coltop + 1 <= 7 + knt) {
        if (ACC_B.c_A[coltop] != 0.0) {
          exitg1 = 1;
        } else {
          coltop++;
        }
      } else {
        lastc = 0;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  } else {
    knt = 0;
    lastc = 0;
  }

  if (knt > 0) {
    ACC_xgemv(knt, lastc, ACC_B.c_A, 8, ACC_B.c_A, 5, ACC_B.work);
    ACC_xgerc(knt, lastc, -ACC_B.tau_idx_1, 5, ACC_B.work, ACC_B.c_A, 8);
  }

  for (knt = 5; knt < 6; knt++) {
    ACC_B.c_A[knt] *= -ACC_B.tau_idx_1;
  }

  ACC_B.c_A[4] = 1.0 - ACC_B.tau_idx_1;
  ACC_B.c_A[3] = 0.0;
  ACC_B.c_A[0] = 1.0;
  if (ACC_B.tau_idx_0 != 0.0) {
    knt = 3;
    lastc = 2;
    while ((knt > 0) && (ACC_B.c_A[lastc] == 0.0)) {
      knt--;
      lastc--;
    }

    lastc = 2;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      b_coltop = (lastc - 1) * 3 + 3;
      coltop = b_coltop;
      do {
        exitg1 = 0;
        if (coltop + 1 <= b_coltop + knt) {
          if (ACC_B.c_A[coltop] != 0.0) {
            exitg1 = 1;
          } else {
            coltop++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    knt = 0;
    lastc = 0;
  }

  if (knt > 0) {
    ACC_xgemv(knt, lastc, ACC_B.c_A, 4, ACC_B.c_A, 1, ACC_B.work);
    ACC_xgerc(knt, lastc, -ACC_B.tau_idx_0, 1, ACC_B.work, ACC_B.c_A, 4);
  }

  for (knt = 1; knt < 3; knt++) {
    ACC_B.c_A[knt] *= -ACC_B.tau_idx_0;
  }

  ACC_B.c_A[0] = 1.0 - ACC_B.tau_idx_0;
  for (knt = 0; knt < 3; knt++) {
    Q[3 * knt] = ACC_B.c_A[3 * knt];
    Q[1 + 3 * knt] = ACC_B.c_A[3 * knt + 1];
    Q[2 + 3 * knt] = ACC_B.c_A[3 * knt + 2];
  }
}

// Function for MATLAB Function: '<S20>/optimizer'
static real_T ACC_KWIKfactor(const real_T b_Ac[294], const int16_T iC[98],
  int16_T nA, const real_T b_Linv[9], real_T RLinv[9], real_T D[9], real_T b_H[9],
  int16_T n)
{
  real_T Status;
  int16_T b_j;
  int16_T c_k;
  int32_T i;
  int32_T TL_tmp;
  int32_T exitg1;
  int32_T exitg2;
  Status = 1.0;
  memset(&RLinv[0], 0, 9U * sizeof(real_T));
  ACC_B.i_k = 1;
  do {
    exitg1 = 0;
    i = nA - 1;
    if (ACC_B.i_k - 1 <= i) {
      TL_tmp = (int16_T)ACC_B.i_k - 1;
      ACC_B.e_i = iC[TL_tmp];
      for (i = 0; i < 3; i++) {
        ACC_B.b_i = i + 3 * TL_tmp;
        RLinv[ACC_B.b_i] = 0.0;
        ACC_B.c_i = 3 * TL_tmp + i;
        RLinv[ACC_B.b_i] = RLinv[ACC_B.c_i] + b_Ac[ACC_B.e_i - 1] * b_Linv[i];
        RLinv[ACC_B.b_i] = RLinv[ACC_B.c_i] + b_Linv[i + 3] * b_Ac[ACC_B.e_i +
          97];
        RLinv[ACC_B.b_i] = RLinv[ACC_B.c_i] + b_Linv[i + 6] * b_Ac[ACC_B.e_i +
          195];
      }

      ACC_B.i_k++;
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  ACC_qr(RLinv, ACC_B.QQ, ACC_B.RR);
  ACC_B.b_i = 1;
  do {
    exitg1 = 0;
    if (ACC_B.b_i - 1 <= i) {
      if (fabs(ACC_B.RR[(((int16_T)ACC_B.b_i - 1) * 3 + (int16_T)ACC_B.b_i) - 1])
          < 1.0E-12) {
        Status = -2.0;
        exitg1 = 1;
      } else {
        ACC_B.b_i++;
      }
    } else {
      ACC_B.c_i = 1;
      do {
        exitg2 = 0;
        ACC_B.b_i = n - 1;
        if (ACC_B.c_i - 1 <= ACC_B.b_i) {
          ACC_B.i_k = 1;
          while (ACC_B.i_k - 1 <= ACC_B.b_i) {
            TL_tmp = ((int16_T)ACC_B.c_i - 1) * 3;
            ACC_B.e_i = ((int16_T)ACC_B.i_k - 1) * 3;
            ACC_B.TL[((int16_T)ACC_B.c_i + 3 * ((int16_T)ACC_B.i_k - 1)) - 1] =
              (b_Linv[TL_tmp + 1] * ACC_B.QQ[ACC_B.e_i + 1] + b_Linv[TL_tmp] *
               ACC_B.QQ[ACC_B.e_i]) + b_Linv[TL_tmp + 2] * ACC_B.QQ[ACC_B.e_i +
              2];
            ACC_B.i_k++;
          }

          ACC_B.c_i++;
        } else {
          exitg2 = 1;
        }
      } while (exitg2 == 0);

      memset(&RLinv[0], 0, 9U * sizeof(real_T));
      b_j = nA;
      while (b_j > 0) {
        ACC_B.c_i = b_j - 1;
        RLinv[(b_j + 3 * ACC_B.c_i) - 1] = 1.0;
        for (c_k = b_j; c_k <= nA; c_k++) {
          ACC_B.i_k = c_k - 1;
          RLinv[(b_j + 3 * ACC_B.i_k) - 1] /= ACC_B.RR[((b_j - 1) * 3 + b_j) - 1];
        }

        if (b_j > 1) {
          ACC_B.e_i = 1;
          while (ACC_B.e_i - 1 <= b_j - 2) {
            for (c_k = b_j; c_k <= nA; c_k++) {
              ACC_B.i_k = c_k - 1;
              TL_tmp = ACC_B.i_k * 3;
              RLinv[((int16_T)ACC_B.e_i + 3 * ACC_B.i_k) - 1] = RLinv[(TL_tmp +
                (int16_T)ACC_B.e_i) - 1] - ACC_B.RR[(ACC_B.c_i * 3 + (int16_T)
                ACC_B.e_i) - 1] * RLinv[(TL_tmp + b_j) - 1];
            }

            ACC_B.e_i++;
          }
        }

        b_j = (int16_T)ACC_B.c_i;
      }

      ACC_B.e_i = 1;
      while (ACC_B.e_i - 1 <= ACC_B.b_i) {
        for (b_j = (int16_T)ACC_B.e_i; b_j <= n; b_j++) {
          ACC_B.c_i = b_j - 1;
          ACC_B.i_k = ((int16_T)ACC_B.e_i + 3 * ACC_B.c_i) - 1;
          b_H[ACC_B.i_k] = 0.0;
          TL_tmp = nA + 1;
          if (TL_tmp > 32767) {
            TL_tmp = 32767;
          }

          for (c_k = (int16_T)TL_tmp; c_k <= n; c_k++) {
            TL_tmp = (c_k - 1) * 3;
            b_H[ACC_B.i_k] = b_H[((b_j - 1) * 3 + (int16_T)ACC_B.e_i) - 1] -
              ACC_B.TL[(TL_tmp + (int16_T)ACC_B.e_i) - 1] * ACC_B.TL[(TL_tmp +
              b_j) - 1];
          }

          b_H[(b_j + 3 * ((int16_T)ACC_B.e_i - 1)) - 1] = b_H[(ACC_B.c_i * 3 +
            (int16_T)ACC_B.e_i) - 1];
        }

        ACC_B.e_i++;
      }

      ACC_B.e_i = 1;
      while (ACC_B.e_i - 1 <= i) {
        ACC_B.f_i = 1;
        while (ACC_B.f_i - 1 <= ACC_B.b_i) {
          ACC_B.c_i = (int16_T)ACC_B.e_i - 1;
          ACC_B.i_k = ((int16_T)ACC_B.f_i + 3 * ACC_B.c_i) - 1;
          D[ACC_B.i_k] = 0.0;
          for (b_j = (int16_T)ACC_B.e_i; b_j <= nA; b_j++) {
            TL_tmp = (b_j - 1) * 3;
            D[ACC_B.i_k] = ACC_B.TL[(TL_tmp + (int16_T)ACC_B.f_i) - 1] * RLinv
              [(TL_tmp + (int16_T)ACC_B.e_i) - 1] + D[(ACC_B.c_i * 3 + (int16_T)
              ACC_B.f_i) - 1];
          }

          ACC_B.f_i++;
        }

        ACC_B.e_i++;
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return Status;
}

// Function for MATLAB Function: '<S20>/optimizer'
static void ACC_DropConstraint(int16_T kDrop, int16_T iA[98], int16_T *nA,
  int16_T iC[98])
{
  int16_T i;
  int32_T tmp;
  iA[iC[kDrop - 1] - 1] = 0;
  if (kDrop < *nA) {
    tmp = *nA - 1;
    if (tmp < -32768) {
      tmp = -32768;
    }

    for (i = kDrop; i <= (int16_T)tmp; i++) {
      iC[i - 1] = iC[i];
    }
  }

  iC[*nA - 1] = 0;
  tmp = *nA - 1;
  if (tmp < -32768) {
    tmp = -32768;
  }

  *nA = (int16_T)tmp;
}

// Function for MATLAB Function: '<S20>/optimizer'
static int16_T ACC_rdivide_helper(int16_T x, int16_T y)
{
  int16_T z;
  uint16_T b_y;
  uint16_T c_y;
  uint16_T q;
  int32_T tmp;
  switch (y) {
   case 0:
    if (x == 0) {
      z = 0;
    } else if (x < 0) {
      z = MIN_int16_T;
    } else {
      z = MAX_int16_T;
    }
    break;

   case 1:
    z = x;
    break;

   case -1:
    tmp = -x;
    if (tmp > 32767) {
      tmp = 32767;
    }

    z = (int16_T)tmp;
    break;

   default:
    if (x >= 0) {
      b_y = (uint16_T)x;
    } else if (x == -32768) {
      b_y = 32768U;
    } else {
      b_y = (uint16_T)-x;
    }

    if (y >= 0) {
      c_y = (uint16_T)y;
    } else if (y == -32768) {
      c_y = 32768U;
    } else {
      c_y = (uint16_T)-y;
    }

    q = (uint16_T)(c_y == 0U ? MAX_uint32_T : (uint32_T)b_y / c_y);
    b_y = (uint16_T)((uint32_T)b_y - (uint16_T)((uint32_T)q * c_y));
    if ((b_y > 0) && (b_y >= (int32_T)((uint32_T)c_y >> 1) + (c_y & 1))) {
      q++;
    }

    z = (int16_T)q;
    if ((x < 0) != (y < 0)) {
      z = (int16_T)-(int16_T)q;
    }
    break;
  }

  return z;
}

// Function for MATLAB Function: '<S20>/optimizer'
static void ACC_qpkwik(const real_T b_Linv[9], const real_T b_Hinv[9], const
  real_T f[3], const real_T b_Ac[294], const real_T b[98], int16_T iA[98],
  int16_T b_maxiter, real_T FeasTol, real_T x[3], real_T lambda[98], real_T
  *status)
{
  boolean_T cTolComputed;
  int16_T nA;
  boolean_T DualFeasible;
  boolean_T ColdReset;
  int16_T kDrop;
  int16_T kNext;
  int16_T tmp;
  int32_T exitg1;
  int32_T exitg2;
  int32_T exitg3;
  boolean_T exitg4;
  int32_T exitg5;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  *status = 1.0;
  x[0] = 0.0;
  ACC_B.r[0] = 0.0;
  x[1] = 0.0;
  ACC_B.r[1] = 0.0;
  x[2] = 0.0;
  ACC_B.r[2] = 0.0;
  ACC_B.rMin = 0.0;
  cTolComputed = false;
  for (ACC_B.i_c = 0; ACC_B.i_c < 98; ACC_B.i_c++) {
    lambda[ACC_B.i_c] = 0.0;
    ACC_B.cTol[ACC_B.i_c] = 1.0;
    ACC_B.iC[ACC_B.i_c] = 0;
  }

  nA = 0;
  for (ACC_B.i_c = 0; ACC_B.i_c < 98; ACC_B.i_c++) {
    if (iA[ACC_B.i_c] == 1) {
      ACC_B.idx = nA + 1;
      if (ACC_B.idx > 32767) {
        ACC_B.idx = 32767;
      }

      nA = (int16_T)ACC_B.idx;
      ACC_B.iC[(int16_T)ACC_B.idx - 1] = (int16_T)(1 + ACC_B.i_c);
    }
  }

  guard1 = false;
  if (nA > 0) {
    for (ACC_B.i_c = 0; ACC_B.i_c < 6; ACC_B.i_c++) {
      ACC_B.Opt[ACC_B.i_c] = 0.0;
    }

    ACC_B.Rhs[0] = f[0];
    ACC_B.Rhs[3] = 0.0;
    ACC_B.Rhs[1] = f[1];
    ACC_B.Rhs[4] = 0.0;
    ACC_B.Rhs[2] = f[2];
    ACC_B.Rhs[5] = 0.0;
    DualFeasible = false;
    ACC_B.idx = 3 * nA;
    if (ACC_B.idx > 32767) {
      ACC_B.idx = 32767;
    }

    ColdReset = false;
    do {
      exitg3 = 0;
      if ((!DualFeasible) && (nA > 0) && ((int32_T)*status <= b_maxiter)) {
        ACC_B.Xnorm0 = ACC_KWIKfactor(b_Ac, ACC_B.iC, nA, b_Linv, ACC_B.RLinv,
          ACC_B.D, ACC_B.b_H, 3);
        if (ACC_B.Xnorm0 < 0.0) {
          if (ColdReset) {
            *status = -2.0;
            exitg3 = 2;
          } else {
            nA = 0;
            memset(&iA[0], 0, 98U * sizeof(int16_T));
            memset(&ACC_B.iC[0], 0, 98U * sizeof(int16_T));
            ColdReset = true;
          }
        } else {
          ACC_B.j = 1;
          do {
            exitg5 = 0;
            ACC_B.i_c = nA - 1;
            if (ACC_B.j - 1 <= ACC_B.i_c) {
              ACC_B.e_k = 3 + (int16_T)ACC_B.j;
              if (ACC_B.e_k > 32767) {
                ACC_B.e_k = 32767;
              }

              ACC_B.b_k = (int16_T)ACC_B.j - 1;
              ACC_B.Rhs[ACC_B.e_k - 1] = b[ACC_B.iC[ACC_B.b_k] - 1];
              for (kNext = (int16_T)ACC_B.j; kNext <= nA; kNext++) {
                ACC_B.e_k = (kNext + 3 * ACC_B.b_k) - 1;
                ACC_B.U[ACC_B.e_k] = 0.0;
                ACC_B.Opt_tmp = 1;
                while (ACC_B.Opt_tmp - 1 <= ACC_B.i_c) {
                  ACC_B.U_tmp = ((int16_T)ACC_B.Opt_tmp - 1) * 3;
                  ACC_B.U[ACC_B.e_k] = ACC_B.RLinv[(ACC_B.U_tmp + kNext) - 1] *
                    ACC_B.RLinv[(ACC_B.U_tmp + (int16_T)ACC_B.j) - 1] + ACC_B.U
                    [(((int16_T)ACC_B.j - 1) * 3 + kNext) - 1];
                  ACC_B.Opt_tmp++;
                }

                ACC_B.U[((int16_T)ACC_B.j + 3 * (kNext - 1)) - 1] = ACC_B.U
                  [(ACC_B.b_k * 3 + kNext) - 1];
              }

              ACC_B.j++;
            } else {
              exitg5 = 1;
            }
          } while (exitg5 == 0);

          for (ACC_B.j = 0; ACC_B.j < 3; ACC_B.j++) {
            ACC_B.e_k = 1 + ACC_B.j;
            ACC_B.Opt[ACC_B.j] = (ACC_B.b_H[ACC_B.e_k - 1] * ACC_B.Rhs[0] +
                                  ACC_B.b_H[ACC_B.e_k + 2] * ACC_B.Rhs[1]) +
              ACC_B.b_H[ACC_B.e_k + 5] * ACC_B.Rhs[2];
            ACC_B.b_k = 1;
            while (ACC_B.b_k - 1 <= ACC_B.i_c) {
              ACC_B.e_k = 3 + (int16_T)ACC_B.b_k;
              if (ACC_B.e_k > 32767) {
                ACC_B.e_k = 32767;
              }

              ACC_B.Opt[ACC_B.j] += ACC_B.D[((int16_T)ACC_B.b_k - 1) * 3 +
                ACC_B.j] * ACC_B.Rhs[ACC_B.e_k - 1];
              ACC_B.b_k++;
            }
          }

          ACC_B.b_k = 1;
          while (ACC_B.b_k - 1 <= ACC_B.i_c) {
            ACC_B.e_k = 3 + (int16_T)ACC_B.b_k;
            ACC_B.j = ACC_B.e_k;
            if (ACC_B.e_k > 32767) {
              ACC_B.j = 32767;
            }

            ACC_B.Opt_tmp = ((int16_T)ACC_B.b_k - 1) * 3;
            ACC_B.Opt[ACC_B.j - 1] = (ACC_B.D[ACC_B.Opt_tmp + 1] * ACC_B.Rhs[1]
              + ACC_B.D[ACC_B.Opt_tmp] * ACC_B.Rhs[0]) + ACC_B.D[ACC_B.Opt_tmp +
              2] * ACC_B.Rhs[2];
            ACC_B.Opt_tmp = 1;
            while (ACC_B.Opt_tmp - 1 <= ACC_B.i_c) {
              ACC_B.j = ACC_B.e_k;
              if (ACC_B.e_k > 32767) {
                ACC_B.j = 32767;
              }

              ACC_B.U_tmp = ACC_B.e_k;
              if (ACC_B.e_k > 32767) {
                ACC_B.U_tmp = 32767;
              }

              ACC_B.i0 = 3 + (int16_T)ACC_B.Opt_tmp;
              if (ACC_B.i0 > 32767) {
                ACC_B.i0 = 32767;
              }

              ACC_B.Opt[ACC_B.j - 1] = ACC_B.U[(((int16_T)ACC_B.Opt_tmp - 1) * 3
                + (int16_T)ACC_B.b_k) - 1] * ACC_B.Rhs[ACC_B.i0 - 1] +
                ACC_B.Opt[ACC_B.U_tmp - 1];
              ACC_B.Opt_tmp++;
            }

            ACC_B.b_k++;
          }

          ACC_B.Xnorm0 = -1.0E-12;
          kDrop = 0;
          ACC_B.b_k = 1;
          while (ACC_B.b_k - 1 <= ACC_B.i_c) {
            ACC_B.e_k = 3 + (int16_T)ACC_B.b_k;
            ACC_B.j = ACC_B.e_k;
            if (ACC_B.e_k > 32767) {
              ACC_B.j = 32767;
            }

            lambda[ACC_B.iC[(int16_T)ACC_B.b_k - 1] - 1] = ACC_B.Opt[ACC_B.j - 1];
            ACC_B.j = ACC_B.e_k;
            if (ACC_B.e_k > 32767) {
              ACC_B.j = 32767;
            }

            if ((ACC_B.Opt[ACC_B.j - 1] < ACC_B.Xnorm0) && ((int16_T)ACC_B.b_k <=
                 nA)) {
              kDrop = (int16_T)ACC_B.b_k;
              if (ACC_B.e_k > 32767) {
                ACC_B.e_k = 32767;
              }

              ACC_B.Xnorm0 = ACC_B.Opt[ACC_B.e_k - 1];
            }

            ACC_B.b_k++;
          }

          if (kDrop <= 0) {
            DualFeasible = true;
            x[0] = ACC_B.Opt[0];
            x[1] = ACC_B.Opt[1];
            x[2] = ACC_B.Opt[2];
          } else {
            (*status)++;
            if ((int16_T)ACC_B.idx > 50) {
              tmp = (int16_T)ACC_B.idx;
            } else {
              tmp = 50;
            }

            if ((int32_T)*status > ACC_rdivide_helper(tmp, 10)) {
              nA = 0;
              memset(&iA[0], 0, 98U * sizeof(int16_T));
              memset(&ACC_B.iC[0], 0, 98U * sizeof(int16_T));
              ColdReset = true;
            } else {
              lambda[ACC_B.iC[kDrop - 1] - 1] = 0.0;
              ACC_DropConstraint(kDrop, iA, &nA, ACC_B.iC);
            }
          }
        }
      } else {
        if (nA <= 0) {
          memset(&lambda[0], 0, 98U * sizeof(real_T));
          ACC_Unconstrained(b_Hinv, f, x, 3);
        }

        exitg3 = 1;
      }
    } while (exitg3 == 0);

    if (exitg3 == 1) {
      guard1 = true;
    }
  } else {
    ACC_Unconstrained(b_Hinv, f, x, 3);
    guard1 = true;
  }

  if (guard1) {
    ACC_B.Xnorm0 = ACC_norm(x);
    do {
      exitg2 = 0;
      if ((int32_T)*status <= b_maxiter) {
        ACC_B.cMin = -FeasTol;
        kNext = 0;
        for (ACC_B.i_c = 0; ACC_B.i_c < 98; ACC_B.i_c++) {
          if (!cTolComputed) {
            ACC_B.idx = 1 + ACC_B.i_c;
            ACC_B.b_Ac[0] = b_Ac[ACC_B.idx - 1] * x[0];
            ACC_B.b_Ac[1] = b_Ac[ACC_B.idx + 97] * x[1];
            ACC_B.b_Ac[2] = b_Ac[ACC_B.idx + 195] * x[2];
            ACC_abs(ACC_B.b_Ac, ACC_B.AcRow);
            if (!rtIsNaN(ACC_B.AcRow[0])) {
              ACC_B.idx = 1;
            } else {
              ACC_B.idx = 0;
              ACC_B.e_k = 2;
              exitg4 = false;
              while ((!exitg4) && (ACC_B.e_k < 4)) {
                if (!rtIsNaN(ACC_B.AcRow[ACC_B.e_k - 1])) {
                  ACC_B.idx = ACC_B.e_k;
                  exitg4 = true;
                } else {
                  ACC_B.e_k++;
                }
              }
            }

            if (ACC_B.idx == 0) {
              ACC_B.cVal = ACC_B.AcRow[0];
            } else {
              ACC_B.cVal = ACC_B.AcRow[ACC_B.idx - 1];
              while (ACC_B.idx + 1 < 4) {
                if (ACC_B.cVal < ACC_B.AcRow[ACC_B.idx]) {
                  ACC_B.cVal = ACC_B.AcRow[ACC_B.idx];
                }

                ACC_B.idx++;
              }
            }

            if ((!(ACC_B.cTol[ACC_B.i_c] > ACC_B.cVal)) && (!rtIsNaN(ACC_B.cVal)))
            {
              ACC_B.cTol[ACC_B.i_c] = ACC_B.cVal;
            }
          }

          if (iA[ACC_B.i_c] == 0) {
            ACC_B.idx = 1 + ACC_B.i_c;
            ACC_B.cVal = (((b_Ac[ACC_B.idx - 1] * x[0] + b_Ac[ACC_B.idx + 97] *
                            x[1]) + b_Ac[ACC_B.idx + 195] * x[2]) - b[ACC_B.i_c])
              / ACC_B.cTol[ACC_B.i_c];
            if (ACC_B.cVal < ACC_B.cMin) {
              ACC_B.cMin = ACC_B.cVal;
              kNext = (int16_T)(1 + ACC_B.i_c);
            }
          }
        }

        cTolComputed = true;
        if (kNext <= 0) {
          exitg2 = 1;
        } else {
          do {
            exitg1 = 0;
            if ((kNext > 0) && ((int32_T)*status <= b_maxiter)) {
              guard2 = false;
              if (nA == 0) {
                for (ACC_B.idx = 0; ACC_B.idx < 3; ACC_B.idx++) {
                  ACC_B.AcRow[ACC_B.idx] = b_Hinv[ACC_B.idx + 6] * b_Ac[kNext +
                    195] + (b_Hinv[ACC_B.idx + 3] * b_Ac[kNext + 97] +
                            b_Ac[kNext - 1] * b_Hinv[ACC_B.idx]);
                }

                guard2 = true;
              } else {
                ACC_B.cMin = ACC_KWIKfactor(b_Ac, ACC_B.iC, nA, b_Linv,
                  ACC_B.RLinv, ACC_B.D, ACC_B.b_H, 3);
                if (ACC_B.cMin <= 0.0) {
                  *status = -2.0;
                  exitg1 = 1;
                } else {
                  for (ACC_B.idx = 0; ACC_B.idx < 9; ACC_B.idx++) {
                    ACC_B.U[ACC_B.idx] = -ACC_B.b_H[ACC_B.idx];
                  }

                  for (ACC_B.idx = 0; ACC_B.idx < 3; ACC_B.idx++) {
                    ACC_B.i_c = kNext - 1;
                    ACC_B.AcRow[ACC_B.idx] = ACC_B.U[ACC_B.idx + 6] * b_Ac[kNext
                      + 195] + (ACC_B.U[ACC_B.idx + 3] * b_Ac[kNext + 97] +
                                b_Ac[ACC_B.i_c] * ACC_B.U[ACC_B.idx]);
                  }

                  ACC_B.idx = 1;
                  while (ACC_B.idx - 1 <= nA - 1) {
                    ACC_B.i_c = ((int16_T)ACC_B.idx - 1) * 3;
                    ACC_B.r[(int16_T)ACC_B.idx - 1] = (ACC_B.D[ACC_B.i_c + 1] *
                      b_Ac[kNext + 97] + ACC_B.D[ACC_B.i_c] * b_Ac[kNext - 1]) +
                      ACC_B.D[ACC_B.i_c + 2] * b_Ac[kNext + 195];
                    ACC_B.idx++;
                  }

                  guard2 = true;
                }
              }

              if (guard2) {
                kDrop = 0;
                ACC_B.cMin = 0.0;
                DualFeasible = true;
                ColdReset = true;
                if (nA > 0) {
                  ACC_B.idx = 0;
                  exitg4 = false;
                  while ((!exitg4) && (ACC_B.idx <= nA - 1)) {
                    if (ACC_B.r[ACC_B.idx] >= 1.0E-12) {
                      ColdReset = false;
                      exitg4 = true;
                    } else {
                      ACC_B.idx++;
                    }
                  }
                }

                if ((nA != 0) && (!ColdReset)) {
                  ACC_B.i_c = 1;
                  while (ACC_B.i_c - 1 <= nA - 1) {
                    ACC_B.idx = (int16_T)ACC_B.i_c - 1;
                    if (ACC_B.r[ACC_B.idx] > 1.0E-12) {
                      ACC_B.cVal = lambda[ACC_B.iC[ACC_B.idx] - 1] / ACC_B.r
                        [(int16_T)ACC_B.i_c - 1];
                      if ((kDrop == 0) || (ACC_B.cVal < ACC_B.rMin)) {
                        ACC_B.rMin = ACC_B.cVal;
                        kDrop = (int16_T)ACC_B.i_c;
                      }
                    }

                    ACC_B.i_c++;
                  }

                  if (kDrop > 0) {
                    ACC_B.cMin = ACC_B.rMin;
                    DualFeasible = false;
                  }
                }

                ACC_B.cVal = (b_Ac[kNext - 1] * ACC_B.AcRow[0] + b_Ac[kNext + 97]
                              * ACC_B.AcRow[1]) + b_Ac[kNext + 195] *
                  ACC_B.AcRow[2];
                if (ACC_B.cVal <= 0.0) {
                  ACC_B.cVal = 0.0;
                  ColdReset = true;
                } else {
                  ACC_B.cVal = (b[kNext - 1] - ((b_Ac[kNext - 1] * x[0] +
                    b_Ac[kNext + 97] * x[1]) + b_Ac[kNext + 195] * x[2])) /
                    ACC_B.cVal;
                  ColdReset = false;
                }

                if (DualFeasible && ColdReset) {
                  *status = -1.0;
                  exitg1 = 1;
                } else {
                  if (ColdReset) {
                    ACC_B.t = ACC_B.cMin;
                  } else if (DualFeasible) {
                    ACC_B.t = ACC_B.cVal;
                  } else if ((ACC_B.cMin < ACC_B.cVal) || rtIsNaN(ACC_B.cVal)) {
                    ACC_B.t = ACC_B.cMin;
                  } else {
                    ACC_B.t = ACC_B.cVal;
                  }

                  ACC_B.idx = 1;
                  while (ACC_B.idx - 1 <= nA - 1) {
                    ACC_B.i_c = (int16_T)ACC_B.idx - 1;
                    ACC_B.e_k = ACC_B.iC[ACC_B.i_c] - 1;
                    lambda[ACC_B.e_k] -= ACC_B.r[ACC_B.i_c] * ACC_B.t;
                    if ((ACC_B.iC[(int16_T)ACC_B.idx - 1] <= 98) &&
                        (lambda[ACC_B.e_k] < 0.0)) {
                      lambda[ACC_B.e_k] = 0.0;
                    }

                    ACC_B.idx++;
                  }

                  lambda[kNext - 1] += ACC_B.t;
                  if (ACC_B.t == ACC_B.cMin) {
                    ACC_DropConstraint(kDrop, iA, &nA, ACC_B.iC);
                  }

                  if (!ColdReset) {
                    x[0] += ACC_B.t * ACC_B.AcRow[0];
                    x[1] += ACC_B.t * ACC_B.AcRow[1];
                    x[2] += ACC_B.t * ACC_B.AcRow[2];
                    if (ACC_B.t == ACC_B.cVal) {
                      if (nA == 3) {
                        *status = -1.0;
                        exitg1 = 1;
                      } else {
                        ACC_B.idx = nA + 1;
                        if (ACC_B.idx > 32767) {
                          ACC_B.idx = 32767;
                        }

                        nA = (int16_T)ACC_B.idx;
                        ACC_B.iC[(int16_T)ACC_B.idx - 1] = kNext;
                        kDrop = (int16_T)ACC_B.idx;
                        exitg4 = false;
                        while ((!exitg4) && (kDrop > 1)) {
                          ACC_B.idx = kDrop - 1;
                          tmp = ACC_B.iC[ACC_B.idx];
                          ACC_B.i_c = kDrop - 2;
                          if (ACC_B.iC[ACC_B.idx] > ACC_B.iC[ACC_B.i_c]) {
                            exitg4 = true;
                          } else {
                            ACC_B.iC[ACC_B.idx] = ACC_B.iC[ACC_B.i_c];
                            ACC_B.iC[ACC_B.i_c] = tmp;
                            kDrop = (int16_T)ACC_B.idx;
                          }
                        }

                        iA[kNext - 1] = 1;
                        kNext = 0;
                        (*status)++;
                      }
                    } else {
                      (*status)++;
                    }
                  } else {
                    (*status)++;
                  }
                }
              }
            } else {
              ACC_B.cMin = ACC_norm(x);
              if (fabs(ACC_B.cMin - ACC_B.Xnorm0) > 0.001) {
                ACC_B.Xnorm0 = ACC_B.cMin;
                ACC_abs_h(b, ACC_B.varargin_1);
                for (ACC_B.idx = 0; ACC_B.idx < 98; ACC_B.idx++) {
                  if (ACC_B.varargin_1[ACC_B.idx] > 1.0) {
                    ACC_B.cTol[ACC_B.idx] = ACC_B.varargin_1[ACC_B.idx];
                  } else {
                    ACC_B.cTol[ACC_B.idx] = 1.0;
                  }
                }

                cTolComputed = false;
              }

              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = 1;
          }
        }
      } else {
        *status = 0.0;
        exitg2 = 1;
      }
    } while (exitg2 == 0);
  }
}

// Function for Chart: '<S6>/ACC Diagnostics'
static void ACC_Standard(const real_T *set_velocity, const real_T *Merge1, const
  real_T *safe_distance, real_T *umin_scale1, real_T *Switch1)
{
  boolean_T hoisted_cond;
  if (((ACC_B.In1.ObjDx < 1.4 * ACC_B.In1_p.VehSpd + 9.9) && (ACC_B.In1_p.VehSpd
        > 0.0)) || ((ACC_B.In1.ObjVx < -20.0) && (ACC_B.In1.ObjDx < 255.0))) {
    ACC_DW.is_ACC_Normal = ACC_IN_NO_ACTIVE_CHILD;
    ACC_DW.is_Standard = ACC_IN_NO_ACTIVE_CHILD;
    ACC_DW.durationLastReferenceTick_1 = ACC_DW.chartAbsoluteTimeCounter;
    ACC_DW.is_ACC = ACC_IN_FullBrakes;
    ACC_DW.condWasTrueAtLastTimeStep_1 = (ACC_B.In1.ObjDx > *safe_distance);
  } else if (ACC_DW.is_Standard == ACC_IN_ACC_Normal) {
    if (ACC_B.In1.ObjDx < *safe_distance) {
      ACC_DW.is_ACC_Normal = ACC_IN_NO_ACTIVE_CHILD;
      ACC_DW.is_Standard = ACC_IN_HalfBrakes;
    } else {
      ACC_B.RateLimiter = false;
      switch (ACC_DW.is_ACC_Normal) {
       case ACC_IN_ACC_Normal:
        if (((ACC_B.In1_p.VehSpd > 2.0) && (ACC_B.In1.ObjVx < 0.0) &&
             (ACC_B.In1_p.VehSpd > 0.0) && (ACC_B.In1.ObjDx < 1.5 * *Merge1)) ||
            ((ACC_B.In1_p.VehSpd + ACC_B.In1.ObjVx < 1.0) && (ACC_B.In1.ObjDx <
              2.0 * *safe_distance))) {
          ACC_DW.durationLastReferenceTick_1_d = ACC_DW.chartAbsoluteTimeCounter;
          ACC_DW.is_ACC_Normal = ACC_IN_ACC_Zeroed;
          ACC_DW.condWasTrueAtLastTimeStep_1_c0 = ((ACC_B.In1.ObjVx > 0.0) &&
            (ACC_B.In1.ObjDx > 1.5 * *Merge1));
        } else if ((*set_velocity < 1.1 * ACC_B.In1_p.VehSpd) && (*umin_scale1 >
                    0.0)) {
          ACC_DW.durationLastReferenceTick_1_n = ACC_DW.chartAbsoluteTimeCounter;
          ACC_DW.is_ACC_Normal = ACC_IN_ACC_OverSpd;
          ACC_DW.condWasTrueAtLastTimeStep_1_d = (*set_velocity >=
            ACC_B.In1_p.VehSpd);
        } else {
          ACC_B.ACC_Accel_Out = *umin_scale1;
        }
        break;

       case ACC_IN_ACC_OverSpd:
        hoisted_cond = (*set_velocity >= ACC_B.In1_p.VehSpd);
        if ((!hoisted_cond) || (!ACC_DW.condWasTrueAtLastTimeStep_1_d)) {
          ACC_DW.durationLastReferenceTick_1_n = ACC_DW.chartAbsoluteTimeCounter;
        }

        ACC_DW.condWasTrueAtLastTimeStep_1_d = hoisted_cond;
        if ((ACC_DW.chartAbsoluteTimeCounter -
             ACC_DW.durationLastReferenceTick_1_n > 13) || (*umin_scale1 < 0.0))
        {
          ACC_DW.is_ACC_Normal = ACC_IN_ACC_Normal;
        } else {
          ACC_B.ACC_Accel_Out = *umin_scale1 / 2.0;
        }
        break;

       default:
        hoisted_cond = ((ACC_B.In1.ObjVx > 0.0) && (ACC_B.In1.ObjDx > 1.5 *
          *Merge1));
        if ((!hoisted_cond) || (!ACC_DW.condWasTrueAtLastTimeStep_1_c0)) {
          ACC_DW.durationLastReferenceTick_1_d = ACC_DW.chartAbsoluteTimeCounter;
        }

        ACC_DW.condWasTrueAtLastTimeStep_1_c0 = hoisted_cond;
        if ((ACC_DW.chartAbsoluteTimeCounter -
             ACC_DW.durationLastReferenceTick_1_d > 18) || (ACC_B.In1_p.VehSpd <
             2.0)) {
          ACC_DW.is_ACC_Normal = ACC_IN_ACC_Normal;
        } else if ((0.0 < *umin_scale1) || rtIsNaN(*umin_scale1)) {
          ACC_B.ACC_Accel_Out = 0.0;
        } else {
          ACC_B.ACC_Accel_Out = *umin_scale1;
        }
        break;
      }
    }
  } else if (ACC_B.In1.ObjDx >= 2.0 * *safe_distance) {
    ACC_DW.is_Standard = ACC_IN_ACC_Normal;
    ACC_DW.is_ACC_Normal = ACC_IN_ACC_Normal;
  } else {
    ACC_B.ACC_Accel_Out = 0.5 * *Switch1;
    if ((!(ACC_B.ACC_Accel_Out < *umin_scale1)) && (!rtIsNaN(*umin_scale1))) {
      ACC_B.ACC_Accel_Out = *umin_scale1;
    }

    ACC_B.RateLimiter = false;
  }
}

// Function for MATLAB Function: '<S20>/optimizer'
static real_T ACC_mod(real_T x)
{
  real_T r;
  if ((!rtIsInf(x)) && (!rtIsNaN(x))) {
    if (x == 0.0) {
      r = 0.0;
    } else {
      r = fmod(x, ACC_ny);
      if (r == 0.0) {
        r = 0.0;
      } else {
        if (x < 0.0) {
          r += ACC_ny;
        }
      }
    }
  } else {
    r = (rtNaN);
  }

  return r;
}

static void matlabCodegenHandle_matlabCod_e(robotics_slros_internal_blo_e_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCodeg(robotics_slros_internal_block_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

// Model step function
void ACC_step(void)
{
  static const real_T b_Ac[294] = { -0.0012143788966212743,
    -0.0046149037073693706, -0.0098783391806144955, -0.016729242404305393,
    -0.024932896411722826, -0.034289288597511992, -0.044627979462304859,
    -0.055803730045320452, -0.067692775868213345, -0.080189651799466827,
    -0.093204486382306453, -0.10666069621303653, -0.12049302121985987,
    -0.134645850437927, -0.1490717953289431, -0.16373047404433194,
    -0.17858747544264236, -0.19361347628341555, -0.20878348894942211,
    -0.224076220397839, -0.23947352589447635, -0.2549599435167918,
    -0.27052229748352541, -0.28614936013451181, -0.30183156388887633,
    -0.31756075579200127, -0.33332998835424749, -0.34913334131546997,
    -0.36496576976275708, -0.38082297470490539, -3.2810551689427126E-5,
    0.0012143788966212743, -0.00025254814631542978, 0.0046149037073693706,
    -0.00082083040969290312, 0.0098783391806144955, -0.0018753787978474752,
    0.016729242404305393, -0.0035335517941387689, 0.024932896411722826,
    -0.0058953557012441813, 0.034289288597511992, -0.009046010268847729,
    0.044627979462304859, -0.013058134977339906, 0.055803730045320452,
    -0.017993612065893422, 0.067692775868213345, -0.023905174100266623,
    0.080189651799466827, -0.030837756808846737, 0.093204486382306453,
    -0.038829651893481615, 0.10666069621303653, -0.047913489390069848,
    0.12049302121985987, -0.058117074781036164, 0.134645850437927,
    -0.069464102335528008, 0.1490717953289431, -0.08197476297783346,
    0.16373047404433194, -0.0956662622786781, 0.17858747544264236,
    -0.11055326185829134, 0.19361347628341555, -0.12664825552528788,
    0.20878348894942211, -0.14396188980107924, 0.224076220397839,
    -0.16250323705276037, 0.23947352589447635, -0.18228002824160244,
    0.2549599435167918, -0.20329885125823541, 0.27052229748352541,
    -0.22556531993274198, 0.28614936013451181, -0.24908421805555947,
    0.30183156388887633, -0.27385962210399672, 0.31756075579200127,
    -0.29989500582287332, 0.33332998835424749, -0.32719332934226181,
    0.34913334131546997, -0.35575711511861791, 0.36496576976275708,
    -0.3855885126475434, 0.38082297470490539, -1.0, -1.0, 1.0, 1.0, -1.0, -0.0,
    1.0, 0.0, -0.0, -0.0012143788966212743, -0.0046149037073693706,
    -0.0098783391806144955, -0.016729242404305393, -0.024932896411722826,
    -0.034289288597511992, -0.044627979462304859, -0.055803730045320452,
    -0.067692775868213345, -0.080189651799466827, -0.093204486382306453,
    -0.10666069621303653, -0.12049302121985987, -0.134645850437927,
    -0.1490717953289431, -0.16373047404433194, -0.17858747544264236,
    -0.19361347628341555, -0.20878348894942211, -0.224076220397839,
    -0.23947352589447635, -0.2549599435167918, -0.27052229748352541,
    -0.28614936013451181, -0.30183156388887633, -0.31756075579200127,
    -0.33332998835424749, -0.34913334131546997, -0.36496576976275708, 0.0, 0.0,
    -3.2810551689427126E-5, 0.0012143788966212743, -0.00025254814631542978,
    0.0046149037073693706, -0.00082083040969290312, 0.0098783391806144955,
    -0.0018753787978474752, 0.016729242404305393, -0.0035335517941387689,
    0.024932896411722826, -0.0058953557012441813, 0.034289288597511992,
    -0.009046010268847729, 0.044627979462304859, -0.013058134977339906,
    0.055803730045320452, -0.017993612065893422, 0.067692775868213345,
    -0.023905174100266623, 0.080189651799466827, -0.030837756808846737,
    0.093204486382306453, -0.038829651893481615, 0.10666069621303653,
    -0.047913489390069848, 0.12049302121985987, -0.058117074781036164,
    0.134645850437927, -0.069464102335528008, 0.1490717953289431,
    -0.08197476297783346, 0.16373047404433194, -0.0956662622786781,
    0.17858747544264236, -0.11055326185829134, 0.19361347628341555,
    -0.12664825552528788, 0.20878348894942211, -0.14396188980107924,
    0.224076220397839, -0.16250323705276037, 0.23947352589447635,
    -0.18228002824160244, 0.2549599435167918, -0.20329885125823541,
    0.27052229748352541, -0.22556531993274198, 0.28614936013451181,
    -0.24908421805555947, 0.30183156388887633, -0.27385962210399672,
    0.31756075579200127, -0.29989500582287332, 0.33332998835424749,
    -0.32719332934226181, 0.34913334131546997, -0.35575711511861791,
    0.36496576976275708, -0.0, -1.0, 0.0, 1.0, -0.0, -1.0, 0.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 2.0, 2.0, 2.0, 2.0 };

  static const real_T b_Hinv[9] = { 0.00178550137182074, -0.00067172895677734931,
    0.0, -0.00067172895677734931, 0.0018674590069849139, 0.0, 0.0, 0.0,
    1.0000000000000001E-11 };

  static const real_T b_Linv[9] = { 0.039292226260900495, -0.015544205585456649,
    0.0, 0.0, 0.043214106573952374, 0.0, 0.0, 0.0, 3.1622776601683796E-6 };

  static const uint8_T b_Mrows[98] = { 2U, 4U, 6U, 8U, 10U, 12U, 14U, 16U, 18U,
    20U, 22U, 24U, 26U, 28U, 30U, 32U, 34U, 36U, 38U, 40U, 42U, 44U, 46U, 48U,
    50U, 52U, 54U, 56U, 58U, 60U, 61U, 62U, 63U, 64U, 65U, 66U, 67U, 68U, 69U,
    70U, 71U, 72U, 73U, 74U, 75U, 76U, 77U, 78U, 79U, 80U, 81U, 82U, 83U, 84U,
    85U, 86U, 87U, 88U, 89U, 90U, 91U, 92U, 93U, 94U, 95U, 96U, 97U, 98U, 99U,
    100U, 101U, 102U, 103U, 104U, 105U, 106U, 107U, 108U, 109U, 110U, 111U, 112U,
    113U, 114U, 115U, 116U, 117U, 118U, 119U, 120U, 121U, 122U, 151U, 152U, 181U,
    182U, 183U, 184U };

  static const real_T b_Mlim[98] = { 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6,
    0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6,
    0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.77, 0.4, 0.77, 0.4, 0.77, 0.4, 0.77, 0.4,
    0.77, 0.4, 0.77, 0.4, 0.77, 0.4, 0.77, 0.4, 0.77, 0.4, 0.77, 0.4, 0.77, 0.4,
    0.77, 0.4, 0.77, 0.4, 0.77, 0.4, 0.77, 0.4, 0.77, 0.4, 0.77, 0.4, 0.77, 0.4,
    0.77, 0.4, 0.77, 0.4, 0.77, 0.4, 0.77, 0.4, 0.77, 0.4, 0.77, 0.4, 0.77, 0.4,
    0.77, 0.4, 0.77, 0.4, 0.77, 0.4, 0.77, 0.4, 0.77, 0.4, 0.5, 0.5, 0.5, 0.5,
    0.2, 0.2, 0.2, 0.2 };

  static const real_T a[8] = { -0.095314511416494357, 0.19429596929466661,
    0.95536137489539175, 0.0029638232952478023, 0.01584142735131373,
    -0.18358838289691443, -0.082702092615874287, 0.068929470890853883 };

  static const real_T b_Mx[392] = { -0.0053894305341981369,
    -0.0045925697557817873, -0.0039135297927835148, -0.00333489010585469,
    -0.0028418058905889387, -0.0024216272391129503, -0.0020635746110014884,
    -0.0017584622878332777, -0.0014984627167084355, -0.0012769056970405226,
    -0.0010881072588286456, -0.00092722384233987376, -0.00079012803823130275,
    -0.00067330270026685543, -0.00057375071412657313, -0.00048891810745788121,
    -0.00041662852858334175, -0.00035502741293842, -0.00030253440484821054,
    -0.00025780281403998549, -0.00021968506676217842, -0.00018720326517001143,
    -0.00015952409968881352, -0.00013593747074024278, -0.00011583827137906757,
    -9.8710863480248466E-5, -8.4115849218178949E-5, -7.1678798464883147E-5,
    -6.1080642912404579E-5, -5.2049490483862194E-5, -0.00046756239306928874,
    0.0053894305341981369, -0.000865992782277461, 0.0045925697557817873,
    -0.0012055127637765947, 0.0039135297927835148, -0.0014948326072410038,
    0.00333489010585469, -0.0017413747148738762, 0.0028418058905889387,
    -0.0019514640406118671, 0.0024216272391129503, -0.0021304903546675944,
    0.0020635746110014884, -0.0022830465162516963, 0.0017584622878332777,
    -0.0024130463018141139, 0.0014984627167084355, -0.0025238248116480674,
    0.0012769056970405226, -0.0026182240307540028, 0.0010881072588286456,
    -0.0026986657389983853, 0.00092722384233987376, -0.0027672136410526673,
    0.00079012803823130275, -0.0028256263100348879, 0.00067330270026685543,
    -0.0028754023031050254, 0.00057375071412657313, -0.0029178186064393679,
    0.00048891810745788121, -0.002953963395876634, 0.00041662852858334175,
    -0.0029847639536990911, 0.00035502741293842, -0.0030110104577441925,
    0.00030253440484821054, -0.0030333762531483018, 0.00025780281403998549,
    -0.0030524351267872019, 0.00021968506676217842, -0.0030686760275832822,
    0.00018720326517001143, -0.0030825156103238779, 0.00015952409968881352,
    -0.00309430892479816, 0.00013593747074024278, -0.0031043585244787441,
    0.00011583827137906757, -0.00311292222842815, 9.8710863480248466E-5,
    -0.0031202197355591809, 8.4115849218178949E-5, -0.0031264382609358255,
    7.1678798464883147E-5, -0.0031317373387120609, 6.1080642912404579E-5,
    -0.0031362529149263287, 5.2049490483862194E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.013116672012841245, 0.013515102371038637, 0.013854622326112126,
    0.014143942147058088, 0.014390484235502004, 0.014600573544888246,
    0.014779599845009933, 0.014932155994720231, 0.015062155770164459,
    0.015172934271376259, 0.01526733348313488, 0.015347775185118295,
    0.015416323081837333, 0.015474735746273159, 0.015524511735469116,
    0.0155669280355021, 0.015603072822126134, 0.015633873377551313,
    0.01566011987955359, 0.015682485673216919, 0.015701544545372426,
    0.015717785444904441, 0.015731625026567872, 0.015743418340124257,
    0.015753467939022658, 0.015762031642305531, 0.015769329148868579,
    0.015775547673761221, 0.015780846751125021, 0.01578536232698783,
    0.0010199704063584079, -0.013116672012841245, 0.0020856661928760297,
    -0.013515102371038637, 0.0031808171809556019, -0.013854622326112126,
    0.0043010682360989389, -0.014143942147058088, 0.0054427081574932989,
    -0.014390484235502004, 0.0066025744684164948, -0.014600573544888246,
    0.0077779722839719685, -0.014779599845009933, 0.0089666051747331367,
    -0.014932155994720231, 0.01016651625262734, -0.015062155770164459,
    0.011376037967637757, -0.015172934271376259, 0.012593749327374764,
    -0.01526733348313488, 0.013818439441999373, -0.015347775185118295,
    0.015049076459256171, -0.015416323081837333, 0.016284781092654577,
    -0.015474735746273159, 0.017524804063672916, -0.015524511735469116,
    0.018768506879272739, -0.0155669280355021, 0.020015345451577037,
    -0.015603072822126134, 0.021264856139480762, -0.015633873377551313,
    0.022516643854095943, -0.01566011987955359, 0.023770371922880593,
    -0.015682485673216919, 0.025025753452419159, -0.015701544545372426,
    0.02628254396826947, -0.015717785444904441, 0.027540535143054073,
    -0.015731625026567872, 0.028799549451892198, -0.015743418340124257,
    0.030059435618059314, -0.015753467939022658, 0.031320064732034192,
    -0.015762031642305531, 0.03258132694436898, -0.015769329148868579,
    0.033843128647538974, -0.015775547673761221, 0.035105390074473393,
    -0.015780846751125021, 0.036368043252158304, -0.01578536232698783, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.1751006590553847E-6, 5.3322988574456767E-6,
    5.466254325840642E-6, 5.5804036462314719E-6, 5.677675280617232E-6,
    5.7605646997016494E-6, 5.8311984033454526E-6, 5.8913884751972E-6,
    5.9426790710830963E-6, 5.9863860337996385E-6, 6.0236306506131174E-6,
    6.0553684195031507E-6, 6.0824135621384376E-6, 6.1054599124568028E-6,
    6.1250987167389372E-6, 6.1418338018306805E-6, 6.1560945006494305E-6,
    6.1682466665741462E-6, 6.178602059289379E-6, 6.1874263428739706E-6,
    6.194945901322657E-6, 6.2013536463504737E-6, 6.206813966477207E-6,
    6.2114669443589693E-6, 6.21543195056111E-6, 6.2188107059694773E-6,
    6.2216898914051527E-6, 6.2241433713914463E-6, 6.2262340891231189E-6,
    6.2280156812526445E-6, 0.028284675871819192, -5.1751006590553847E-6,
    0.028285096335406431, -5.3322988574456767E-6, 0.028285528420358665,
    -5.466254325840642E-6, 0.028285970408384903, -5.5804036462314719E-6,
    0.028286420835254143, -5.677675280617232E-6, 0.028286878453231034,
    -5.7605646997016494E-6, 0.028287342199065646, -5.8311984033454526E-6,
    0.028287811166716154, -5.8913884751972E-6, 0.028288284584104647,
    -5.9426790710830963E-6, 0.028288761793309723, -5.9863860337996385E-6,
    0.028289242233687749, -6.0236306506131174E-6, 0.028289725427489736,
    -6.0553684195031507E-6, 0.028290210967604854, -6.0824135621384376E-6,
    0.028290698507116126, -6.1054599124568028E-6, 0.028291187750400421,
    -6.1250987167389372E-6, 0.028291678445544307, -6.1418338018306805E-6,
    0.028292170377881332, -6.1560945006494305E-6, 0.028292663364484803,
    -6.1682466665741462E-6, 0.028293157249474881, -6.178602059289379E-6,
    0.028293651900019524, -6.1874263428739706E-6, 0.028294147202926733,
    -6.194945901322657E-6, 0.028294643061740652, -6.2013536463504737E-6,
    0.028295139394267022, -6.206813966477207E-6, 0.028295636130464513,
    -6.2114669443589693E-6, 0.028296133210647847, -6.21543195056111E-6,
    0.028296630583956577, -6.2188107059694773E-6, 0.028297128207050291,
    -6.2216898914051527E-6, 0.028297626042996731, -6.2241433713914463E-6,
    0.0282981240603243, -6.2262340891231189E-6, 0.028298622232214667,
    -6.2280156812526445E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
    1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
    1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T b_Mu1[98] = { -0.0012143788966212743,
    -0.0046149037073693706, -0.0098783391806144955, -0.016729242404305393,
    -0.024932896411722826, -0.034289288597511992, -0.044627979462304859,
    -0.055803730045320452, -0.067692775868213345, -0.080189651799466827,
    -0.093204486382306453, -0.10666069621303653, -0.12049302121985987,
    -0.134645850437927, -0.1490717953289431, -0.16373047404433194,
    -0.17858747544264236, -0.19361347628341555, -0.20878348894942211,
    -0.224076220397839, -0.23947352589447635, -0.2549599435167918,
    -0.27052229748352541, -0.28614936013451181, -0.30183156388887633,
    -0.31756075579200127, -0.33332998835424749, -0.34913334131546997,
    -0.36496576976275708, -0.38082297470490539, -3.2810551689427126E-5,
    0.0012143788966212743, -0.00025254814631542978, 0.0046149037073693706,
    -0.00082083040969290312, 0.0098783391806144955, -0.0018753787978474752,
    0.016729242404305393, -0.0035335517941387689, 0.024932896411722826,
    -0.0058953557012441813, 0.034289288597511992, -0.009046010268847729,
    0.044627979462304859, -0.013058134977339906, 0.055803730045320452,
    -0.017993612065893422, 0.067692775868213345, -0.023905174100266623,
    0.080189651799466827, -0.030837756808846737, 0.093204486382306453,
    -0.038829651893481615, 0.10666069621303653, -0.047913489390069848,
    0.12049302121985987, -0.058117074781036164, 0.134645850437927,
    -0.069464102335528008, 0.1490717953289431, -0.08197476297783346,
    0.16373047404433194, -0.0956662622786781, 0.17858747544264236,
    -0.11055326185829134, 0.19361347628341555, -0.12664825552528788,
    0.20878348894942211, -0.14396188980107924, 0.224076220397839,
    -0.16250323705276037, 0.23947352589447635, -0.18228002824160244,
    0.2549599435167918, -0.20329885125823541, 0.27052229748352541,
    -0.22556531993274198, 0.28614936013451181, -0.24908421805555947,
    0.30183156388887633, -0.27385962210399672, 0.31756075579200127,
    -0.29989500582287332, 0.33332998835424749, -0.32719332934226181,
    0.34913334131546997, -0.35575711511861791, 0.36496576976275708,
    -0.3855885126475434, 0.38082297470490539, -1.0, -1.0, 1.0, 1.0, 0.0, 0.0,
    0.0, 0.0 };

  static const real_T b_Mv[6076] = { 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 2.7105054312137611E-20, 2.7105054312137611E-20,
    2.3716922523120409E-20, 2.7105054312137611E-20, 2.3716922523120409E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 3.3881317890172014E-20, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -1.6940658945086007E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999974, -2.3716922523120409E-20,
    0.079999999999999974, -2.7105054312137611E-20, 0.079999999999999974,
    -3.0493186101154812E-20, 0.07999999999999996, -3.3881317890172014E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -3.0493186101154812E-20, 0.07999999999999996, -3.0493186101154812E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -2.7105054312137611E-20, 0.07999999999999996, -2.7105054312137611E-20,
    0.07999999999999996, -2.3716922523120409E-20, 0.07999999999999996,
    -2.7105054312137611E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.079999999999999946,
    -3.0493186101154812E-20, 0.079999999999999932, -3.0493186101154812E-20,
    0.079999999999999932, -3.3881317890172014E-20, 0.079999999999999932,
    -3.3881317890172014E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.6940658945086007E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 2.7105054312137611E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 2.7105054312137611E-20,
    2.7105054312137611E-20, 2.3716922523120409E-20, 2.7105054312137611E-20,
    2.3716922523120409E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -1.6940658945086007E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999974, -2.3716922523120409E-20,
    0.079999999999999974, -2.7105054312137611E-20, 0.079999999999999974,
    -3.0493186101154812E-20, 0.07999999999999996, -3.3881317890172014E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -3.0493186101154812E-20, 0.07999999999999996, -3.0493186101154812E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -2.7105054312137611E-20, 0.07999999999999996, -2.7105054312137611E-20,
    0.07999999999999996, -2.3716922523120409E-20, 0.07999999999999996,
    -2.7105054312137611E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.079999999999999946,
    -3.0493186101154812E-20, 0.079999999999999932, -3.0493186101154812E-20,
    0.079999999999999932, -3.3881317890172014E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.6940658945086007E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 2.7105054312137611E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    2.7105054312137611E-20, 2.7105054312137611E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 2.3716922523120409E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -1.6940658945086007E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999974, -2.3716922523120409E-20,
    0.079999999999999974, -2.7105054312137611E-20, 0.079999999999999974,
    -3.0493186101154812E-20, 0.07999999999999996, -3.3881317890172014E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -3.0493186101154812E-20, 0.07999999999999996, -3.0493186101154812E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -2.7105054312137611E-20, 0.07999999999999996, -2.7105054312137611E-20,
    0.07999999999999996, -2.3716922523120409E-20, 0.07999999999999996,
    -2.7105054312137611E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.079999999999999946,
    -3.0493186101154812E-20, 0.079999999999999932, -3.0493186101154812E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 2.7105054312137611E-20, 2.7105054312137611E-20,
    2.3716922523120409E-20, 2.7105054312137611E-20, 2.3716922523120409E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.08, -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -1.6940658945086007E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999974, -2.3716922523120409E-20,
    0.079999999999999974, -2.7105054312137611E-20, 0.079999999999999974,
    -3.0493186101154812E-20, 0.07999999999999996, -3.3881317890172014E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -3.0493186101154812E-20, 0.07999999999999996, -3.0493186101154812E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -2.7105054312137611E-20, 0.07999999999999996, -2.7105054312137611E-20,
    0.07999999999999996, -2.3716922523120409E-20, 0.07999999999999996,
    -2.7105054312137611E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.079999999999999946,
    -3.0493186101154812E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.6940658945086007E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 2.7105054312137611E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 2.7105054312137611E-20,
    2.7105054312137611E-20, 2.3716922523120409E-20, 2.7105054312137611E-20,
    2.3716922523120409E-20, 2.3716922523120409E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -1.6940658945086007E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999974, -2.3716922523120409E-20,
    0.079999999999999974, -2.7105054312137611E-20, 0.079999999999999974,
    -3.0493186101154812E-20, 0.07999999999999996, -3.3881317890172014E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -3.0493186101154812E-20, 0.07999999999999996, -3.0493186101154812E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -2.7105054312137611E-20, 0.07999999999999996, -2.7105054312137611E-20,
    0.07999999999999996, -2.3716922523120409E-20, 0.07999999999999996,
    -2.7105054312137611E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.6940658945086007E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 2.7105054312137611E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    2.7105054312137611E-20, 2.7105054312137611E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 2.3716922523120409E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999988, -1.6940658945086007E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -2.0328790734103208E-20, 0.079999999999999974, -1.6940658945086007E-20,
    0.079999999999999974, -2.0328790734103208E-20, 0.079999999999999988,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -2.0328790734103208E-20, 0.079999999999999974,
    -2.3716922523120409E-20, 0.079999999999999974, -2.7105054312137611E-20,
    0.079999999999999974, -3.0493186101154812E-20, 0.07999999999999996,
    -3.3881317890172014E-20, 0.07999999999999996, -3.0493186101154812E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -3.0493186101154812E-20, 0.07999999999999996, -3.0493186101154812E-20,
    0.07999999999999996, -2.7105054312137611E-20, 0.07999999999999996,
    -2.7105054312137611E-20, 0.07999999999999996, -2.3716922523120409E-20,
    0.07999999999999996, -2.7105054312137611E-20, 0.079999999999999946,
    -2.3716922523120409E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.6940658945086007E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 2.7105054312137611E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    2.7105054312137611E-20, 2.7105054312137611E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -1.6940658945086007E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999974, -2.3716922523120409E-20,
    0.079999999999999974, -2.7105054312137611E-20, 0.079999999999999974,
    -3.0493186101154812E-20, 0.07999999999999996, -3.3881317890172014E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -3.0493186101154812E-20, 0.07999999999999996, -3.0493186101154812E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -2.7105054312137611E-20, 0.07999999999999996, -2.7105054312137611E-20,
    0.07999999999999996, -2.3716922523120409E-20, 0.07999999999999996,
    -2.7105054312137611E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.6940658945086007E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 2.7105054312137611E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    2.7105054312137611E-20, 2.7105054312137611E-20, 2.3716922523120409E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -1.6940658945086007E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999974, -2.3716922523120409E-20,
    0.079999999999999974, -2.7105054312137611E-20, 0.079999999999999974,
    -3.0493186101154812E-20, 0.07999999999999996, -3.3881317890172014E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -3.0493186101154812E-20, 0.07999999999999996, -3.0493186101154812E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -2.7105054312137611E-20, 0.07999999999999996, -2.7105054312137611E-20,
    0.07999999999999996, -2.3716922523120409E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 2.7105054312137611E-20, 2.7105054312137611E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.08, -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -1.6940658945086007E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999974, -2.3716922523120409E-20,
    0.079999999999999974, -2.7105054312137611E-20, 0.079999999999999974,
    -3.0493186101154812E-20, 0.07999999999999996, -3.3881317890172014E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -3.0493186101154812E-20, 0.07999999999999996, -3.0493186101154812E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -2.7105054312137611E-20, 0.07999999999999996, -2.7105054312137611E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.6940658945086007E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 2.7105054312137611E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 2.7105054312137611E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -1.6940658945086007E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999974, -2.3716922523120409E-20,
    0.079999999999999974, -2.7105054312137611E-20, 0.079999999999999974,
    -3.0493186101154812E-20, 0.07999999999999996, -3.3881317890172014E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -3.0493186101154812E-20, 0.07999999999999996, -3.0493186101154812E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -2.7105054312137611E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -1.6940658945086007E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999974, -2.3716922523120409E-20,
    0.079999999999999974, -2.7105054312137611E-20, 0.079999999999999974,
    -3.0493186101154812E-20, 0.07999999999999996, -3.3881317890172014E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -3.0493186101154812E-20, 0.07999999999999996, -3.0493186101154812E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.6940658945086007E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 2.7105054312137611E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -1.6940658945086007E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999974, -2.3716922523120409E-20,
    0.079999999999999974, -2.7105054312137611E-20, 0.079999999999999974,
    -3.0493186101154812E-20, 0.07999999999999996, -3.3881317890172014E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -3.0493186101154812E-20, 0.07999999999999996, -3.0493186101154812E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.6940658945086007E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 2.7105054312137611E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -1.6940658945086007E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999974, -2.3716922523120409E-20,
    0.079999999999999974, -2.7105054312137611E-20, 0.079999999999999974,
    -3.0493186101154812E-20, 0.07999999999999996, -3.3881317890172014E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.07999999999999996,
    -3.0493186101154812E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    3.0493186101154812E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.08, -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -1.6940658945086007E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999974, -2.3716922523120409E-20,
    0.079999999999999974, -2.7105054312137611E-20, 0.079999999999999974,
    -3.0493186101154812E-20, 0.07999999999999996, -3.3881317890172014E-20,
    0.07999999999999996, -3.0493186101154812E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.6940658945086007E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 2.7105054312137611E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20,
    0.08, -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -1.6940658945086007E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999974, -2.3716922523120409E-20,
    0.079999999999999974, -2.7105054312137611E-20, 0.079999999999999974,
    -3.0493186101154812E-20, 0.07999999999999996, -3.3881317890172014E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 3.0493186101154812E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20,
    0.08, -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -1.6940658945086007E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999974,
    -1.6940658945086007E-20, 0.079999999999999974, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999988,
    -2.0328790734103208E-20, 0.079999999999999974, -2.0328790734103208E-20,
    0.079999999999999974, -2.3716922523120409E-20, 0.079999999999999974,
    -2.7105054312137611E-20, 0.079999999999999974, -3.0493186101154812E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999988, -1.6940658945086007E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -2.0328790734103208E-20, 0.079999999999999974, -1.6940658945086007E-20,
    0.079999999999999974, -2.0328790734103208E-20, 0.079999999999999988,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -2.0328790734103208E-20, 0.079999999999999974,
    -2.3716922523120409E-20, 0.079999999999999974, -2.7105054312137611E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.3716922523120409E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20,
    0.08, -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -1.6940658945086007E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999974, -2.3716922523120409E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.6940658945086007E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.0328790734103208E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999988, -1.6940658945086007E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -2.0328790734103208E-20, 0.079999999999999974, -1.6940658945086007E-20,
    0.079999999999999974, -2.0328790734103208E-20, 0.079999999999999988,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -2.0328790734103208E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999974, -1.6940658945086007E-20, 0.079999999999999974,
    -2.0328790734103208E-20, 0.079999999999999988, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.0328790734103208E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20,
    0.08, -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -1.6940658945086007E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.079999999999999974,
    -1.6940658945086007E-20, 0.079999999999999974, -2.0328790734103208E-20,
    0.079999999999999988, -2.0328790734103208E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999988, -1.6940658945086007E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -2.0328790734103208E-20, 0.079999999999999974, -1.6940658945086007E-20,
    0.079999999999999974, -2.0328790734103208E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    1.6940658945086007E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999988, -1.6940658945086007E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -2.0328790734103208E-20, 0.079999999999999974, -1.6940658945086007E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.6940658945086007E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.079999999999999988,
    -1.6940658945086007E-20, 0.079999999999999988, -2.0328790734103208E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.6940658945086007E-20,
    1.6940658945086007E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999988, -1.6940658945086007E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.6940658945086007E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.6940658945086007E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20,
    0.08, -1.3552527156068805E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.1858461261560205E-20, 1.3552527156068805E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.1858461261560205E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T b_a[8] = { -2.7529251322709746E-20, 0.00632455532033672,
    -1.1159380919009811E-5, -0.012649109656163445, 0.028284269046030115,
    -4.99062686434405E-6, 0.0, 1.0 };

  static const real_T b_Kx[8] = { 0.35875722517480957, -15.352761356806175,
    -0.0070601277709030508, 982.27157961646458, 0.3024065960592725,
    -14.200647826702088, -0.006496507695743484, 907.63027657430314 };

  static const real_T b_Kr[120] = { 3.2810551689427132E-7, -0.23801826373776977,
    2.5254814631542985E-6, -0.90452112664439666, 8.2083040969290328E-6,
    -1.9361544794004411, 1.8753787978474755E-5, -3.2789315112438571,
    3.5335517941387695E-5, -4.8868476966976742, 5.8953557012441823E-5,
    -6.7207005651123506, 9.0460102688477308E-5, -8.747083974611753,
    0.00013058134977339908, -10.937531088882809, 0.00017993612065893424,
    -13.267784070169816, 0.00023905174100266626, -15.717171752695497,
    0.00030837756808846741, -18.268079330932064, 0.00038829651893481623,
    -20.905496457755159, 0.00047913489390069856, -23.616632159092536,
    0.00058117074781036181, -26.390586685833693, 0.00069464102335528024,
    -29.218071884472849, 0.00081974762977833474, -32.091172912689061,
    0.00095666262278678118, -35.003145186757905, 0.0011055326185829136,
    -37.948241351549449, 0.001266482555252879, -40.921563834086733,
    0.0014396188980107926, -43.918939197976442, 0.001625032370527604,
    -46.936811075317365, 0.0018228002824160249, -49.972148929291194,
    0.0020329885125823545, -53.022370306770981, 0.0022556531993274203,
    -56.085274586364314, 0.0024908421805555953, -59.158986522219763,
    0.0027385962210399678, -62.241908135232251, 0.0029989500582287337,
    -65.3326777174325, 0.0032719332934226185, -68.430134897832119,
    0.0035575711511861797, -71.533290873500391, 0.0038558851264754346,
    -74.641303042161454, -0.0, -0.0, 3.2810551689427132E-7, -0.23801826373776977,
    2.5254814631542985E-6, -0.90452112664439666, 8.2083040969290328E-6,
    -1.9361544794004411, 1.8753787978474755E-5, -3.2789315112438571,
    3.5335517941387695E-5, -4.8868476966976742, 5.8953557012441823E-5,
    -6.7207005651123506, 9.0460102688477308E-5, -8.747083974611753,
    0.00013058134977339908, -10.937531088882809, 0.00017993612065893424,
    -13.267784070169816, 0.00023905174100266626, -15.717171752695497,
    0.00030837756808846741, -18.268079330932064, 0.00038829651893481623,
    -20.905496457755159, 0.00047913489390069856, -23.616632159092536,
    0.00058117074781036181, -26.390586685833693, 0.00069464102335528024,
    -29.218071884472849, 0.00081974762977833474, -32.091172912689061,
    0.00095666262278678118, -35.003145186757905, 0.0011055326185829136,
    -37.948241351549449, 0.001266482555252879, -40.921563834086733,
    0.0014396188980107926, -43.918939197976442, 0.001625032370527604,
    -46.936811075317365, 0.0018228002824160249, -49.972148929291194,
    0.0020329885125823545, -53.022370306770981, 0.0022556531993274203,
    -56.085274586364314, 0.0024908421805555953, -59.158986522219763,
    0.0027385962210399678, -62.241908135232251, 0.0029989500582287337,
    -65.3326777174325, 0.0032719332934226185, -68.430134897832119,
    0.0035575711511861797, -71.533290873500391 };

  static const real_T b_Kv[124] = { -0.0028363241232317073, 0.0,
    -0.0028362978747903554, 0.0, -0.0028360958362733022, 0.0,
    -0.0028354391719455476, 0.0, -0.0028339388689072684, 0.0,
    -0.0028311120274719576, 0.0, -0.0028263957429109621, 0.0,
    -0.0028191589346958836, 0.0, -0.0028087124267140114, 0.0,
    -0.0027943175370612962, 0.0, -0.0027751933977810824, 0.0,
    -0.0027505231923340037, 0.0, -0.0027194594708192178, 0.0,
    -0.0026811286793071605, 0.0, -0.0026346350194823304, 0.0,
    -0.0025790637376139061, 0.0, -0.0025134839272316378, 0.0,
    -0.0024369509174086944, 0.0, -0.0023485083079220603, 0.0,
    -0.0022471897035018285, 0.0, -0.0021320201916609645, 0.0,
    -0.0020020176020187548, 0.0, -0.0018561935794254719, 0.0,
    -0.0016935544984188829, 0.0, -0.0015131022424726879, 0.0,
    -0.0013138348680282392, 0.0, -0.0010947471703450411, 0.0,
    -0.00085483116568674136, 0.0, -0.000593076502212931, 0.0,
    -0.00030847081011803562, 0.0, 0.0, 0.0, -0.0025278533131136705, 0.0,
    -0.00252785331311367, 0.0, -0.0025278270646723181, 0.0,
    -0.0025276250261552654, 0.0, -0.0025269683618275103, 0.0,
    -0.0025254680587892325, 0.0, -0.0025226412173539213, 0.0,
    -0.0025179249327929253, 0.0, -0.0025106881245778468, 0.0,
    -0.0025002416165959746, 0.0, -0.0024858467269432594, 0.0,
    -0.0024667225876630456, 0.0, -0.0024420523822159669, 0.0,
    -0.002410988660701181, 0.0, -0.0023726578691891237, 0.0,
    -0.0023261642093642928, 0.0, -0.0022705929274958693, 0.0,
    -0.0022050131171136014, 0.0, -0.0021284801072906576, 0.0,
    -0.0020400374978040239, 0.0, -0.0019387188933837926, 0.0,
    -0.0018235493815429284, 0.0, -0.001693546791900719, 0.0,
    -0.001547722769307436, 0.0, -0.0013850836883008467, 0.0,
    -0.0012046314323546519, 0.0, -0.0010053640579102035, 0.0,
    -0.00078627636022700527, 0.0, -0.00054636035556870563, 0.0,
    -0.00028460569209489523, 0.0, 0.0, 0.0 };

  static const real_T c_a[16] = { 0.85214378896621124, 6.52213355998541E-6,
    -0.016530825651721814, 0.0, -0.073928099762894439, 0.99998561653740869,
    0.036455940405912865, 0.0, -2.916786799510717E-5, -5.6749049349282934E-9,
    1.0000143834625914, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T d_a[4] = { -1.8318475397195182, -1.0119283148112526,
    -0.0015592782385630637, 0.0 };

  static const real_T e_a[8] = { -3.5598245308881651E-18, -0.0011159380919009794,
    2.8284269046030124, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T f_a[8] = { -0.095613466557929683, 0.194292547570302,
    0.96403398614800939, 0.0029638232952478023, 0.027073926455613347,
    -0.18358563847104539, -0.089658041178056452, 0.068929470890853883 };

  boolean_T b_varargout_1;
  boolean_T exitg1;
  boolean_T guard1 = false;

  // Outputs for Atomic SubSystem: '<Root>/target_output'
  // MATLABSystem: '<S5>/SourceBlock' incorporates:
  //   Inport: '<S41>/In1'

  b_varargout_1 = Sub_ACC_144.getLatestMessage(&ACC_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S5>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S41>/Enable'

  if (b_varargout_1) {
    ACC_B.In1 = ACC_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S5>/SourceBlock'
  // End of Outputs for SubSystem: '<S5>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/target_output'

  // Outputs for Atomic SubSystem: '<Root>/drive_ctrl_input'
  // MATLABSystem: '<S4>/SourceBlock' incorporates:
  //   Inport: '<S40>/In1'

  b_varargout_1 = Sub_ACC_145.getLatestMessage(&ACC_B.b_varargout_2_m);

  // Outputs for Enabled SubSystem: '<S4>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S40>/Enable'

  if (b_varargout_1) {
    ACC_B.In1_p = ACC_B.b_varargout_2_m;
  }

  // End of MATLABSystem: '<S4>/SourceBlock'
  // End of Outputs for SubSystem: '<S4>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/drive_ctrl_input'

  // If: '<S7>/If' incorporates:
  //   Constant: '<S7>/Constant'
  //   Constant: '<S7>/Constant1'
  //   Constant: '<S7>/Constant2'
  //   Constant: '<S7>/Constant3'
  //   Constant: '<S7>/Constant4'
  //   Constant: '<S7>/Constant6'
  //   Inport: '<S13>/In2'
  //   Inport: '<S13>/Input'
  //   Inport: '<S14>/In1'
  //   Inport: '<S14>/In2'
  //   Inport: '<S15>/In2'
  //   Inport: '<S15>/Input'

  if (ACC_B.In1_p.AccGapLevel == 1) {
    // Outputs for IfAction SubSystem: '<S7>/Near' incorporates:
    //   ActionPort: '<S13>/Action Port'

    ACC_B.Switch2 = ACC_P.data.acc_near_time;
    ACC_B.Merge1 = ACC_P.data.acc_near_dist;

    // End of Outputs for SubSystem: '<S7>/Near'
  } else if (ACC_B.In1_p.AccGapLevel == 3) {
    // Outputs for IfAction SubSystem: '<S7>/Time_Far' incorporates:
    //   ActionPort: '<S14>/Action Port'

    ACC_B.Switch2 = ACC_P.data.acc_far_time;
    ACC_B.Merge1 = ACC_P.data.acc_far_dist;

    // End of Outputs for SubSystem: '<S7>/Time_Far'
  } else {
    // Outputs for IfAction SubSystem: '<S7>/Time_Medium' incorporates:
    //   ActionPort: '<S15>/Action Port'

    ACC_B.Switch2 = ACC_P.data.acc_med_time;
    ACC_B.Merge1 = ACC_P.data.acc_med_dist;

    // End of Outputs for SubSystem: '<S7>/Time_Medium'
  }

  // End of If: '<S7>/If'

  // Gain: '<S1>/km//h to m//s'
  ACC_B.set_velocity = ACC_P.kmhtoms_Gain * ACC_B.In1_p.AccSpeedSetPoint;

  // Sum: '<S1>/Sum6'
  ACC_B.lead_velocity = ACC_B.In1_p.VehSpd + ACC_B.In1.ObjVx;

  // Switch: '<S8>/Switch1' incorporates:
  //   Constant: '<S8>/Constant3'
  //   Constant: '<S8>/Constant4'

  if (ACC_B.In1_p.VehSpd > ACC_P.data.veh_spd_thr) {
    ACC_B.Switch1 = ACC_P.data.acc_accel_min_over;
  } else {
    ACC_B.Switch1 = ACC_P.data.accel_min_under;
  }

  // End of Switch: '<S8>/Switch1'

  // Gain: '<S20>/umin_scale'
  ACC_B.umin_scale = ACC_P.umin_scale_Gain * ACC_B.Switch1;

  // Gain: '<S20>/umax_scale' incorporates:
  //   Constant: '<S8>/Constant2'

  ACC_B.umax_scale = ACC_P.umax_scale_Gain * ACC_P.data.acc_accel_max;

  // Product: '<S1>/Product2'
  ACC_B.Switch2 *= ACC_B.In1_p.VehSpd;

  // Sum: '<S1>/Sum1'
  ACC_B.safe_distance = ACC_B.Merge1 + ACC_B.Switch2;

  // Gain: '<S20>/ymin_scale' incorporates:
  //   Constant: '<S1>/Minimum velocity constant'

  ACC_B.ymin_scale[0] = ACC_P.ymin_scale_Gain[0] * ACC_B.safe_distance;
  ACC_B.ymin_scale[1] = ACC_P.ymin_scale_Gain[1] *
    ACC_P.Minimumvelocityconstant_Value;

  // Gain: '<S20>/ymax_scale' incorporates:
  //   Constant: '<S1>/Maximum velocity constant'
  //   Constant: '<S1>/Unconstrained'

  ACC_B.ymax_scale[0] = ACC_P.ymax_scale_Gain[0] * ACC_P.Unconstrained_Value;
  ACC_B.ymax_scale[1] = ACC_P.ymax_scale_Gain[1] *
    ACC_P.Maximumvelocityconstant_Value;

  // MATLAB Function: '<S20>/optimizer' incorporates:
  //   Memory: '<S20>/last_x'
  //   SignalConversion: '<S39>/TmpSignal ConversionAt SFunction Inport4'

  memset(&ACC_B.vseq[0], 0, 62U * sizeof(real_T));
  for (ACC_B.i = 0; ACC_B.i < 31; ACC_B.i++) {
    ACC_B.vseq[(ACC_B.i * (int32_T)ACC_nv + (int32_T)ACC_nv) - 1] = 1.0;
  }

  memset(&ACC_B.rseq[0], 0, 60U * sizeof(real_T));
  for (ACC_B.i = 0; ACC_B.i < 30; ACC_B.i++) {
    ACC_B.rseq_tmp = ACC_B.i * (int32_T)ACC_ny;
    ACC_B.rseq[ACC_B.rseq_tmp] = ACC_B.Merge1 * 0.02 - 0.97;
    ACC_B.rseq[1 + ACC_B.rseq_tmp] = ACC_B.set_velocity * 0.02 - 0.4;
  }

  for (ACC_B.i = 0; ACC_B.i < 31; ACC_B.i++) {
    ACC_B.vseq[ACC_B.i * (int32_T)ACC_nv] = ACC_RMDscale * ACC_B.lead_velocity -
      ACC_voff;
  }

  ACC_B.lead_velocity = ACC_B.vseq[0];
  ACC_B.Switch2 = ACC_B.vseq[1];
  ACC_B.xk[0] = ACC_DW.last_x_PreviousInput[0];
  ACC_B.xk[1] = ACC_DW.last_x_PreviousInput[1];
  ACC_B.xk[2] = ACC_DW.last_x_PreviousInput[2];
  ACC_B.xk[3] = ACC_DW.last_x_PreviousInput[3];

  // SignalConversion: '<S39>/TmpSignal ConversionAt SFunction Inport3' incorporates:
  //   MATLAB Function: '<S20>/optimizer'

  ACC_B.dv0[0] = ACC_B.In1.ObjDx;
  ACC_B.dv0[1] = ACC_B.In1_p.VehSpd;

  // MATLAB Function: '<S20>/optimizer'
  for (ACC_B.i = 0; ACC_B.i < 2; ACC_B.i++) {
    ACC_B.umax_incr = b_a[ACC_B.i + 6] * ACC_B.xk[3] + (b_a[ACC_B.i + 4] *
      ACC_B.xk[2] + (b_a[ACC_B.i + 2] * ACC_B.xk[1] + b_a[ACC_B.i] * ACC_B.xk[0]));
    ACC_B.y_innov[ACC_B.i] = (ACC_B.dv0[ACC_B.i] * 0.02 - (-0.57 * (real_T)
      ACC_B.i + 0.97)) - (ACC_B.umax_incr + (0.0 * ACC_B.Switch2 + 0.0 *
      ACC_B.lead_velocity));
  }

  for (ACC_B.i = 0; ACC_B.i < 4; ACC_B.i++) {
    ACC_B.xest[ACC_B.i] = (a[ACC_B.i + 4] * ACC_B.y_innov[1] + a[ACC_B.i] *
      ACC_B.y_innov[0]) + ACC_B.xk[ACC_B.i];
  }

  ACC_B.umax_incr = 1.0;
  memset(&ACC_B.iAout[0], 0, 98U * sizeof(boolean_T));

  // Switch: '<S1>/Switch' incorporates:
  //   Constant: '<S1>/Constant1'
  //   Constant: '<S1>/Not use ACC output constant'
  //   Constant: '<S1>/Use ACC output constant'

  if (ACC_P.Constant1_Value != 0.0) {
    ACC_B.umin_incr = ACC_P.UseACCoutputconstant_Value;
  } else {
    ACC_B.umin_incr = ACC_P.NotuseACCoutputconstant_Value;
  }

  // End of Switch: '<S1>/Switch'

  // MATLAB Function: '<S20>/optimizer' incorporates:
  //   Memory: '<S20>/Memory'
  //   UnitDelay: '<S20>/last_mv'

  if (!(ACC_B.umin_incr != ACC_enable_value)) {
    for (ACC_B.i = 0; ACC_B.i < 98; ACC_B.i++) {
      ACC_B.b_Mv[ACC_B.i] = 0.0;
      for (ACC_B.rseq_tmp = 0; ACC_B.rseq_tmp < 62; ACC_B.rseq_tmp++) {
        ACC_B.b_Mv[ACC_B.i] += b_Mv[98 * ACC_B.rseq_tmp + ACC_B.i] *
          ACC_B.vseq[ACC_B.rseq_tmp];
      }

      ACC_B.Bc[ACC_B.i] = -((((((b_Mx[ACC_B.i + 98] * ACC_B.xest[1] +
        b_Mx[ACC_B.i] * ACC_B.xest[0]) + b_Mx[ACC_B.i + 196] * ACC_B.xest[2]) +
        b_Mx[ACC_B.i + 294] * ACC_B.xest[3]) + b_Mlim[ACC_B.i]) + b_Mu1[ACC_B.i]
        * ACC_DW.last_mv_DSTATE) + ACC_B.b_Mv[ACC_B.i]);
    }

    ACC_B.ymax_incr[0] = (rtNaN);
    ACC_B.ymin_incr[0] = (rtNaN);
    ACC_B.ymax_incr[1] = (rtNaN);
    ACC_B.ymin_incr[1] = (rtNaN);
    ACC_B.umax_incr = (rtNaN);
    ACC_B.umin_incr = (rtNaN);
    ACC_B.rseq_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (ACC_B.rseq_tmp < 98)) {
      guard1 = false;
      if (b_Mrows[ACC_B.rseq_tmp] <= 60) {
        ACC_B.i = (int32_T)(ACC_mod((real_T)b_Mrows[ACC_B.rseq_tmp] - 1.0) + 1.0)
          - 1;
        if (rtIsNaN(ACC_B.ymax_incr[ACC_B.i])) {
          ACC_B.DelBound = -(ACC_B.ymax_scale[ACC_B.i] - ((real_T)ACC_B.i *
            -0.57 + 0.97)) - (-b_Mlim[ACC_B.rseq_tmp]);
        } else {
          ACC_B.DelBound = ACC_B.ymax_incr[ACC_B.i];
        }

        ACC_B.ymax_incr[ACC_B.i] = ACC_B.DelBound;
        guard1 = true;
      } else if (b_Mrows[ACC_B.rseq_tmp] <= 120) {
        ACC_B.i = (int32_T)(ACC_mod(((real_T)b_Mrows[ACC_B.rseq_tmp] - 60.0) -
          1.0) + 1.0) - 1;
        if (rtIsNaN(ACC_B.ymin_incr[ACC_B.i])) {
          ACC_B.DelBound = (ACC_B.ymin_scale[ACC_B.i] - ((real_T)ACC_B.i * -0.57
            + 0.97)) - (-b_Mlim[ACC_B.rseq_tmp]);
        } else {
          ACC_B.DelBound = ACC_B.ymin_incr[ACC_B.i];
        }

        ACC_B.ymin_incr[ACC_B.i] = ACC_B.DelBound;
        guard1 = true;
      } else if (b_Mrows[ACC_B.rseq_tmp] <= 150) {
        if (rtIsNaN(ACC_B.umax_incr)) {
          ACC_B.umax_incr = -ACC_B.umax_scale - (-b_Mlim[ACC_B.rseq_tmp]);
        }

        ACC_B.DelBound = ACC_B.umax_incr;
        guard1 = true;
      } else if (b_Mrows[ACC_B.rseq_tmp] <= 180) {
        if (rtIsNaN(ACC_B.umin_incr)) {
          ACC_B.umin_incr = ACC_B.umin_scale - (-b_Mlim[ACC_B.rseq_tmp]);
        }

        ACC_B.DelBound = ACC_B.umin_incr;
        guard1 = true;
      } else {
        exitg1 = true;
      }

      if (guard1) {
        ACC_B.Bc[ACC_B.rseq_tmp] += ACC_B.DelBound;
        ACC_B.rseq_tmp++;
      }
    }

    ACC_B.f[0] = 0.0;
    ACC_B.f[1] = 0.0;
    ACC_B.f[2] = 0.0;
    for (ACC_B.rseq_tmp = 0; ACC_B.rseq_tmp < 2; ACC_B.rseq_tmp++) {
      ACC_B.umin_scale = 0.0;
      for (ACC_B.i = 0; ACC_B.i < 60; ACC_B.i++) {
        ACC_B.umin_scale += b_Kr[60 * ACC_B.rseq_tmp + ACC_B.i] *
          ACC_B.rseq[ACC_B.i];
      }

      ACC_B.umax_scale = 0.0;
      for (ACC_B.i = 0; ACC_B.i < 62; ACC_B.i++) {
        ACC_B.umax_scale += b_Kv[62 * ACC_B.rseq_tmp + ACC_B.i] *
          ACC_B.vseq[ACC_B.i];
      }

      ACC_B.i = ACC_B.rseq_tmp << 2;
      ACC_B.f[ACC_B.rseq_tmp] = (((((b_Kx[ACC_B.i + 1] * ACC_B.xest[1] +
        b_Kx[ACC_B.i] * ACC_B.xest[0]) + b_Kx[ACC_B.i + 2] * ACC_B.xest[2]) +
        b_Kx[ACC_B.i + 3] * ACC_B.xest[3]) + ACC_B.umin_scale) +
        (-14.733191722230885 * (real_T)ACC_B.rseq_tmp + 247.71913547111888) *
        ACC_DW.last_mv_DSTATE) + ACC_B.umax_scale;
    }

    for (ACC_B.i = 0; ACC_B.i < 98; ACC_B.i++) {
      ACC_B.iAnew[ACC_B.i] = ACC_DW.Memory_PreviousInput[ACC_B.i];
    }

    ACC_qpkwik(b_Linv, b_Hinv, ACC_B.f, b_Ac, ACC_B.Bc, ACC_B.iAnew, 404, 1.0E-6,
               ACC_B.zopt, ACC_B.b_Mv, &ACC_B.umax_incr);
    if ((ACC_B.umax_incr < 0.0) || (ACC_B.umax_incr == 0.0)) {
      ACC_B.zopt[0] = 0.0;
    }

    for (ACC_B.i = 0; ACC_B.i < 98; ACC_B.i++) {
      ACC_B.iAout[ACC_B.i] = (ACC_B.iAnew[ACC_B.i] != 0);
    }

    ACC_DW.last_mv_DSTATE += ACC_B.zopt[0];
  }

  // Gain: '<S20>/umin_scale1' incorporates:
  //   UnitDelay: '<S20>/last_mv'

  ACC_B.umin_scale = ACC_P.umin_scale1_Gain * ACC_DW.last_mv_DSTATE;

  // Chart: '<S6>/ACC Diagnostics' incorporates:
  //   Delay: '<S6>/Delay'

  ACC_DW.chartAbsoluteTimeCounter++;
  b_varargout_1 = (ACC_B.In1.ObjDx > ACC_B.safe_distance);
  if ((!b_varargout_1) || (!ACC_DW.condWasTrueAtLastTimeStep_1)) {
    ACC_DW.durationLastReferenceTick_1 = ACC_DW.chartAbsoluteTimeCounter;
  }

  ACC_DW.condWasTrueAtLastTimeStep_1 = b_varargout_1;
  b_varargout_1 = (ACC_DW.Delay_DSTATE > 1.0);
  if ((!b_varargout_1) || (!ACC_DW.condWasTrueAtLastTimeStep_1_c)) {
    ACC_DW.durationLastReferenceTick_1_h = ACC_DW.chartAbsoluteTimeCounter;
  }

  ACC_DW.condWasTrueAtLastTimeStep_1_c = b_varargout_1;
  b_varargout_1 = ((ACC_B.In1.ObjVx > 0.0) && (ACC_B.In1.ObjDx > 1.5 *
    ACC_B.Merge1));
  if ((!b_varargout_1) || (!ACC_DW.condWasTrueAtLastTimeStep_1_c0)) {
    ACC_DW.durationLastReferenceTick_1_d = ACC_DW.chartAbsoluteTimeCounter;
  }

  ACC_DW.condWasTrueAtLastTimeStep_1_c0 = b_varargout_1;
  b_varargout_1 = (ACC_B.set_velocity >= ACC_B.In1_p.VehSpd);
  if ((!b_varargout_1) || (!ACC_DW.condWasTrueAtLastTimeStep_1_d)) {
    ACC_DW.durationLastReferenceTick_1_n = ACC_DW.chartAbsoluteTimeCounter;
  }

  ACC_DW.condWasTrueAtLastTimeStep_1_d = b_varargout_1;
  if (ACC_DW.is_active_c14_ACC == 0U) {
    ACC_DW.chartAbsoluteTimeCounter = 0;
    ACC_DW.is_active_c14_ACC = 1U;
    ACC_DW.is_c14_ACC = ACC_IN_Init;
  } else if (ACC_DW.is_c14_ACC == ACC_IN_ACC) {
    if (!ACC_B.In1_p.AccActivation) {
      ACC_DW.is_ACC_Normal = ACC_IN_NO_ACTIVE_CHILD;
      ACC_DW.is_Standard = ACC_IN_NO_ACTIVE_CHILD;
      ACC_DW.is_ACC = ACC_IN_NO_ACTIVE_CHILD;
      ACC_DW.is_c14_ACC = ACC_IN_Init;
    } else {
      switch (ACC_DW.is_ACC) {
       case ACC_IN_FullBrakes:
        b_varargout_1 = (ACC_B.In1.ObjDx > ACC_B.safe_distance);
        if ((!b_varargout_1) || (!ACC_DW.condWasTrueAtLastTimeStep_1)) {
          ACC_DW.durationLastReferenceTick_1 = ACC_DW.chartAbsoluteTimeCounter;
        }

        ACC_DW.condWasTrueAtLastTimeStep_1 = b_varargout_1;
        if ((ACC_DW.chartAbsoluteTimeCounter -
             ACC_DW.durationLastReferenceTick_1 > 23) && (ACC_B.In1.ObjVx > 0.0))
        {
          ACC_DW.durationLastReferenceTick_1_h = ACC_DW.chartAbsoluteTimeCounter;
          ACC_DW.is_ACC = ACC_IN_RateLimited;
          ACC_DW.condWasTrueAtLastTimeStep_1_c = (ACC_DW.Delay_DSTATE > 1.0);
        } else {
          ACC_B.ACC_Accel_Out = ACC_B.Switch1;
          ACC_B.RateLimiter = false;
        }
        break;

       case ACC_IN_RateLimited:
        b_varargout_1 = (ACC_DW.Delay_DSTATE > 1.0);
        if ((!b_varargout_1) || (!ACC_DW.condWasTrueAtLastTimeStep_1_c)) {
          ACC_DW.durationLastReferenceTick_1_h = ACC_DW.chartAbsoluteTimeCounter;
        }

        ACC_DW.condWasTrueAtLastTimeStep_1_c = b_varargout_1;
        if (ACC_DW.chartAbsoluteTimeCounter -
            ACC_DW.durationLastReferenceTick_1_h > 29) {
          ACC_DW.is_ACC = ACC_IN_Standard;
          ACC_DW.is_Standard = ACC_IN_ACC_Normal;
          ACC_DW.is_ACC_Normal = ACC_IN_ACC_Normal;
        } else if (ACC_B.In1.ObjDx < ACC_B.safe_distance) {
          ACC_DW.durationLastReferenceTick_1 = ACC_DW.chartAbsoluteTimeCounter;
          ACC_DW.is_ACC = ACC_IN_FullBrakes;
          ACC_DW.condWasTrueAtLastTimeStep_1 = (ACC_B.In1.ObjDx >
            ACC_B.safe_distance);
        } else {
          ACC_B.ACC_Accel_Out = ACC_B.umin_scale;
          ACC_B.RateLimiter = true;
        }
        break;

       default:
        ACC_Standard(&ACC_B.set_velocity, &ACC_B.Merge1, &ACC_B.safe_distance,
                     &ACC_B.umin_scale, &ACC_B.Switch1);
        break;
      }
    }
  } else if (ACC_B.In1_p.AccActivation) {
    ACC_DW.is_c14_ACC = ACC_IN_ACC;
    ACC_DW.is_ACC = ACC_IN_Standard;
    ACC_DW.is_Standard = ACC_IN_ACC_Normal;
    ACC_DW.is_ACC_Normal = ACC_IN_ACC_Normal;
  } else {
    ACC_B.ACC_Accel_Out = 0.0;
  }

  // End of Chart: '<S6>/ACC Diagnostics'

  // RateLimiter: '<S6>/Rate Limiter' incorporates:
  //   Delay: '<S6>/Delay'

  ACC_B.Merge1 = ACC_B.ACC_Accel_Out - ACC_DW.PrevY;
  if (ACC_B.Merge1 > ACC_P.RateLimiter_RisingLim) {
    ACC_DW.Delay_DSTATE = ACC_DW.PrevY + ACC_P.RateLimiter_RisingLim;
  } else if (ACC_B.Merge1 < ACC_P.RateLimiter_FallingLim) {
    ACC_DW.Delay_DSTATE = ACC_DW.PrevY + ACC_P.RateLimiter_FallingLim;
  } else {
    ACC_DW.Delay_DSTATE = ACC_B.ACC_Accel_Out;
  }

  ACC_DW.PrevY = ACC_DW.Delay_DSTATE;

  // End of RateLimiter: '<S6>/Rate Limiter'

  // Switch: '<S6>/Switch' incorporates:
  //   Delay: '<S6>/Delay'

  if (ACC_B.RateLimiter) {
    ACC_B.Merge1 = ACC_DW.Delay_DSTATE;
  } else {
    ACC_B.Merge1 = ACC_B.ACC_Accel_Out;
  }

  // End of Switch: '<S6>/Switch'

  // BusAssignment: '<Root>/BusAssign' incorporates:
  //   Constant: '<S16>/Constant'
  //   Constant: '<S2>/Constant'
  //   Logic: '<S9>/NOT2'
  //   MATLAB Function: '<S20>/optimizer'
  //   RelationalOperator: '<S16>/Compare'

  ACC_B.BusAssign = ACC_P.Constant_Value_e;
  ACC_B.BusAssign.AccFault = !(ACC_B.umax_incr > ACC_P.CompareToConstant_const);

  // Switch: '<S12>/Switch2' incorporates:
  //   Constant: '<S8>/Constant2'
  //   RelationalOperator: '<S12>/LowerRelop1'
  //   RelationalOperator: '<S12>/UpperRelop'
  //   Switch: '<S12>/Switch'

  if (ACC_B.Merge1 > ACC_P.data.acc_accel_max) {
    // BusAssignment: '<Root>/BusAssign'
    ACC_B.BusAssign.AccAccel = ACC_P.data.acc_accel_max;
  } else if (ACC_B.Merge1 < ACC_B.Switch1) {
    // Switch: '<S12>/Switch' incorporates:
    //   BusAssignment: '<Root>/BusAssign'

    ACC_B.BusAssign.AccAccel = ACC_B.Switch1;
  } else {
    // BusAssignment: '<Root>/BusAssign' incorporates:
    //   Switch: '<S12>/Switch'

    ACC_B.BusAssign.AccAccel = ACC_B.Merge1;
  }

  // End of Switch: '<S12>/Switch2'

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S3>/SinkBlock'
  Pub_ACC_141.publish(&ACC_B.BusAssign);

  // End of Outputs for SubSystem: '<Root>/Publish'
  for (ACC_B.i = 0; ACC_B.i < 4; ACC_B.i++) {
    // Update for Memory: '<S20>/last_x' incorporates:
    //   MATLAB Function: '<S20>/optimizer'

    ACC_DW.last_x_PreviousInput[ACC_B.i] = (((((c_a[ACC_B.i + 4] * ACC_B.xk[1] +
      c_a[ACC_B.i] * ACC_B.xk[0]) + c_a[ACC_B.i + 8] * ACC_B.xk[2]) +
      c_a[ACC_B.i + 12] * ACC_B.xk[3]) + d_a[ACC_B.i] * ACC_DW.last_mv_DSTATE) +
      (0.0 * ACC_B.Switch2 + e_a[ACC_B.i] * ACC_B.lead_velocity)) + (f_a[ACC_B.i
      + 4] * ACC_B.y_innov[1] + f_a[ACC_B.i] * ACC_B.y_innov[0]);
  }

  // Update for Memory: '<S20>/Memory'
  memcpy(&ACC_DW.Memory_PreviousInput[0], &ACC_B.iAout[0], 98U * sizeof
         (boolean_T));
}

// Model initialize function
void ACC_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // initialize error status
  rtmSetErrorStatus(ACC_M, (NULL));

  // block I/O
  (void) memset(((void *) &ACC_B), 0,
                sizeof(B_ACC_T));

  // states (dwork)
  (void) memset((void *)&ACC_DW, 0,
                sizeof(DW_ACC_T));

  {
    static const char_T tmp[15] = { '/', 'a', 'c', 'c', '_', 'o', 'u', 't', 'p',
      'u', 't', '_', 'm', 's', 'g' };

    static const char_T tmp_0[17] = { '/', 'd', 'r', 'i', 'v', 'e', '_', 'c',
      't', 'r', 'l', '_', 'i', 'n', 'p', 'u', 't' };

    static const char_T tmp_1[14] = { '/', 't', 'a', 'r', 'g', 'e', 't', '_',
      'o', 'u', 't', 'p', 'u', 't' };

    char_T tmp_2[16];
    char_T tmp_3[18];
    char_T tmp_4[15];
    int32_T i;

    // Start for Atomic SubSystem: '<Root>/target_output'
    // Start for MATLABSystem: '<S5>/SourceBlock'
    ACC_DW.obj_f.matlabCodegenIsDeleted = true;
    ACC_DW.obj_f.isInitialized = 0;
    ACC_DW.obj_f.matlabCodegenIsDeleted = false;
    ACC_DW.obj_f.isSetupComplete = false;
    ACC_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_4[i] = tmp_1[i];
    }

    tmp_4[14] = '\x00';
    Sub_ACC_144.createSubscriber(tmp_4, 1);
    ACC_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/SourceBlock'
    // End of Start for SubSystem: '<Root>/target_output'

    // Start for Atomic SubSystem: '<Root>/drive_ctrl_input'
    // Start for MATLABSystem: '<S4>/SourceBlock'
    ACC_DW.obj_p.matlabCodegenIsDeleted = true;
    ACC_DW.obj_p.isInitialized = 0;
    ACC_DW.obj_p.matlabCodegenIsDeleted = false;
    ACC_DW.obj_p.isSetupComplete = false;
    ACC_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      tmp_3[i] = tmp_0[i];
    }

    tmp_3[17] = '\x00';
    Sub_ACC_145.createSubscriber(tmp_3, 1);
    ACC_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S4>/SourceBlock'
    // End of Start for SubSystem: '<Root>/drive_ctrl_input'

    // Start for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S3>/SinkBlock'
    ACC_DW.obj.matlabCodegenIsDeleted = true;
    ACC_DW.obj.isInitialized = 0;
    ACC_DW.obj.matlabCodegenIsDeleted = false;
    ACC_DW.obj.isSetupComplete = false;
    ACC_DW.obj.isInitialized = 1;
    for (i = 0; i < 15; i++) {
      tmp_2[i] = tmp[i];
    }

    tmp_2[15] = '\x00';
    Pub_ACC_141.createPublisher(tmp_2, 1);
    ACC_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S3>/SinkBlock'
    // End of Start for SubSystem: '<Root>/Publish'

    // InitializeConditions for Memory: '<S20>/last_x'
    ACC_DW.last_x_PreviousInput[0] = ACC_P.last_x_InitialCondition[0];
    ACC_DW.last_x_PreviousInput[1] = ACC_P.last_x_InitialCondition[1];
    ACC_DW.last_x_PreviousInput[2] = ACC_P.last_x_InitialCondition[2];
    ACC_DW.last_x_PreviousInput[3] = ACC_P.last_x_InitialCondition[3];

    // InitializeConditions for UnitDelay: '<S20>/last_mv'
    ACC_DW.last_mv_DSTATE = ACC_P.last_mv_InitialCondition;

    // InitializeConditions for Memory: '<S20>/Memory'
    memcpy(&ACC_DW.Memory_PreviousInput[0], &ACC_P.Memory_InitialCondition[0],
           98U * sizeof(boolean_T));

    // InitializeConditions for Delay: '<S6>/Delay'
    ACC_DW.Delay_DSTATE = ACC_P.Delay_InitialCondition;

    // InitializeConditions for RateLimiter: '<S6>/Rate Limiter'
    ACC_DW.PrevY = ACC_P.RateLimiter_IC;

    // SystemInitialize for Atomic SubSystem: '<Root>/target_output'
    // SystemInitialize for Enabled SubSystem: '<S5>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S41>/Out1'
    ACC_B.In1 = ACC_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S5>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/target_output'

    // SystemInitialize for Atomic SubSystem: '<Root>/drive_ctrl_input'
    // SystemInitialize for Enabled SubSystem: '<S4>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S40>/Out1'
    ACC_B.In1_p = ACC_P.Out1_Y0_a;

    // End of SystemInitialize for SubSystem: '<S4>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/drive_ctrl_input'

    // SystemInitialize for Chart: '<S6>/ACC Diagnostics'
    ACC_DW.is_ACC = ACC_IN_NO_ACTIVE_CHILD;
    ACC_DW.is_Standard = ACC_IN_NO_ACTIVE_CHILD;
    ACC_DW.is_ACC_Normal = ACC_IN_NO_ACTIVE_CHILD;
    ACC_DW.is_active_c14_ACC = 0U;
    ACC_DW.is_c14_ACC = ACC_IN_NO_ACTIVE_CHILD;
    ACC_DW.chartAbsoluteTimeCounter = 0;
  }
}

// Model terminate function
void ACC_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/target_output'
  // Terminate for MATLABSystem: '<S5>/SourceBlock'
  matlabCodegenHandle_matlabCod_e(&ACC_DW.obj_f);

  // End of Terminate for SubSystem: '<Root>/target_output'

  // Terminate for Atomic SubSystem: '<Root>/drive_ctrl_input'
  // Terminate for MATLABSystem: '<S4>/SourceBlock'
  matlabCodegenHandle_matlabCod_e(&ACC_DW.obj_p);

  // End of Terminate for SubSystem: '<Root>/drive_ctrl_input'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S3>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&ACC_DW.obj);

  // End of Terminate for SubSystem: '<Root>/Publish'
}

//
// File trailer for generated code.
//
// [EOF]
//
