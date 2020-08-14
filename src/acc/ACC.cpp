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
// C/C++ source code generated on : Thu Aug 13 16:50:59 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "ACC.h"
#include "ACC_private.h"

// Named constants for Chart: '<S6>/ACC Diagnostics'
#define ACC_IN_ACC_Normal              ((uint8_T)1U)
#define ACC_IN_ACC_Zeroed              ((uint8_T)2U)
#define ACC_IN_FullBrakes              ((uint8_T)1U)
#define ACC_IN_HalfBrakes              ((uint8_T)2U)
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
static void ACC_abs_a(const real_T x[98], real_T y[98]);
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
static void ACC_abs_a(const real_T x[98], real_T y[98])
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
  ACC_B.i_c = 1;
  do {
    exitg1 = 0;
    i = nA - 1;
    if (ACC_B.i_c - 1 <= i) {
      TL_tmp = (int16_T)ACC_B.i_c - 1;
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

      ACC_B.i_c++;
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
          ACC_B.i_c = 1;
          while (ACC_B.i_c - 1 <= ACC_B.b_i) {
            TL_tmp = ((int16_T)ACC_B.c_i - 1) * 3;
            ACC_B.e_i = ((int16_T)ACC_B.i_c - 1) * 3;
            ACC_B.TL[((int16_T)ACC_B.c_i + 3 * ((int16_T)ACC_B.i_c - 1)) - 1] =
              (b_Linv[TL_tmp + 1] * ACC_B.QQ[ACC_B.e_i + 1] + b_Linv[TL_tmp] *
               ACC_B.QQ[ACC_B.e_i]) + b_Linv[TL_tmp + 2] * ACC_B.QQ[ACC_B.e_i +
              2];
            ACC_B.i_c++;
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
          ACC_B.i_c = c_k - 1;
          RLinv[(b_j + 3 * ACC_B.i_c) - 1] /= ACC_B.RR[((b_j - 1) * 3 + b_j) - 1];
        }

        if (b_j > 1) {
          ACC_B.e_i = 1;
          while (ACC_B.e_i - 1 <= b_j - 2) {
            for (c_k = b_j; c_k <= nA; c_k++) {
              ACC_B.i_c = c_k - 1;
              TL_tmp = ACC_B.i_c * 3;
              RLinv[((int16_T)ACC_B.e_i + 3 * ACC_B.i_c) - 1] = RLinv[(TL_tmp +
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
          ACC_B.i_c = ((int16_T)ACC_B.e_i + 3 * ACC_B.c_i) - 1;
          b_H[ACC_B.i_c] = 0.0;
          TL_tmp = nA + 1;
          if (TL_tmp > 32767) {
            TL_tmp = 32767;
          }

          for (c_k = (int16_T)TL_tmp; c_k <= n; c_k++) {
            TL_tmp = (c_k - 1) * 3;
            b_H[ACC_B.i_c] = b_H[((b_j - 1) * 3 + (int16_T)ACC_B.e_i) - 1] -
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
          ACC_B.i_c = ((int16_T)ACC_B.f_i + 3 * ACC_B.c_i) - 1;
          D[ACC_B.i_c] = 0.0;
          for (b_j = (int16_T)ACC_B.e_i; b_j <= nA; b_j++) {
            TL_tmp = (b_j - 1) * 3;
            D[ACC_B.i_c] = ACC_B.TL[(TL_tmp + (int16_T)ACC_B.f_i) - 1] * RLinv
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
  for (ACC_B.i_k = 0; ACC_B.i_k < 98; ACC_B.i_k++) {
    lambda[ACC_B.i_k] = 0.0;
    ACC_B.cTol[ACC_B.i_k] = 1.0;
    ACC_B.iC[ACC_B.i_k] = 0;
  }

  nA = 0;
  for (ACC_B.i_k = 0; ACC_B.i_k < 98; ACC_B.i_k++) {
    if (iA[ACC_B.i_k] == 1) {
      ACC_B.idx = nA + 1;
      if (ACC_B.idx > 32767) {
        ACC_B.idx = 32767;
      }

      nA = (int16_T)ACC_B.idx;
      ACC_B.iC[(int16_T)ACC_B.idx - 1] = (int16_T)(1 + ACC_B.i_k);
    }
  }

  guard1 = false;
  if (nA > 0) {
    for (ACC_B.i_k = 0; ACC_B.i_k < 6; ACC_B.i_k++) {
      ACC_B.Opt[ACC_B.i_k] = 0.0;
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
            ACC_B.i_k = nA - 1;
            if (ACC_B.j - 1 <= ACC_B.i_k) {
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
                while (ACC_B.Opt_tmp - 1 <= ACC_B.i_k) {
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
            while (ACC_B.b_k - 1 <= ACC_B.i_k) {
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
          while (ACC_B.b_k - 1 <= ACC_B.i_k) {
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
            while (ACC_B.Opt_tmp - 1 <= ACC_B.i_k) {
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
          while (ACC_B.b_k - 1 <= ACC_B.i_k) {
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
        for (ACC_B.i_k = 0; ACC_B.i_k < 98; ACC_B.i_k++) {
          if (!cTolComputed) {
            ACC_B.idx = 1 + ACC_B.i_k;
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

            if ((!(ACC_B.cTol[ACC_B.i_k] > ACC_B.cVal)) && (!rtIsNaN(ACC_B.cVal)))
            {
              ACC_B.cTol[ACC_B.i_k] = ACC_B.cVal;
            }
          }

          if (iA[ACC_B.i_k] == 0) {
            ACC_B.idx = 1 + ACC_B.i_k;
            ACC_B.cVal = (((b_Ac[ACC_B.idx - 1] * x[0] + b_Ac[ACC_B.idx + 97] *
                            x[1]) + b_Ac[ACC_B.idx + 195] * x[2]) - b[ACC_B.i_k])
              / ACC_B.cTol[ACC_B.i_k];
            if (ACC_B.cVal < ACC_B.cMin) {
              ACC_B.cMin = ACC_B.cVal;
              kNext = (int16_T)(1 + ACC_B.i_k);
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
                    ACC_B.i_k = kNext - 1;
                    ACC_B.AcRow[ACC_B.idx] = ACC_B.U[ACC_B.idx + 6] * b_Ac[kNext
                      + 195] + (ACC_B.U[ACC_B.idx + 3] * b_Ac[kNext + 97] +
                                b_Ac[ACC_B.i_k] * ACC_B.U[ACC_B.idx]);
                  }

                  ACC_B.idx = 1;
                  while (ACC_B.idx - 1 <= nA - 1) {
                    ACC_B.i_k = ((int16_T)ACC_B.idx - 1) * 3;
                    ACC_B.r[(int16_T)ACC_B.idx - 1] = (ACC_B.D[ACC_B.i_k + 1] *
                      b_Ac[kNext + 97] + ACC_B.D[ACC_B.i_k] * b_Ac[kNext - 1]) +
                      ACC_B.D[ACC_B.i_k + 2] * b_Ac[kNext + 195];
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
                  ACC_B.i_k = 1;
                  while (ACC_B.i_k - 1 <= nA - 1) {
                    ACC_B.idx = (int16_T)ACC_B.i_k - 1;
                    if (ACC_B.r[ACC_B.idx] > 1.0E-12) {
                      ACC_B.cVal = lambda[ACC_B.iC[ACC_B.idx] - 1] / ACC_B.r
                        [(int16_T)ACC_B.i_k - 1];
                      if ((kDrop == 0) || (ACC_B.cVal < ACC_B.rMin)) {
                        ACC_B.rMin = ACC_B.cVal;
                        kDrop = (int16_T)ACC_B.i_k;
                      }
                    }

                    ACC_B.i_k++;
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
                    ACC_B.i_k = (int16_T)ACC_B.idx - 1;
                    ACC_B.e_k = ACC_B.iC[ACC_B.i_k] - 1;
                    lambda[ACC_B.e_k] -= ACC_B.r[ACC_B.i_k] * ACC_B.t;
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
                          ACC_B.i_k = kDrop - 2;
                          if (ACC_B.iC[ACC_B.idx] > ACC_B.iC[ACC_B.i_k]) {
                            exitg4 = true;
                          } else {
                            ACC_B.iC[ACC_B.idx] = ACC_B.iC[ACC_B.i_k];
                            ACC_B.iC[ACC_B.i_k] = tmp;
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
                ACC_abs_a(b, ACC_B.varargin_1);
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
  static const real_T b_Ac[294] = { -0.0012143788966212739, -0.00461490370736937,
    -0.0098783391806144955, -0.016729242404305397, -0.024932896411722829,
    -0.034289288597512, -0.044627979462304873, -0.055803730045320465,
    -0.067692775868213345, -0.080189651799466827, -0.093204486382306453,
    -0.10666069621303653, -0.12049302121985989, -0.13464585043792704,
    -0.14907179532894313, -0.16373047404433197, -0.17858747544264242,
    -0.1936134762834156, -0.20878348894942217, -0.22407622039783906,
    -0.2394735258944764, -0.25495994351679185, -0.27052229748352546,
    -0.28614936013451187, -0.30183156388887639, -0.31756075579200138,
    -0.3333299883542476, -0.34913334131547008, -0.36496576976275724,
    -0.38082297470490556, -3.2810551689427227E-5, 0.0012143788966212739,
    -0.00025254814631543021, 0.00461490370736937, -0.00082083040969290421,
    0.0098783391806144955, -0.0018753787978474771, 0.016729242404305397,
    -0.003533551794138771, 0.024932896411722829, -0.0058953557012441831,
    0.034289288597512, -0.0090460102688477324, 0.044627979462304873,
    -0.013058134977339909, 0.055803730045320465, -0.017993612065893422,
    0.067692775868213345, -0.023905174100266623, 0.080189651799466827,
    -0.030837756808846737, 0.093204486382306453, -0.038829651893481615,
    0.10666069621303653, -0.047913489390069841, 0.12049302121985989,
    -0.058117074781036157, 0.13464585043792704, -0.069464102335528,
    0.14907179532894313, -0.081974762977833446, 0.16373047404433197,
    -0.095666262278678083, 0.17858747544264242, -0.11055326185829133,
    0.1936134762834156, -0.12664825552528786, 0.20878348894942217,
    -0.14396188980107921, 0.22407622039783906, -0.16250323705276032,
    0.2394735258944764, -0.18228002824160236, 0.25495994351679185,
    -0.20329885125823532, 0.27052229748352546, -0.2255653199327419,
    0.28614936013451187, -0.24908421805555936, 0.30183156388887639,
    -0.27385962210399661, 0.31756075579200138, -0.29989500582287321,
    0.3333299883542476, -0.32719332934226164, 0.34913334131547008,
    -0.35575711511861774, 0.36496576976275724, -0.38558851264754324,
    0.38082297470490556, -1.0, -1.0, 1.0, 1.0, -1.0, -0.0, 1.0, 0.0, -0.0,
    -0.0012143788966212739, -0.00461490370736937, -0.0098783391806144955,
    -0.016729242404305397, -0.024932896411722829, -0.034289288597512,
    -0.044627979462304873, -0.055803730045320465, -0.067692775868213345,
    -0.080189651799466827, -0.093204486382306453, -0.10666069621303653,
    -0.12049302121985989, -0.13464585043792704, -0.14907179532894313,
    -0.16373047404433197, -0.17858747544264242, -0.1936134762834156,
    -0.20878348894942217, -0.22407622039783906, -0.2394735258944764,
    -0.25495994351679185, -0.27052229748352546, -0.28614936013451187,
    -0.30183156388887639, -0.31756075579200138, -0.3333299883542476,
    -0.34913334131547008, -0.36496576976275724, 0.0, 0.0, -3.2810551689427227E-5,
    0.0012143788966212739, -0.00025254814631543021, 0.00461490370736937,
    -0.00082083040969290421, 0.0098783391806144955, -0.0018753787978474771,
    0.016729242404305397, -0.003533551794138771, 0.024932896411722829,
    -0.0058953557012441831, 0.034289288597512, -0.0090460102688477324,
    0.044627979462304873, -0.013058134977339909, 0.055803730045320465,
    -0.017993612065893422, 0.067692775868213345, -0.023905174100266623,
    0.080189651799466827, -0.030837756808846737, 0.093204486382306453,
    -0.038829651893481615, 0.10666069621303653, -0.047913489390069841,
    0.12049302121985989, -0.058117074781036157, 0.13464585043792704,
    -0.069464102335528, 0.14907179532894313, -0.081974762977833446,
    0.16373047404433197, -0.095666262278678083, 0.17858747544264242,
    -0.11055326185829133, 0.1936134762834156, -0.12664825552528786,
    0.20878348894942217, -0.14396188980107921, 0.22407622039783906,
    -0.16250323705276032, 0.2394735258944764, -0.18228002824160236,
    0.25495994351679185, -0.20329885125823532, 0.27052229748352546,
    -0.2255653199327419, 0.28614936013451187, -0.24908421805555936,
    0.30183156388887639, -0.27385962210399661, 0.31756075579200138,
    -0.29989500582287321, 0.3333299883542476, -0.32719332934226164,
    0.34913334131547008, -0.35575711511861774, 0.36496576976275724, -0.0, -1.0,
    0.0, 1.0, -0.0, -1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 2.0, 2.0 };

  static const real_T b_Hinv[9] = { 0.0017855013718207403,
    -0.00067172895677734953, 0.0, -0.00067172895677734953, 0.0018674590069849132,
    0.0, 0.0, 0.0, 1.0000000000000001E-11 };

  static const real_T b_Linv[9] = { 0.039292226260900495, -0.015544205585456656,
    0.0, 0.0, 0.043214106573952367, 0.0, 0.0, 0.0, 3.1622776601683796E-6 };

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

  static const real_T a[8] = { -0.095314511416483, 0.19429596929464307,
    0.95536137489535289, 0.0029638232952471973, 0.015841427351331497,
    -0.18358838289695409, -0.082702092616065689, 0.068929470890852967 };

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
    -6.1080642912404579E-5, -5.2049490483862194E-5, -0.00046756239306928863,
    0.0053894305341981369, -0.0008659927822774608, 0.0045925697557817873,
    -0.0012055127637765944, 0.0039135297927835148, -0.0014948326072410035,
    0.00333489010585469, -0.001741374714873876, 0.0028418058905889387,
    -0.0019514640406118669, 0.0024216272391129503, -0.0021304903546675944,
    0.0020635746110014884, -0.0022830465162516967, 0.0017584622878332777,
    -0.0024130463018141144, 0.0014984627167084355, -0.0025238248116480678,
    0.0012769056970405226, -0.0026182240307540024, 0.0010881072588286456,
    -0.0026986657389983849, 0.00092722384233987376, -0.0027672136410526668,
    0.00079012803823130275, -0.002825626310034887, 0.00067330270026685543,
    -0.002875402303105025, 0.00057375071412657313, -0.0029178186064393674,
    0.00048891810745788121, -0.0029539633958766335, 0.00041662852858334175,
    -0.0029847639536990907, 0.00035502741293842, -0.0030110104577441917,
    0.00030253440484821054, -0.0030333762531483009, 0.00025780281403998549,
    -0.0030524351267872006, 0.00021968506676217842, -0.0030686760275832805,
    0.00018720326517001143, -0.0030825156103238762, 0.00015952409968881352,
    -0.0030943089247981582, 0.00013593747074024278, -0.0031043585244787419,
    0.00011583827137906757, -0.0031129222284281477, 9.8710863480248466E-5,
    -0.0031202197355591787, 8.4115849218178949E-5, -0.0031264382609358233,
    7.1678798464883147E-5, -0.0031317373387120587, 6.1080642912404579E-5,
    -0.0031362529149263261, 5.2049490483862194E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.013116672012841245, 0.013515102371038637, 0.013854622326112127,
    0.014143942147058089, 0.014390484235502005, 0.014600573544888248,
    0.014779599845009934, 0.014932155994720232, 0.015062155770164461,
    0.015172934271376261, 0.015267333483134884, 0.015347775185118298,
    0.015416323081837336, 0.015474735746273163, 0.015524511735469119,
    0.015566928035502104, 0.015603072822126137, 0.015633873377551316,
    0.015660119879553593, 0.015682485673216923, 0.015701544545372429,
    0.015717785444904445, 0.015731625026567875, 0.015743418340124261,
    0.015753467939022662, 0.015762031642305538, 0.01576932914886859,
    0.015775547673761232, 0.015780846751125031, 0.015785362326987844,
    0.0010199704063584079, -0.013116672012841245, 0.0020856661928760297,
    -0.013515102371038637, 0.0031808171809556023, -0.013854622326112127,
    0.0043010682360989389, -0.014143942147058089, 0.005442708157493298,
    -0.014390484235502005, 0.0066025744684164948, -0.014600573544888248,
    0.0077779722839719685, -0.014779599845009934, 0.0089666051747331367,
    -0.014932155994720232, 0.01016651625262734, -0.015062155770164461,
    0.011376037967637757, -0.015172934271376261, 0.012593749327374764,
    -0.015267333483134884, 0.013818439441999371, -0.015347775185118298,
    0.015049076459256169, -0.015416323081837336, 0.016284781092654573,
    -0.015474735746273163, 0.017524804063672912, -0.015524511735469119,
    0.018768506879272735, -0.015566928035502104, 0.020015345451577033,
    -0.015603072822126137, 0.021264856139480759, -0.015633873377551316,
    0.022516643854095936, -0.015660119879553593, 0.023770371922880586,
    -0.015682485673216923, 0.025025753452419152, -0.015701544545372429,
    0.02628254396826946, -0.015717785444904445, 0.02754053514305406,
    -0.015731625026567875, 0.028799549451892184, -0.015743418340124261,
    0.030059435618059297, -0.015753467939022662, 0.031320064732034171,
    -0.015762031642305538, 0.032581326944368959, -0.01576932914886859,
    0.033843128647538953, -0.015775547673761232, 0.035105390074473365,
    -0.015780846751125031, 0.036368043252158276, -0.015785362326987844, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.1751006590553847E-6, 5.3322988574456767E-6,
    5.4662543258406428E-6, 5.5804036462314719E-6, 5.6776752806172312E-6,
    5.7605646997016477E-6, 5.8311984033454509E-6, 5.8913884751971984E-6,
    5.9426790710830946E-6, 5.9863860337996368E-6, 6.0236306506131166E-6,
    6.05536841950315E-6, 6.0824135621384368E-6, 6.105459912456802E-6,
    6.1250987167389363E-6, 6.14183380183068E-6, 6.15609450064943E-6,
    6.1682466665741454E-6, 6.178602059289379E-6, 6.1874263428739714E-6,
    6.1949459013226579E-6, 6.2013536463504745E-6, 6.206813966477207E-6,
    6.2114669443589693E-6, 6.21543195056111E-6, 6.2188107059694773E-6,
    6.2216898914051536E-6, 6.2241433713914463E-6, 6.2262340891231189E-6,
    6.2280156812526453E-6, 0.028284675871819192, -5.1751006590553847E-6,
    0.028285096335406431, -5.3322988574456767E-6, 0.028285528420358669,
    -5.4662543258406428E-6, 0.028285970408384906, -5.5804036462314719E-6,
    0.028286420835254147, -5.6776752806172312E-6, 0.028286878453231037,
    -5.7605646997016477E-6, 0.028287342199065649, -5.8311984033454509E-6,
    0.028287811166716154, -5.8913884751971984E-6, 0.028288284584104643,
    -5.9426790710830946E-6, 0.028288761793309719, -5.9863860337996368E-6,
    0.028289242233687745, -6.0236306506131166E-6, 0.028289725427489733,
    -6.05536841950315E-6, 0.028290210967604847, -6.0824135621384368E-6,
    0.028290698507116119, -6.105459912456802E-6, 0.028291187750400414,
    -6.1250987167389363E-6, 0.028291678445544303, -6.14183380183068E-6,
    0.028292170377881329, -6.15609450064943E-6, 0.0282926633644848,
    -6.1682466665741454E-6, 0.028293157249474874, -6.178602059289379E-6,
    0.028293651900019513, -6.1874263428739714E-6, 0.028294147202926719,
    -6.1949459013226579E-6, 0.028294643061740638, -6.2013536463504745E-6,
    0.028295139394267008, -6.206813966477207E-6, 0.0282956361304645,
    -6.2114669443589693E-6, 0.02829613321064783, -6.21543195056111E-6,
    0.02829663058395656, -6.2188107059694773E-6, 0.028297128207050274,
    -6.2216898914051536E-6, 0.028297626042996717, -6.2241433713914463E-6,
    0.028298124060324285, -6.2262340891231189E-6, 0.028298622232214653,
    -6.2280156812526453E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
    1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
    1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T b_Mu1[98] = { -0.0012143788966212739, -0.00461490370736937,
    -0.0098783391806144955, -0.016729242404305397, -0.024932896411722829,
    -0.034289288597512, -0.044627979462304873, -0.055803730045320465,
    -0.067692775868213345, -0.080189651799466827, -0.093204486382306453,
    -0.10666069621303653, -0.12049302121985989, -0.13464585043792704,
    -0.14907179532894313, -0.16373047404433197, -0.17858747544264242,
    -0.1936134762834156, -0.20878348894942217, -0.22407622039783906,
    -0.2394735258944764, -0.25495994351679185, -0.27052229748352546,
    -0.28614936013451187, -0.30183156388887639, -0.31756075579200138,
    -0.3333299883542476, -0.34913334131547008, -0.36496576976275724,
    -0.38082297470490556, -3.2810551689427227E-5, 0.0012143788966212739,
    -0.00025254814631543021, 0.00461490370736937, -0.00082083040969290421,
    0.0098783391806144955, -0.0018753787978474771, 0.016729242404305397,
    -0.003533551794138771, 0.024932896411722829, -0.0058953557012441831,
    0.034289288597512, -0.0090460102688477324, 0.044627979462304873,
    -0.013058134977339909, 0.055803730045320465, -0.017993612065893422,
    0.067692775868213345, -0.023905174100266623, 0.080189651799466827,
    -0.030837756808846737, 0.093204486382306453, -0.038829651893481615,
    0.10666069621303653, -0.047913489390069841, 0.12049302121985989,
    -0.058117074781036157, 0.13464585043792704, -0.069464102335528,
    0.14907179532894313, -0.081974762977833446, 0.16373047404433197,
    -0.095666262278678083, 0.17858747544264242, -0.11055326185829133,
    0.1936134762834156, -0.12664825552528786, 0.20878348894942217,
    -0.14396188980107921, 0.22407622039783906, -0.16250323705276032,
    0.2394735258944764, -0.18228002824160236, 0.25495994351679185,
    -0.20329885125823532, 0.27052229748352546, -0.2255653199327419,
    0.28614936013451187, -0.24908421805555936, 0.30183156388887639,
    -0.27385962210399661, 0.31756075579200138, -0.29989500582287321,
    0.3333299883542476, -0.32719332934226164, 0.34913334131547008,
    -0.35575711511861774, 0.36496576976275724, -0.38558851264754324,
    0.38082297470490556, -1.0, -1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T b_Mv[6076] = { 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 2.7105054312137611E-20,
    2.3716922523120409E-20, 2.3716922523120409E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 2.7105054312137611E-20, 2.7105054312137611E-20,
    2.3716922523120409E-20, 2.3716922523120409E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 2.3716922523120409E-20, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.07999999999999996, -1.6940658945086007E-20, 0.07999999999999996,
    -2.0328790734103208E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.7105054312137611E-20, 0.079999999999999946,
    -2.3716922523120409E-20, 0.07999999999999996, -2.3716922523120409E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.079999999999999946,
    -2.7105054312137611E-20, 0.079999999999999932, -2.7105054312137611E-20,
    0.079999999999999932, -2.7105054312137611E-20, 0.079999999999999918,
    -2.3716922523120409E-20, 0.079999999999999918, -2.3716922523120409E-20,
    0.0799999999999999, -1.6940658945086007E-20, 0.0799999999999999,
    -2.0328790734103208E-20, 0.0799999999999999, -2.0328790734103208E-20,
    0.079999999999999891, -2.0328790734103208E-20, 0.079999999999999891,
    -2.3716922523120409E-20, 0.079999999999999891, -2.3716922523120409E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 2.7105054312137611E-20,
    2.3716922523120409E-20, 2.3716922523120409E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 2.7105054312137611E-20, 2.7105054312137611E-20,
    2.3716922523120409E-20, 2.3716922523120409E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.07999999999999996, -1.6940658945086007E-20,
    0.07999999999999996, -2.0328790734103208E-20, 0.079999999999999946,
    -2.3716922523120409E-20, 0.079999999999999946, -2.7105054312137611E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.07999999999999996,
    -2.3716922523120409E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.7105054312137611E-20, 0.079999999999999932,
    -2.7105054312137611E-20, 0.079999999999999932, -2.7105054312137611E-20,
    0.079999999999999918, -2.3716922523120409E-20, 0.079999999999999918,
    -2.3716922523120409E-20, 0.0799999999999999, -1.6940658945086007E-20,
    0.0799999999999999, -2.0328790734103208E-20, 0.0799999999999999,
    -2.0328790734103208E-20, 0.079999999999999891, -2.0328790734103208E-20,
    0.079999999999999891, -2.3716922523120409E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 2.7105054312137611E-20, 2.3716922523120409E-20,
    2.3716922523120409E-20, 2.3716922523120409E-20, 2.7105054312137611E-20,
    2.7105054312137611E-20, 2.7105054312137611E-20, 2.3716922523120409E-20,
    2.3716922523120409E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 2.0328790734103208E-20, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.07999999999999996, -1.6940658945086007E-20, 0.07999999999999996,
    -2.0328790734103208E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.7105054312137611E-20, 0.079999999999999946,
    -2.3716922523120409E-20, 0.07999999999999996, -2.3716922523120409E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.079999999999999946,
    -2.7105054312137611E-20, 0.079999999999999932, -2.7105054312137611E-20,
    0.079999999999999932, -2.7105054312137611E-20, 0.079999999999999918,
    -2.3716922523120409E-20, 0.079999999999999918, -2.3716922523120409E-20,
    0.0799999999999999, -1.6940658945086007E-20, 0.0799999999999999,
    -2.0328790734103208E-20, 0.0799999999999999, -2.0328790734103208E-20,
    0.079999999999999891, -2.0328790734103208E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 2.7105054312137611E-20, 2.3716922523120409E-20,
    2.3716922523120409E-20, 2.3716922523120409E-20, 2.7105054312137611E-20,
    2.7105054312137611E-20, 2.7105054312137611E-20, 2.3716922523120409E-20,
    2.3716922523120409E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.0328790734103208E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.07999999999999996, -1.6940658945086007E-20, 0.07999999999999996,
    -2.0328790734103208E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.7105054312137611E-20, 0.079999999999999946,
    -2.3716922523120409E-20, 0.07999999999999996, -2.3716922523120409E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.079999999999999946,
    -2.7105054312137611E-20, 0.079999999999999932, -2.7105054312137611E-20,
    0.079999999999999932, -2.7105054312137611E-20, 0.079999999999999918,
    -2.3716922523120409E-20, 0.079999999999999918, -2.3716922523120409E-20,
    0.0799999999999999, -1.6940658945086007E-20, 0.0799999999999999,
    -2.0328790734103208E-20, 0.0799999999999999, -2.0328790734103208E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 2.7105054312137611E-20,
    2.3716922523120409E-20, 2.3716922523120409E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 2.7105054312137611E-20, 2.7105054312137611E-20,
    2.3716922523120409E-20, 2.3716922523120409E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.07999999999999996, -1.6940658945086007E-20, 0.07999999999999996,
    -2.0328790734103208E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.7105054312137611E-20, 0.079999999999999946,
    -2.3716922523120409E-20, 0.07999999999999996, -2.3716922523120409E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.079999999999999946,
    -2.7105054312137611E-20, 0.079999999999999932, -2.7105054312137611E-20,
    0.079999999999999932, -2.7105054312137611E-20, 0.079999999999999918,
    -2.3716922523120409E-20, 0.079999999999999918, -2.3716922523120409E-20,
    0.0799999999999999, -1.6940658945086007E-20, 0.0799999999999999,
    -2.0328790734103208E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 2.3716922523120409E-20, 2.3716922523120409E-20,
    2.3716922523120409E-20, 2.7105054312137611E-20, 2.7105054312137611E-20,
    2.7105054312137611E-20, 2.3716922523120409E-20, 2.3716922523120409E-20,
    1.6940658945086007E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.08, -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.07999999999999996, -1.6940658945086007E-20, 0.07999999999999996,
    -2.0328790734103208E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.7105054312137611E-20, 0.079999999999999946,
    -2.3716922523120409E-20, 0.07999999999999996, -2.3716922523120409E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.079999999999999946,
    -2.7105054312137611E-20, 0.079999999999999932, -2.7105054312137611E-20,
    0.079999999999999932, -2.7105054312137611E-20, 0.079999999999999918,
    -2.3716922523120409E-20, 0.079999999999999918, -2.3716922523120409E-20,
    0.0799999999999999, -1.6940658945086007E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 2.7105054312137611E-20,
    2.3716922523120409E-20, 2.3716922523120409E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 2.7105054312137611E-20, 2.7105054312137611E-20,
    2.3716922523120409E-20, 2.3716922523120409E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.07999999999999996, -1.6940658945086007E-20,
    0.07999999999999996, -2.0328790734103208E-20, 0.079999999999999946,
    -2.3716922523120409E-20, 0.079999999999999946, -2.7105054312137611E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.07999999999999996,
    -2.3716922523120409E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.7105054312137611E-20, 0.079999999999999932,
    -2.7105054312137611E-20, 0.079999999999999932, -2.7105054312137611E-20,
    0.079999999999999918, -2.3716922523120409E-20, 0.079999999999999918,
    -2.3716922523120409E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 2.7105054312137611E-20, 2.3716922523120409E-20,
    2.3716922523120409E-20, 2.3716922523120409E-20, 2.7105054312137611E-20,
    2.7105054312137611E-20, 2.7105054312137611E-20, 2.3716922523120409E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.07999999999999996, -1.6940658945086007E-20, 0.07999999999999996,
    -2.0328790734103208E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.7105054312137611E-20, 0.079999999999999946,
    -2.3716922523120409E-20, 0.07999999999999996, -2.3716922523120409E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.079999999999999946,
    -2.7105054312137611E-20, 0.079999999999999932, -2.7105054312137611E-20,
    0.079999999999999932, -2.7105054312137611E-20, 0.079999999999999918,
    -2.3716922523120409E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 2.7105054312137611E-20, 2.3716922523120409E-20,
    2.3716922523120409E-20, 2.3716922523120409E-20, 2.7105054312137611E-20,
    2.7105054312137611E-20, 2.7105054312137611E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.07999999999999996, -1.6940658945086007E-20, 0.07999999999999996,
    -2.0328790734103208E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.7105054312137611E-20, 0.079999999999999946,
    -2.3716922523120409E-20, 0.07999999999999996, -2.3716922523120409E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.079999999999999946,
    -2.7105054312137611E-20, 0.079999999999999932, -2.7105054312137611E-20,
    0.079999999999999932, -2.7105054312137611E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 2.7105054312137611E-20,
    2.3716922523120409E-20, 2.3716922523120409E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 2.7105054312137611E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.07999999999999996, -1.6940658945086007E-20, 0.07999999999999996,
    -2.0328790734103208E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.7105054312137611E-20, 0.079999999999999946,
    -2.3716922523120409E-20, 0.07999999999999996, -2.3716922523120409E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.079999999999999946,
    -2.7105054312137611E-20, 0.079999999999999932, -2.7105054312137611E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 2.7105054312137611E-20, 2.3716922523120409E-20,
    2.3716922523120409E-20, 2.3716922523120409E-20, 2.7105054312137611E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.07999999999999996, -1.6940658945086007E-20,
    0.07999999999999996, -2.0328790734103208E-20, 0.079999999999999946,
    -2.3716922523120409E-20, 0.079999999999999946, -2.7105054312137611E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.07999999999999996,
    -2.3716922523120409E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.7105054312137611E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 2.3716922523120409E-20, 2.3716922523120409E-20,
    2.3716922523120409E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.07999999999999996, -1.6940658945086007E-20, 0.07999999999999996,
    -2.0328790734103208E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.7105054312137611E-20, 0.079999999999999946,
    -2.3716922523120409E-20, 0.07999999999999996, -2.3716922523120409E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 2.3716922523120409E-20, 2.3716922523120409E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.07999999999999996, -1.6940658945086007E-20,
    0.07999999999999996, -2.0328790734103208E-20, 0.079999999999999946,
    -2.3716922523120409E-20, 0.079999999999999946, -2.7105054312137611E-20,
    0.079999999999999946, -2.3716922523120409E-20, 0.07999999999999996,
    -2.3716922523120409E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 2.7105054312137611E-20,
    2.3716922523120409E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.08, -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.07999999999999996, -1.6940658945086007E-20, 0.07999999999999996,
    -2.0328790734103208E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.7105054312137611E-20, 0.079999999999999946,
    -2.3716922523120409E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    2.7105054312137611E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20,
    0.08, -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.07999999999999996, -1.6940658945086007E-20, 0.07999999999999996,
    -2.0328790734103208E-20, 0.079999999999999946, -2.3716922523120409E-20,
    0.079999999999999946, -2.7105054312137611E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.07999999999999996, -1.6940658945086007E-20,
    0.07999999999999996, -2.0328790734103208E-20, 0.079999999999999946,
    -2.3716922523120409E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 2.0328790734103208E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.07999999999999996, -1.6940658945086007E-20, 0.07999999999999996,
    -2.0328790734103208E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.6940658945086007E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20,
    0.08, -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.07999999999999996, -1.6940658945086007E-20, 0.0,
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
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20,
    0.079999999999999974, -1.3552527156068805E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20,
    0.08, -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999974,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999974, -1.3552527156068805E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.079999999999999988, -1.3552527156068805E-20,
    0.079999999999999988, -1.3552527156068805E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
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
    -0.0, -0.0, -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.079999999999999988,
    -1.3552527156068805E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20,
    1.3552527156068805E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20,
    1.3552527156068805E-20, 1.3552527156068805E-20, 1.3552527156068805E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.08,
    -1.3552527156068805E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.1858461261560205E-20, 1.3552527156068805E-20, 1.3552527156068805E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20, 0.08,
    -1.3552527156068805E-20, 0.08, -1.3552527156068805E-20, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20, 1.3552527156068805E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
    -1.1858461261560205E-20, 0.08, -1.3552527156068805E-20, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 1.1858461261560205E-20, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08, -1.1858461261560205E-20,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0 };

  static const real_T b_a[8] = { -2.7529251322709746E-20, 0.00632455532033672,
    -1.1159380919009811E-5, -0.012649109656163445, 0.028284269046030115,
    -4.99062686434405E-6, 0.0, 1.0 };

  static const real_T b_Kx[8] = { 0.35875722517480962, -15.352761356806186,
    -0.0070601277709030508, 982.27157961646458, 0.3024065960592725,
    -14.200647826702099, -0.0064965076957434848, 907.63027657430314 };

  static const real_T b_Kr[120] = { 3.2810551689427232E-7, -0.23801826373776969,
    2.5254814631543027E-6, -0.90452112664439643, 8.208304096929043E-6,
    -1.9361544794004411, 1.8753787978474776E-5, -3.2789315112438575,
    3.5335517941387715E-5, -4.8868476966976742, 5.8953557012441843E-5,
    -6.7207005651123515, 9.0460102688477348E-5, -8.7470839746117548,
    0.00013058134977339913, -10.93753108888281, 0.00017993612065893424,
    -13.267784070169816, 0.00023905174100266626, -15.717171752695497,
    0.00030837756808846741, -18.268079330932064, 0.00038829651893481623,
    -20.905496457755159, 0.00047913489390069851, -23.616632159092539,
    0.0005811707478103617, -26.3905866858337, 0.00069464102335528,
    -29.218071884472852, 0.00081974762977833464, -32.091172912689068,
    0.000956662622786781, -35.003145186757912, 0.0011055326185829134,
    -37.948241351549456, 0.0012664825552528788, -40.921563834086747,
    0.0014396188980107924, -43.918939197976457, 0.0016250323705276036,
    -46.936811075317372, 0.001822800282416024, -49.9721489292912,
    0.0020329885125823536, -53.022370306770988, 0.0022556531993274194,
    -56.085274586364328, 0.002490842180555594, -59.15898652221977,
    0.0027385962210399665, -62.241908135232272, 0.0029989500582287328,
    -65.332677717432531, 0.0032719332934226172, -68.430134897832133,
    0.003557571151186178, -71.53329087350042, 0.0038558851264754333,
    -74.6413030421615, -0.0, -0.0, 3.2810551689427232E-7, -0.23801826373776969,
    2.5254814631543027E-6, -0.90452112664439643, 8.208304096929043E-6,
    -1.9361544794004411, 1.8753787978474776E-5, -3.2789315112438575,
    3.5335517941387715E-5, -4.8868476966976742, 5.8953557012441843E-5,
    -6.7207005651123515, 9.0460102688477348E-5, -8.7470839746117548,
    0.00013058134977339913, -10.93753108888281, 0.00017993612065893424,
    -13.267784070169816, 0.00023905174100266626, -15.717171752695497,
    0.00030837756808846741, -18.268079330932064, 0.00038829651893481623,
    -20.905496457755159, 0.00047913489390069851, -23.616632159092539,
    0.0005811707478103617, -26.3905866858337, 0.00069464102335528,
    -29.218071884472852, 0.00081974762977833464, -32.091172912689068,
    0.000956662622786781, -35.003145186757912, 0.0011055326185829134,
    -37.948241351549456, 0.0012664825552528788, -40.921563834086747,
    0.0014396188980107924, -43.918939197976457, 0.0016250323705276036,
    -46.936811075317372, 0.001822800282416024, -49.9721489292912,
    0.0020329885125823536, -53.022370306770988, 0.0022556531993274194,
    -56.085274586364328, 0.002490842180555594, -59.15898652221977,
    0.0027385962210399665, -62.241908135232272, 0.0029989500582287328,
    -65.332677717432531, 0.0032719332934226172, -68.430134897832133,
    0.003557571151186178, -71.53329087350042 };

  static const real_T b_Kv[124] = { -0.002836324123231699, 0.0,
    -0.0028362978747903476, 0.0, -0.0028360958362732948, 0.0,
    -0.0028354391719455406, 0.0, -0.0028339388689072628, 0.0,
    -0.0028311120274719511, 0.0, -0.0028263957429109561, 0.0,
    -0.002819158934695878, 0.0, -0.0028087124267140054, 0.0,
    -0.00279431753706129, 0.0, -0.0027751933977810759, 0.0,
    -0.0027505231923339981, 0.0, -0.0027194594708192118, 0.0,
    -0.0026811286793071544, 0.0, -0.0026346350194823248, 0.0,
    -0.0025790637376139009, 0.0, -0.0025134839272316334, 0.0,
    -0.00243695091740869, 0.0, -0.0023485083079220559, 0.0,
    -0.0022471897035018259, 0.0, -0.0021320201916609619, 0.0,
    -0.002002017602018753, 0.0, -0.0018561935794254704, 0.0,
    -0.0016935544984188814, 0.0, -0.0015131022424726872, 0.0,
    -0.0013138348680282387, 0.0, -0.0010947471703450405, 0.0,
    -0.00085483116568674114, 0.0, -0.00059307650221293075, 0.0,
    -0.00030847081011803551, 0.0, 0.0, 0.0, -0.0025278533131136631, 0.0,
    -0.0025278533131136626, 0.0, -0.0025278270646723116, 0.0,
    -0.0025276250261552593, 0.0, -0.0025269683618275047, 0.0,
    -0.0025254680587892269, 0.0, -0.0025226412173539156, 0.0,
    -0.00251792493279292, 0.0, -0.002510688124577842, 0.0, -0.002500241616595969,
    0.0, -0.0024858467269432537, 0.0, -0.00246672258766304, 0.0,
    -0.0024420523822159617, 0.0, -0.0024109886607011754, 0.0,
    -0.0023726578691891185, 0.0, -0.0023261642093642888, 0.0,
    -0.0022705929274958649, 0.0, -0.0022050131171135975, 0.0,
    -0.0021284801072906541, 0.0, -0.0020400374978040204, 0.0,
    -0.0019387188933837904, 0.0, -0.0018235493815429262, 0.0,
    -0.0016935467919007175, 0.0, -0.0015477227693074349, 0.0,
    -0.0013850836883008456, 0.0, -0.0012046314323546515, 0.0,
    -0.001005364057910203, 0.0, -0.00078627636022700494, 0.0,
    -0.00054636035556870541, 0.0, -0.00028460569209489512, 0.0, 0.0, 0.0 };

  static const real_T c_a[16] = { 0.85214378896621124, 6.52213355998541E-6,
    -0.016530825651721811, 0.0, -0.073928099762894439, 0.99998561653740869,
    0.036455940405912865, 0.0, -2.916786799510717E-5, -5.6749049349282934E-9,
    1.0000143834625914, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T d_a[4] = { -1.8318475397195182, -1.0119283148112526,
    -0.0015592782385630672, 0.0 };

  static const real_T e_a[8] = { -3.556347466058661E-18, -0.0011159380919009794,
    2.8284269046030124, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T f_a[8] = { -0.095613466557918247, 0.19429254757027847,
    0.96403398614796942, 0.0029638232952471978, 0.02707392645563142,
    -0.183585638471085, -0.089658041178249617, 0.068929470890852967 };

  boolean_T b_varargout_1;
  boolean_T exitg1;
  boolean_T guard1 = false;

  // Outputs for Atomic SubSystem: '<Root>/target_output'
  // MATLABSystem: '<S5>/SourceBlock' incorporates:
  //   Inport: '<S41>/In1'

  b_varargout_1 = Sub_ACC_139.getLatestMessage(&ACC_B.b_varargout_2_m);

  // Outputs for Enabled SubSystem: '<S5>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S41>/Enable'

  if (b_varargout_1) {
    ACC_B.In1 = ACC_B.b_varargout_2_m;
  }

  // End of MATLABSystem: '<S5>/SourceBlock'
  // End of Outputs for SubSystem: '<S5>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/target_output'

  // Outputs for Atomic SubSystem: '<Root>/drive_ctrl_input'
  // MATLABSystem: '<S4>/SourceBlock' incorporates:
  //   Inport: '<S40>/In1'

  b_varargout_1 = Sub_ACC_140.getLatestMessage(&ACC_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S4>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S40>/Enable'

  if (b_varargout_1) {
    ACC_B.In1_p = ACC_B.b_varargout_2;
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

  if (ACC_B.In1_p.AccGapLevel == 1.0) {
    // Outputs for IfAction SubSystem: '<S7>/Near' incorporates:
    //   ActionPort: '<S13>/Action Port'

    ACC_B.Switch2 = ACC_P.Constant_Value_f;
    ACC_B.Merge1 = ACC_P.Constant3_Value_i;

    // End of Outputs for SubSystem: '<S7>/Near'
  } else if (ACC_B.In1_p.AccGapLevel == 3.0) {
    // Outputs for IfAction SubSystem: '<S7>/Time_Far' incorporates:
    //   ActionPort: '<S14>/Action Port'

    ACC_B.Switch2 = ACC_P.Constant2_Value;
    ACC_B.Merge1 = ACC_P.Constant6_Value;

    // End of Outputs for SubSystem: '<S7>/Time_Far'
  } else {
    // Outputs for IfAction SubSystem: '<S7>/Time_Medium' incorporates:
    //   ActionPort: '<S15>/Action Port'

    ACC_B.Switch2 = ACC_P.Constant1_Value;
    ACC_B.Merge1 = ACC_P.Constant4_Value_n;

    // End of Outputs for SubSystem: '<S7>/Time_Medium'
  }

  // End of If: '<S7>/If'

  // Sum: '<S1>/Sum6' incorporates:
  //   Chart: '<S6>/ACC Diagnostics'

  ACC_B.rtb_lead_velocity_tmp = ACC_B.In1_p.VehSpd + ACC_B.In1.ObjVx;

  // Switch: '<S8>/Switch1' incorporates:
  //   Constant: '<S8>/Constant3'
  //   Constant: '<S8>/Constant4'

  if (ACC_B.In1_p.VehSpd > ACC_P.Switch1_Threshold) {
    ACC_B.Switch1 = ACC_P.Constant4_Value;
  } else {
    ACC_B.Switch1 = ACC_P.Constant3_Value;
  }

  // End of Switch: '<S8>/Switch1'

  // Gain: '<S20>/umin_scale'
  ACC_B.umin_scale = ACC_P.umin_scale_Gain * ACC_B.Switch1;

  // Gain: '<S20>/umax_scale' incorporates:
  //   Constant: '<S8>/Constant2'

  ACC_B.umax_scale = ACC_P.umax_scale_Gain * ACC_P.Constant2_Value_m;

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

  // SignalConversion: '<S39>/TmpSignal ConversionAt SFunction Inport4' incorporates:
  //   Gain: '<S1>/km//h to m//s'
  //   MATLAB Function: '<S20>/optimizer'

  ACC_B.Switch2 = ACC_P.kmhtoms_Gain * ACC_B.In1_p.AccSpeedSetPoint;

  // MATLAB Function: '<S20>/optimizer' incorporates:
  //   Memory: '<S20>/last_x'
  //   SignalConversion: '<S39>/TmpSignal ConversionAt SFunction Inport4'
  //   Sum: '<S1>/Sum6'

  memset(&ACC_B.vseq[0], 0, 62U * sizeof(real_T));
  for (ACC_B.i = 0; ACC_B.i < 31; ACC_B.i++) {
    ACC_B.vseq[(ACC_B.i * (int32_T)ACC_nv + (int32_T)ACC_nv) - 1] = 1.0;
  }

  memset(&ACC_B.rseq[0], 0, 60U * sizeof(real_T));
  for (ACC_B.i = 0; ACC_B.i < 30; ACC_B.i++) {
    ACC_B.rseq_tmp = ACC_B.i * (int32_T)ACC_ny;
    ACC_B.rseq[ACC_B.rseq_tmp] = ACC_B.Merge1 * 0.02 - 0.97;
    ACC_B.rseq[1 + ACC_B.rseq_tmp] = ACC_B.Switch2 * 0.02 - 0.4;
  }

  for (ACC_B.i = 0; ACC_B.i < 31; ACC_B.i++) {
    ACC_B.vseq[ACC_B.i * (int32_T)ACC_nv] = ACC_RMDscale *
      ACC_B.rtb_lead_velocity_tmp - ACC_voff;
  }

  ACC_B.rtb_TmpSignalConversionAtSFun_c = ACC_B.vseq[0];
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
      ACC_B.rtb_TmpSignalConversionAtSFun_c));
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

  if (ACC_P.Constant1_Value_m != 0.0) {
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
        (-14.733191722230941 * (real_T)ACC_B.rseq_tmp + 247.71913547111902) *
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
  b_varargout_1 = ((ACC_B.In1.ObjVx > 0.0) && (ACC_B.In1.ObjDx > 1.5 *
    ACC_B.Merge1));
  if ((!b_varargout_1) || (!ACC_DW.condWasTrueAtLastTimeStep_1)) {
    ACC_DW.durationLastReferenceTick_1 = ACC_DW.chartAbsoluteTimeCounter;
  }

  ACC_DW.condWasTrueAtLastTimeStep_1 = b_varargout_1;
  b_varargout_1 = (ACC_B.In1.ObjDx > ACC_B.safe_distance);
  if ((!b_varargout_1) || (!ACC_DW.condWasTrueAtLastTimeStep_1_d)) {
    ACC_DW.durationLastReferenceTick_1_e = ACC_DW.chartAbsoluteTimeCounter;
  }

  ACC_DW.condWasTrueAtLastTimeStep_1_d = b_varargout_1;
  b_varargout_1 = (ACC_DW.Delay_DSTATE > 1.0);
  if ((!b_varargout_1) || (!ACC_DW.condWasTrueAtLastTimeStep_1_j)) {
    ACC_DW.durationLastReferenceTick_1_j = ACC_DW.chartAbsoluteTimeCounter;
  }

  ACC_DW.condWasTrueAtLastTimeStep_1_j = b_varargout_1;
  if (ACC_DW.is_active_c11_ACC == 0U) {
    ACC_DW.chartAbsoluteTimeCounter = 0;
    ACC_DW.is_active_c11_ACC = 1U;
    ACC_DW.is_c11_ACC = ACC_IN_Standard;
    ACC_DW.is_Standard = ACC_IN_ACC_Normal;
    ACC_DW.is_ACC_Normal = ACC_IN_ACC_Normal;
  } else {
    switch (ACC_DW.is_c11_ACC) {
     case ACC_IN_FullBrakes:
      b_varargout_1 = (ACC_B.In1.ObjDx > ACC_B.safe_distance);
      if ((!b_varargout_1) || (!ACC_DW.condWasTrueAtLastTimeStep_1_d)) {
        ACC_DW.durationLastReferenceTick_1_e = ACC_DW.chartAbsoluteTimeCounter;
      }

      ACC_DW.condWasTrueAtLastTimeStep_1_d = b_varargout_1;
      if ((ACC_DW.chartAbsoluteTimeCounter -
           ACC_DW.durationLastReferenceTick_1_e > 23) && (ACC_B.In1.ObjVx > 0.0))
      {
        ACC_DW.durationLastReferenceTick_1_j = ACC_DW.chartAbsoluteTimeCounter;
        ACC_DW.is_c11_ACC = ACC_IN_RateLimited;
        ACC_DW.condWasTrueAtLastTimeStep_1_j = (ACC_DW.Delay_DSTATE > 1.0);
      } else {
        ACC_B.ACC_Accel_Out = ACC_B.Switch1;
        ACC_B.RateLimiter = false;
      }
      break;

     case ACC_IN_RateLimited:
      b_varargout_1 = (ACC_DW.Delay_DSTATE > 1.0);
      if ((!b_varargout_1) || (!ACC_DW.condWasTrueAtLastTimeStep_1_j)) {
        ACC_DW.durationLastReferenceTick_1_j = ACC_DW.chartAbsoluteTimeCounter;
      }

      ACC_DW.condWasTrueAtLastTimeStep_1_j = b_varargout_1;
      if (ACC_DW.chartAbsoluteTimeCounter - ACC_DW.durationLastReferenceTick_1_j
          > 29) {
        ACC_DW.is_c11_ACC = ACC_IN_Standard;
        ACC_DW.is_Standard = ACC_IN_ACC_Normal;
        ACC_DW.is_ACC_Normal = ACC_IN_ACC_Normal;
      } else if (ACC_B.In1.ObjDx < ACC_B.safe_distance) {
        ACC_DW.durationLastReferenceTick_1_e = ACC_DW.chartAbsoluteTimeCounter;
        ACC_DW.is_c11_ACC = ACC_IN_FullBrakes;
        ACC_DW.condWasTrueAtLastTimeStep_1_d = (ACC_B.In1.ObjDx >
          ACC_B.safe_distance);
      } else {
        ACC_B.ACC_Accel_Out = ACC_B.umin_scale;
        ACC_B.RateLimiter = true;
      }
      break;

     default:
      if (((ACC_B.In1.ObjDx < 1.4 * ACC_B.In1_p.VehSpd + 9.9) &&
           (ACC_B.In1_p.VehSpd > 0.0)) || ((ACC_B.In1.ObjVx < -20.0) &&
           (ACC_B.In1.ObjDx < 255.0))) {
        ACC_DW.is_ACC_Normal = ACC_IN_NO_ACTIVE_CHILD;
        ACC_DW.is_Standard = ACC_IN_NO_ACTIVE_CHILD;
        ACC_DW.durationLastReferenceTick_1_e = ACC_DW.chartAbsoluteTimeCounter;
        ACC_DW.is_c11_ACC = ACC_IN_FullBrakes;
        ACC_DW.condWasTrueAtLastTimeStep_1_d = (ACC_B.In1.ObjDx >
          ACC_B.safe_distance);
      } else if (ACC_DW.is_Standard == ACC_IN_ACC_Normal) {
        if (ACC_B.In1.ObjDx < ACC_B.safe_distance) {
          ACC_DW.is_ACC_Normal = ACC_IN_NO_ACTIVE_CHILD;
          ACC_DW.is_Standard = ACC_IN_HalfBrakes;
        } else {
          ACC_B.RateLimiter = false;
          if (ACC_DW.is_ACC_Normal == ACC_IN_ACC_Normal) {
            if (((ACC_B.In1.ObjVx < 0.0) && (ACC_B.In1_p.VehSpd > 0.0) &&
                 (ACC_B.In1.ObjDx < 1.5 * ACC_B.Merge1)) ||
                ((ACC_B.rtb_lead_velocity_tmp < 1.0) && (ACC_B.In1.ObjDx < 2.0 *
                  ACC_B.safe_distance))) {
              ACC_DW.durationLastReferenceTick_1 =
                ACC_DW.chartAbsoluteTimeCounter;
              ACC_DW.is_ACC_Normal = ACC_IN_ACC_Zeroed;
              ACC_DW.condWasTrueAtLastTimeStep_1 = ((ACC_B.In1.ObjVx > 0.0) &&
                (ACC_B.In1.ObjDx > 1.5 * ACC_B.Merge1));
            } else {
              ACC_B.ACC_Accel_Out = ACC_B.umin_scale;
            }
          } else {
            b_varargout_1 = ((ACC_B.In1.ObjVx > 0.0) && (ACC_B.In1.ObjDx > 1.5 *
              ACC_B.Merge1));
            if ((!b_varargout_1) || (!ACC_DW.condWasTrueAtLastTimeStep_1)) {
              ACC_DW.durationLastReferenceTick_1 =
                ACC_DW.chartAbsoluteTimeCounter;
            }

            ACC_DW.condWasTrueAtLastTimeStep_1 = b_varargout_1;
            if (ACC_DW.chartAbsoluteTimeCounter -
                ACC_DW.durationLastReferenceTick_1 > 18) {
              ACC_DW.is_ACC_Normal = ACC_IN_ACC_Normal;
            } else if ((0.0 < ACC_B.umin_scale) || rtIsNaN(ACC_B.umin_scale)) {
              ACC_B.ACC_Accel_Out = 0.0;
            } else {
              ACC_B.ACC_Accel_Out = ACC_B.umin_scale;
            }
          }
        }
      } else if (ACC_B.In1.ObjDx >= 2.0 * ACC_B.safe_distance) {
        ACC_DW.is_Standard = ACC_IN_ACC_Normal;
        ACC_DW.is_ACC_Normal = ACC_IN_ACC_Normal;
      } else {
        ACC_B.ACC_Accel_Out = 0.5 * ACC_B.Switch1;
        if ((!(ACC_B.ACC_Accel_Out < ACC_B.umin_scale)) && (!rtIsNaN
             (ACC_B.umin_scale))) {
          ACC_B.ACC_Accel_Out = ACC_B.umin_scale;
        }

        ACC_B.RateLimiter = false;
      }
      break;
    }
  }

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
  //   Constant: '<S2>/Constant'

  ACC_B.BusAssign = ACC_P.Constant_Value_i;

  // Switch: '<S12>/Switch2' incorporates:
  //   Constant: '<S8>/Constant2'
  //   RelationalOperator: '<S12>/LowerRelop1'
  //   RelationalOperator: '<S12>/UpperRelop'
  //   Switch: '<S12>/Switch'

  if (ACC_B.Merge1 > ACC_P.Constant2_Value_m) {
    // BusAssignment: '<Root>/BusAssign'
    ACC_B.BusAssign.AccAccel = ACC_P.Constant2_Value_m;
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

  // BusAssignment: '<Root>/BusAssign' incorporates:
  //   Constant: '<S16>/Constant'
  //   Logic: '<S9>/NOT2'
  //   MATLAB Function: '<S20>/optimizer'
  //   RelationalOperator: '<S16>/Compare'

  ACC_B.BusAssign.AccFault = !(ACC_B.umax_incr > ACC_P.CompareToConstant_const);

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S3>/SinkBlock'
  Pub_ACC_136.publish(&ACC_B.BusAssign);

  // End of Outputs for SubSystem: '<Root>/Publish'
  for (ACC_B.i = 0; ACC_B.i < 4; ACC_B.i++) {
    // Update for Memory: '<S20>/last_x' incorporates:
    //   MATLAB Function: '<S20>/optimizer'

    ACC_DW.last_x_PreviousInput[ACC_B.i] = (((((c_a[ACC_B.i + 4] * ACC_B.xk[1] +
      c_a[ACC_B.i] * ACC_B.xk[0]) + c_a[ACC_B.i + 8] * ACC_B.xk[2]) +
      c_a[ACC_B.i + 12] * ACC_B.xk[3]) + d_a[ACC_B.i] * ACC_DW.last_mv_DSTATE) +
      (0.0 * ACC_B.Switch2 + e_a[ACC_B.i] *
       ACC_B.rtb_TmpSignalConversionAtSFun_c)) + (f_a[ACC_B.i + 4] *
      ACC_B.y_innov[1] + f_a[ACC_B.i] * ACC_B.y_innov[0]);
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
    ACC_DW.obj_c.matlabCodegenIsDeleted = true;
    ACC_DW.obj_c.isInitialized = 0;
    ACC_DW.obj_c.matlabCodegenIsDeleted = false;
    ACC_DW.obj_c.isSetupComplete = false;
    ACC_DW.obj_c.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_4[i] = tmp_1[i];
    }

    tmp_4[14] = '\x00';
    Sub_ACC_139.createSubscriber(tmp_4, 1);
    ACC_DW.obj_c.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/SourceBlock'
    // End of Start for SubSystem: '<Root>/target_output'

    // Start for Atomic SubSystem: '<Root>/drive_ctrl_input'
    // Start for MATLABSystem: '<S4>/SourceBlock'
    ACC_DW.obj_b.matlabCodegenIsDeleted = true;
    ACC_DW.obj_b.isInitialized = 0;
    ACC_DW.obj_b.matlabCodegenIsDeleted = false;
    ACC_DW.obj_b.isSetupComplete = false;
    ACC_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      tmp_3[i] = tmp_0[i];
    }

    tmp_3[17] = '\x00';
    Sub_ACC_140.createSubscriber(tmp_3, 1);
    ACC_DW.obj_b.isSetupComplete = true;

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
    Pub_ACC_136.createPublisher(tmp_2, 1);
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
    ACC_B.In1_p = ACC_P.Out1_Y0_i;

    // End of SystemInitialize for SubSystem: '<S4>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/drive_ctrl_input'

    // SystemInitialize for Chart: '<S6>/ACC Diagnostics'
    ACC_DW.is_Standard = ACC_IN_NO_ACTIVE_CHILD;
    ACC_DW.is_ACC_Normal = ACC_IN_NO_ACTIVE_CHILD;
    ACC_DW.is_active_c11_ACC = 0U;
    ACC_DW.is_c11_ACC = ACC_IN_NO_ACTIVE_CHILD;
    ACC_DW.chartAbsoluteTimeCounter = 0;
  }
}

// Model terminate function
void ACC_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/target_output'
  // Terminate for MATLABSystem: '<S5>/SourceBlock'
  matlabCodegenHandle_matlabCod_e(&ACC_DW.obj_c);

  // End of Terminate for SubSystem: '<Root>/target_output'

  // Terminate for Atomic SubSystem: '<Root>/drive_ctrl_input'
  // Terminate for MATLABSystem: '<S4>/SourceBlock'
  matlabCodegenHandle_matlabCod_e(&ACC_DW.obj_b);

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
