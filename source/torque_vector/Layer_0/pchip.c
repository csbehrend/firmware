/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: pchip.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 28-Apr-2022 13:32:12
 */

/* Include Files */
#include "pchip.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static double exteriorSlope(double d1, double d2, double h1, double h2);

/* Function Definitions */
/*
 * Arguments    : double d1
 *                double d2
 *                double h1
 *                double h2
 * Return Type  : double
 */
static double exteriorSlope(double d1, double d2, double h1, double h2)
{
  double b_s;
  double signd1;
  double signs;
  b_s = ((2.0 * h1 + h2) * d1 - h1 * d2) / (h1 + h2);
  signd1 = d1;
  if (d1 < 0.0) {
    signd1 = -1.0;
  } else if (d1 > 0.0) {
    signd1 = 1.0;
  } else if (d1 == 0.0) {
    signd1 = 0.0;
  }
  signs = b_s;
  if (b_s < 0.0) {
    signs = -1.0;
  } else if (b_s > 0.0) {
    signs = 1.0;
  } else if (b_s == 0.0) {
    signs = 0.0;
  }
  if (signs != signd1) {
    b_s = 0.0;
  } else {
    signs = d2;
    if (d2 < 0.0) {
      signs = -1.0;
    } else if (d2 > 0.0) {
      signs = 1.0;
    } else if (d2 == 0.0) {
      signs = 0.0;
    }
    if ((signd1 != signs) && (fabs(b_s) > fabs(3.0 * d1))) {
      b_s = 3.0 * d1;
    }
  }
  return b_s;
}

/*
 * Arguments    : const double xx[4]
 *                double v[4]
 * Return Type  : void
 */
void pchip(const double xx[4], double v[4])
{
  static const double dv[7] = {0.0,    204.13, 427.04, 668.1,
                               895.72, 1124.4, 1324.4};
  static const double dv1[7] = {0.0,      13757.41, 21278.97, 26666.02,
                                30253.47, 30313.18, 30313.18};
  static const double pp_breaks[7] = {0.0,    204.13, 427.04, 668.1,
                                      895.72, 1124.4, 1324.4};
  double pp_coefs[24];
  double slopes[7];
  double del[6];
  double h[6];
  double d;
  double d1;
  double dzdxdx;
  double hs;
  double hs3;
  double w1;
  int high_i;
  int k;
  int low_i;
  int low_ip1;
  int mid_i;
  for (k = 0; k < 6; k++) {
    d = dv[k + 1] - dv[k];
    h[k] = d;
    del[k] = (dv1[k + 1] - dv1[k]) / d;
  }
  for (k = 0; k < 5; k++) {
    dzdxdx = h[k + 1];
    d = h[k];
    hs = d + dzdxdx;
    hs3 = 3.0 * hs;
    w1 = (d + hs) / hs3;
    dzdxdx = (dzdxdx + hs) / hs3;
    slopes[k + 1] = 0.0;
    d = del[k];
    if (d < 0.0) {
      d1 = del[k + 1];
      if (d1 <= d) {
        slopes[k + 1] = d / (w1 * (d / d1) + dzdxdx);
      } else if (d1 < 0.0) {
        slopes[k + 1] = d1 / (w1 + dzdxdx * (d1 / d));
      }
    } else if (d > 0.0) {
      d1 = del[k + 1];
      if (d1 >= d) {
        slopes[k + 1] = d / (w1 * (d / del[k + 1]) + dzdxdx);
      } else if (d1 > 0.0) {
        slopes[k + 1] = del[k + 1] / (w1 + dzdxdx * (del[k + 1] / d));
      }
    }
  }
  slopes[0] = exteriorSlope(del[0], del[1], h[0], h[1]);
  slopes[6] = exteriorSlope(del[5], del[4], h[5], h[4]);
  for (k = 0; k < 6; k++) {
    d = del[k];
    d1 = slopes[k];
    hs = h[k];
    hs3 = (d - d1) / hs;
    dzdxdx = (slopes[k + 1] - d) / hs;
    pp_coefs[k] = (dzdxdx - hs3) / hs;
    pp_coefs[k + 6] = 2.0 * hs3 - dzdxdx;
    pp_coefs[k + 12] = d1;
    pp_coefs[k + 18] = dv1[k];
  }
  for (k = 0; k < 4; k++) {
    dzdxdx = xx[k];
    if (!rtIsNaN(dzdxdx)) {
      low_i = 0;
      low_ip1 = 2;
      high_i = 7;
      while (high_i > low_ip1) {
        mid_i = ((low_i + high_i) + 1) >> 1;
        if (xx[k] >= pp_breaks[mid_i - 1]) {
          low_i = mid_i - 1;
          low_ip1 = mid_i + 1;
        } else {
          high_i = mid_i;
        }
      }
      dzdxdx = xx[k] - pp_breaks[low_i];
      dzdxdx =
          dzdxdx * (dzdxdx * (dzdxdx * pp_coefs[low_i] + pp_coefs[low_i + 6]) +
                    pp_coefs[low_i + 12]) +
          pp_coefs[low_i + 18];
    }
    v[k] = dzdxdx;
  }
}

/*
 * File trailer for pchip.c
 *
 * [EOF]
 */
