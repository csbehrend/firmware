/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: pchip.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 29-Apr-2022 01:05:09
 */

/* Include Files */
#include "pchip.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : double d1
 *                double d2
 *                double h1
 *                double h2
 * Return Type  : double
 */
double exteriorSlope(double d1, double d2, double h1, double h2)
{
  double s;
  double signd1;
  double signs;
  s = ((2.0 * h1 + h2) * d1 - h1 * d2) / (h1 + h2);
  signd1 = d1;
  if (d1 < 0.0) {
    signd1 = -1.0;
  } else if (d1 > 0.0) {
    signd1 = 1.0;
  }
  signs = s;
  if (s < 0.0) {
    signs = -1.0;
  } else if (s > 0.0) {
    signs = 1.0;
  }
  if (signs != signd1) {
    s = 0.0;
  } else {
    signs = d2;
    if (d2 < 0.0) {
      signs = -1.0;
    } else if (d2 > 0.0) {
      signs = 1.0;
    }
    if ((signd1 != signs) && (fabs(s) > fabs(3.0 * d1))) {
      s = 3.0 * d1;
    }
  }
  return s;
}

/*
 * Arguments    : double d1
 *                double d2
 *                double w1
 *                double w2
 * Return Type  : double
 */
double interiorSlope(double d1, double d2, double w1, double w2)
{
  double s;
  s = 0.0;
  if (d1 < 0.0) {
    if (d2 <= d1) {
      s = d1 / (w1 * (d1 / d2) + w2);
    } else if (d2 < 0.0) {
      s = d2 / (w1 + w2 * (d2 / d1));
    }
  } else if (d1 > 0.0) {
    if (d2 >= d1) {
      s = d1 / (w1 * (d1 / d2) + w2);
    } else if (d2 > 0.0) {
      s = d2 / (w1 + w2 * (d2 / d1));
    }
  }
  return s;
}

/*
 * File trailer for pchip.c
 *
 * [EOF]
 */
