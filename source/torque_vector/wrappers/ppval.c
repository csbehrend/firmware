/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ppval.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 29-Apr-2022 01:05:09
 */

/* Include Files */
#include "ppval.h"

/* Function Definitions */
/*
 * Arguments    : const double pp_breaks[10]
 *                const double pp_coefs[36]
 *                double x
 * Return Type  : double
 */
double ppval(const double pp_breaks[10], const double pp_coefs[36], double x)
{
  double xloc;
  int high_i;
  int low_i;
  int low_ip1;
  int mid_i;
  low_i = 0;
  low_ip1 = 2;
  high_i = 10;
  while (high_i > low_ip1) {
    mid_i = ((low_i + high_i) + 1) >> 1;
    if (x >= pp_breaks[mid_i - 1]) {
      low_i = mid_i - 1;
      low_ip1 = mid_i + 1;
    } else {
      high_i = mid_i;
    }
  }
  xloc = x - pp_breaks[low_i];
  return xloc * (xloc * (xloc * pp_coefs[low_i] + pp_coefs[low_i + 9]) +
                 pp_coefs[low_i + 18]) +
         pp_coefs[low_i + 27];
}

/*
 * File trailer for ppval.c
 *
 * [EOF]
 */
