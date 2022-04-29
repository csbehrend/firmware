/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: pchip.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 29-Apr-2022 01:05:09
 */

#ifndef PCHIP_H
#define PCHIP_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double exteriorSlope(double d1, double d2, double h1, double h2);

double interiorSlope(double d1, double d2, double w1, double w2);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for pchip.h
 *
 * [EOF]
 */
