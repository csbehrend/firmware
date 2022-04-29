/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Layer_0.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 28-Apr-2022 13:32:12
 */

#ifndef LAYER_0_H
#define LAYER_0_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void Layer_0(double omega_w[4], const double *center_steer_angle,
                    double Vx, double Vy, const double *yaw, double FZ[4],
                    double SL[4], const double vel[6],
                    double acker_steer_angles[2], double FY[4], double Vg[2],
                    double Fx_max[4]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Layer_0.h
 *
 * [EOF]
 */
