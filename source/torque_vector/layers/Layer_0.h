/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Layer_0.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 29-Apr-2022 01:46:54
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
                    double Vx, double Vy, const double *yaw, const double FZ[4],
                    const double SL[4], double RE, const double s[2],
                    const double l[2], const double vel[6], double mu_factor,
                    double k_limit, double A1, double A2, double B1, double B2,
                    double B3, double C2, double C1, double C3, double C4,
                    double a, double b, double c, double d, double C[4],
                    double acker_steer_angles[2], double FY[4], double Vg[2],
                    double Fx_max[4], double n[4], double R[4], double SA[4],
                    double FX[4]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Layer_0.h
 *
 * [EOF]
 */
