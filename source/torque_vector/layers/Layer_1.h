/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Layer_1.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 29-Apr-2022 01:46:54
 */

#ifndef LAYER_1_H
#define LAYER_1_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void Layer_1(double *driver_input, const double power_limits_battery[2],
                    const double C[4], const double omega_w[4],
                    double center_steer_angle,
                    const double acker_steer_angles[2], const double FY[4],
                    const double Vg[2], const double *yaw, double K_u,
                    const double gr[4], const double gearbox_efficiency[4],
                    double tau, double c_factor, double J_z, const double l[2],
                    const double s[2], double RE, double g,
                    const double tire_mu[4], const double brakeforce_max[4],
                    double brakepad_mu, double disk_diameter, double mech_brake,
                    double yaw_factor, double m, float T2F[4],
                    double T_brake[4], double yaw_ref[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Layer_1.h
 *
 * [EOF]
 */
