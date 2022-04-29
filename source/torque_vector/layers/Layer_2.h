/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Layer_2.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 29-Apr-2022 01:05:09
 */

#ifndef LAYER_2_H
#define LAYER_2_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void
Layer_2(const double prev_torque[4], const double *driver_input,
        const double power_limits_battery[2],
        const double acker_steer_angles[2], const double FY[4],
        const double omega_w[4], const float T2F[4], const double T_brake[4],
        double yaw, const double yaw_ref[2], const double Fx_max[4],
        const double motor_limit_power[4], const double motor_efficiency[4],
        double J_z, const double gr[4], double RE, double tau,
        const double motor_limit_torque[2], double t, const double l[2],
        double disk_diameter, const double s[2], double *power_limit_battery,
        double omega_m[4], double x[4], double y[4], float slip_limit[4],
        double torque_limit_motor[4], double *tau_limit_upper,
        double *tau_limit_lower, double rpm_limit[4], double *yaw_err,
        double *yaw_err_percent);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Layer_2.h
 *
 * [EOF]
 */
