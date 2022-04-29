/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: LP_calc.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 29-Apr-2022 01:05:09
 */

#ifndef LP_CALC_H
#define LP_CALC_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void LP_calc(const double prev_torque[4], double driver_input,
                    double power_limit_battery, const double FY[4],
                    const double omega_m[4], const double x[4],
                    const double y[4], float T2F[4], const float slip_limit[4],
                    const double power_limit[4],
                    const double tau_limit_upper[4],
                    const double tau_limit_lower[4], const double rpm_limit[4],
                    double PID, double J_z, const double motor_efficiency[4],
                    const double motor_limit_torque[2], double min_speed_regen,
                    double Tx_actual[4], float lb[4], float ub[4],
                    double M_max[2], double *power, float *bigM_flag,
                    double *typed, double *zero_crossing, double *yaw_accel,
                    double ub_adjust[4], double lb_adjust[4], float lb_plus[4],
                    float ub_neg[4], double Tx2[4], bool gh[4]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for LP_calc.h
 *
 * [EOF]
 */
