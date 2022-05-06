/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: LP_calc.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 29-Apr-2022 01:46:54
 */

/* Include Files */
#include "LP_calc.h"
#include <math.h>

/* Function Definitions */
/*
 * LP_calc Computation
 *  Developer(s):    David Farell
 *                   Demetrius Gulewicz
 *                   Elliot Stockwell
 *                   Tao Sun
 *
 *  Developed for: Purdue Electric Racing
 *  Date:          12/25/2021
 *
 *  Description: This function primarily computes the desired torque
 *               distribution after constructing an optimization statement.
 *
 *
 *  Pass Through Signals
 *  omega_m - Motor Shaft Angular Velocity [rad/s][1x4]
 *
 *
 *  Input Only
 *  prev_torque - Torque Requested To Motor In Previous Time Step [Nm][4x1]
 *  driver_input - Proportion of total available power to use [Unitless][1x1]
 *  T2F - Convert Torque At The Motor Shaft to Force At The Tire [1/m][1x4]
 *  power_limit_battery - Max & Min Total Battery Power Output [W][1x2]
 *  FY - Tires Forces y Direction, Tire fixed coordinates [N][1x4]
 *  x - Equivalent Distances That FX Acts From CoG [m][1x4]
 *  y - Equivalent Distances That FY Acts From CoG [m][1x4]
 *  slip_limit - Max Torque Before Tire Slips [Nm][1x4]
 *  torque_limit_motor - Max Torque Motor Can Output [Nm][1x4]
 *  tau_limit_upper - Max Torque Motor Is Allowed To Change To [Nm][1x4]
 *  tau_limit_lower - Min Torque Motor Is Allowed To Change To [Nm][1x4]
 *  PID - Desired Yaw Acceleration [rad/s^2][1x1]
 *
 *
 *  Output Only
 *  Tx - Torque Request To Motors [Nm][4x1]
 *  lb - Min Torque Request [Nm][1x4]
 *  ub - Max Torque Request [Nm][1x4]
 *  Aeq - Moment About CoG, Optimization Variable [rad/s^2][1x1]
 *  beq - Moment About CoG, Yaw Acceleration & Other Foces [rad/s^2][1x1]
 *  A - Power Consumed, Optimization Variable [W][1x1]
 *  b - Power Consumed, Driver Input [W][1x1]
 *  M_max - Max & Min Yaw Acceleration Achievable With lb & ub [rad/s^2][1x2]
 *  Power - Total Estimated Power Consumed by the 4 Motors
 *  TR - The Estimated Yaw Acceleration from the Vehicle
 *
 *
 *  Constants
 *  global J_z; % Polar Moment Of Inertia [kg m^2][1x1]
 *  global motor_efficiency; % Motor efficiency [Unitless][1x4]
 *  global motor_limit_torque; % Minimum & Max Torque Request From Motor
 * [Nm][1x2] global min_speed_regen; % Min Motor Shaft Speed For Regen Braking
 * [rad/s][1x1]
 *
 * Arguments    : const double prev_torque[4]
 *                double driver_input
 *                double power_limit_battery
 *                const double FY[4]
 *                const double omega_m[4]
 *                const double x[4]
 *                const double y[4]
 *                float T2F[4]
 *                const float slip_limit[4]
 *                const double power_limit[4]
 *                const double tau_limit_upper[4]
 *                const double tau_limit_lower[4]
 *                const double rpm_limit[4]
 *                double PID
 *                double J_z
 *                const double motor_efficiency[4]
 *                const double motor_limit_torque[2]
 *                double min_speed_regen
 *                double Tx_actual[4]
 *                float lb[4]
 *                float ub[4]
 *                double M_max[2]
 *                double *power
 *                float *bigM_flag
 *                double *typed
 *                double *zero_crossing
 *                double *yaw_accel
 *                double ub_adjust[4]
 *                double lb_adjust[4]
 *                float lb_plus[4]
 *                float ub_neg[4]
 *                double Tx2[4]
 *                bool gh[4]
 * Return Type  : void
 */
void LP_calc(const double prev_torque[4], double driver_input,
             double power_limit_battery, const double FY[4],
             const double omega_m[4], const double x[4], const double y[4],
             float T2F[4], const float slip_limit[4],
             const double power_limit[4], const double tau_limit_upper[4],
             const double tau_limit_lower[4], const double rpm_limit[4],
             double PID, double J_z, const double motor_efficiency[4],
             const double motor_limit_torque[2], double min_speed_regen,
             double Tx_actual[4], float lb[4], float ub[4], double M_max[2],
             double *power, float *bigM_flag, double *typed,
             double *zero_crossing, double *yaw_accel, double ub_adjust[4],
             double lb_adjust[4], float lb_plus[4], float ub_neg[4],
             double Tx2[4], bool gh[4])
{
  double u;
  float b_ex;
  float ex;
  float ub_f_idx_2;
  float ub_f_idx_3;
  float varargin_1_idx_2;
  float varargin_1_idx_3;
  int lb_f_idx_0;
  int lb_f_idx_1;
  int lb_f_idx_2;
  int lb_f_idx_3;
  signed char crossing_locations_idx_0;
  signed char crossing_locations_idx_1;
  signed char crossing_locations_idx_2;
  signed char crossing_locations_idx_3;
  (void)power_limit_battery;
  (void)PID;
  (void)motor_efficiency;
  /*  Calculation */
  /*  Calculate Boundary Conditions, The Max and Min Allowable Torque */
  /*  slip_limit = single([50 50 50 50]); */
  /*  Enforce No Backwards Rotation Of Tires */
  varargin_1_idx_2 = (float)motor_limit_torque[1];
  ub_f_idx_2 = (float)motor_limit_torque[0];
  for (lb_f_idx_0 = 0; lb_f_idx_0 < 4; lb_f_idx_0++) {
    u = power_limit[lb_f_idx_0];
    varargin_1_idx_3 = (float)tau_limit_upper[lb_f_idx_0];
    ub_f_idx_3 = (float)rpm_limit[lb_f_idx_0];
    ex = slip_limit[lb_f_idx_0];
    b_ex = ex;
    if (ex > (float)u) {
      b_ex = (float)u;
    }
    if (b_ex > varargin_1_idx_2) {
      b_ex = varargin_1_idx_2;
    }
    if (b_ex > varargin_1_idx_3) {
      b_ex = varargin_1_idx_3;
    }
    if (b_ex > ub_f_idx_3) {
      b_ex = ub_f_idx_3;
    }
    ub[lb_f_idx_0] = b_ex;
    ub_f_idx_3 = (float)tau_limit_lower[lb_f_idx_0];
    b_ex = -ex;
    if (-ex < (float)-u) {
      b_ex = (float)-u;
    }
    if (b_ex < ub_f_idx_2) {
      b_ex = ub_f_idx_2;
    }
    if (b_ex < ub_f_idx_3) {
      b_ex = ub_f_idx_3;
    }
    lb[lb_f_idx_0] = b_ex;
    if ((omega_m[lb_f_idx_0] < min_speed_regen) && (b_ex < 0.0F)) {
      lb[lb_f_idx_0] = 0.001F;
    }
  }
  /*  Max & Min Forces on the Tires */
  /*  Find Max & Min For PID, The Yaw Acceleration Of The Car */
  varargin_1_idx_3 =
      (float)(((y[0] * FY[0] + y[1] * FY[1]) + y[2] * FY[2]) + y[3] * FY[3]);
  M_max[0] =
      (((((float)x[0] * (ub[0] * T2F[0]) + (float)x[2] * (ub[2] * T2F[2])) +
         (float)x[1] * (lb[1] * T2F[1])) +
        (float)x[3] * (lb[3] * T2F[3])) +
       varargin_1_idx_3) /
      (float)J_z;
  M_max[1] =
      (((((float)x[1] * (ub[1] * T2F[1]) + (float)x[3] * (ub[3] * T2F[3])) +
         (float)x[0] * (lb[0] * T2F[0])) +
        (float)x[2] * (lb[2] * T2F[2])) +
       varargin_1_idx_3) /
      (float)J_z;
  /*  Limit PID To Be No Larger Than Possible Due To Torque Constraints */
  /*  Calculate Yaw Constraint */
  *zero_crossing = 0.0;
  /*  Optimization */
  lb_plus[0] = lb[0];
  ub_neg[0] = ub[0];
  crossing_locations_idx_0 = 0;
  ub_f_idx_3 = fabsf(ub[0] - lb[0]);
  gh[0] = (ub_f_idx_3 < 0.5F);
  if ((lb[0] < 0.0F) && (ub[0] > 0.0F)) {
    *zero_crossing = 1.0;
    crossing_locations_idx_0 = 1;
    u = prev_torque[0];
    if (prev_torque[0] < 0.0) {
      u = -1.0;
    } else if (prev_torque[0] > 0.0) {
      u = 1.0;
    }
    if (u > 0.0) {
      lb_plus[0] = 0.001F;
    } else if (u < 0.0) {
      ub_neg[0] = -0.001F;
    }
  } else if (ub_f_idx_3 < 0.5F) {
    ub_neg[0] = -0.001F;
    lb_plus[0] = -10.0F;
    lb[0] = -0.002F;
    ub[0] = 0.002F;
  }
  lb_f_idx_0 = 0;
  ub_adjust[0] = 0.0;
  lb_adjust[0] = 0.0;
  lb_plus[1] = lb[1];
  ub_neg[1] = ub[1];
  crossing_locations_idx_1 = 0;
  ub_f_idx_3 = fabsf(ub[1] - lb[1]);
  gh[1] = (ub_f_idx_3 < 0.5F);
  if ((lb[1] < 0.0F) && (ub[1] > 0.0F)) {
    *zero_crossing = 1.0;
    crossing_locations_idx_1 = 2;
    u = prev_torque[1];
    if (prev_torque[1] < 0.0) {
      u = -1.0;
    } else if (prev_torque[1] > 0.0) {
      u = 1.0;
    }
    if (u > 0.0) {
      lb_plus[1] = 0.001F;
    } else if (u < 0.0) {
      ub_neg[1] = -0.001F;
    }
  } else if (ub_f_idx_3 < 0.5F) {
    ub_neg[1] = -0.001F;
    lb_plus[1] = -10.0F;
    lb[1] = -0.002F;
    ub[1] = 0.002F;
  }
  lb_f_idx_1 = 0;
  ub_adjust[1] = 0.0;
  lb_adjust[1] = 0.0;
  lb_plus[2] = lb[2];
  ub_neg[2] = ub[2];
  crossing_locations_idx_2 = 0;
  ub_f_idx_3 = fabsf(ub[2] - lb[2]);
  gh[2] = (ub_f_idx_3 < 0.5F);
  if ((lb[2] < 0.0F) && (ub[2] > 0.0F)) {
    *zero_crossing = 1.0;
    crossing_locations_idx_2 = 3;
    u = prev_torque[2];
    if (prev_torque[2] < 0.0) {
      u = -1.0;
    } else if (prev_torque[2] > 0.0) {
      u = 1.0;
    }
    if (u > 0.0) {
      lb_plus[2] = 0.001F;
    } else if (u < 0.0) {
      ub_neg[2] = -0.001F;
    }
  } else if (ub_f_idx_3 < 0.5F) {
    ub_neg[2] = -0.001F;
    lb_plus[2] = -10.0F;
    lb[2] = -0.002F;
    ub[2] = 0.002F;
  }
  lb_f_idx_2 = 0;
  ub_adjust[2] = 0.0;
  lb_adjust[2] = 0.0;
  lb_plus[3] = lb[3];
  ub_neg[3] = ub[3];
  crossing_locations_idx_3 = 0;
  ub_f_idx_3 = fabsf(ub[3] - lb[3]);
  gh[3] = (ub_f_idx_3 < 0.5F);
  if ((lb[3] < 0.0F) && (ub[3] > 0.0F)) {
    *zero_crossing = 1.0;
    crossing_locations_idx_3 = 4;
    u = prev_torque[3];
    if (prev_torque[3] < 0.0) {
      u = -1.0;
    } else if (prev_torque[3] > 0.0) {
      u = 1.0;
    }
    if (u > 0.0) {
      lb_plus[3] = 0.001F;
    } else if (u < 0.0) {
      ub_neg[3] = -0.001F;
    }
  } else if (ub_f_idx_3 < 0.5F) {
    ub_neg[3] = -0.001F;
    lb_plus[3] = -10.0F;
    lb[3] = -0.002F;
    ub[3] = 0.002F;
  }
  lb_f_idx_3 = 0;
  ub_adjust[3] = 0.0;
  lb_adjust[3] = 0.0;
  if (driver_input < 0.0) {
    T2F[0] = -T2F[0];
    T2F[1] = -T2F[1];
    T2F[2] = -T2F[2];
    T2F[3] = -T2F[3];
  }
  Tx2[0] = 0.0;
  Tx2[1] = 0.0;
  Tx2[2] = 0.0;
  Tx2[3] = 0.0;
  if ((omega_m[0] < min_speed_regen) && (driver_input < 0.0)) {
    *typed = 7.0;
    *bigM_flag = 3.0F;
  } else if ((*zero_crossing == 1.0) && (omega_m[0] > 10.0)) {
    /*  Zero Crossing Primary Optimization */
    /* [Tx21, Tx22, Tx23, Tx24, bigM_flag2] = call_bigM_C(T2F, b, A, beq, Aeq,
     * lb_plus, ub_neg, signs, yaw_error_limit);            */
    /* [Tx31, Tx32, Tx33, Tx34, bigM_flag3] = call_bigM_C(T2F, b, A, beq, Aeq,
     * lb_adjust, ub_adjust, signs_adjust, yaw_error_limit); */
    /*  Zero Crossing Secondary Optimization */
    /* [Tx5, bigM_flag5] = call_bigM_alt(b, A, beq, Aeq, lb, ub_neg); */
    /* [Tx6, bigM_flag6] = call_bigM_alt(b, A, beq, Aeq, lb_adjust, ub_adjust);
     */
    /*  Optimization Selection */
    Tx2[0] = 1.0;
    if (crossing_locations_idx_0 > 0) {
      lb_adjust[0] = lb_plus[0];
      ub_adjust[0] = ub_neg[0];
    } else {
      lb_adjust[0] = lb[0];
      ub_adjust[0] = ub[0];
    }
    lb_f_idx_0 = 69;
    Tx2[1] = 2.0;
    if (crossing_locations_idx_1 > 0) {
      lb_adjust[1] = lb_plus[1];
      ub_adjust[1] = ub_neg[1];
    } else {
      lb_adjust[1] = lb[1];
      ub_adjust[1] = ub[1];
    }
    lb_f_idx_1 = 69;
    Tx2[2] = 3.0;
    if (crossing_locations_idx_2 > 0) {
      lb_adjust[2] = lb_plus[2];
      ub_adjust[2] = ub_neg[2];
    } else {
      lb_adjust[2] = lb[2];
      ub_adjust[2] = ub[2];
    }
    lb_f_idx_2 = 69;
    Tx2[3] = 4.0;
    if (crossing_locations_idx_3 > 0) {
      lb_adjust[3] = lb_plus[3];
      ub_adjust[3] = ub_neg[3];
    } else {
      lb_adjust[3] = lb[3];
      ub_adjust[3] = ub[3];
    }
    lb_f_idx_3 = 69;
    *typed = 6.0;
    *bigM_flag = 2.0F;
  } else {
    /*  No Zero Crossing */
    /*  UPDATE THIS -> */
    /* [Tx11, Tx12, Tx13, Tx14, bigM_flag1] = call_bigM_C(T2F, b, A, beq, Aeq,
     * lb, ub, signs, 0.05); */
    /*  [Tx4, bigM_flag4] = call_bigM_alt(b, A, beq, Aeq, lb, ub); */
    /*  Optimization Selection */
    lb_f_idx_0 = 1;
    lb_f_idx_1 = 2;
    lb_f_idx_2 = 3;
    lb_f_idx_3 = 4;
    *typed = 1.0;
    *bigM_flag = 1.0F;
  }
  *yaw_accel = (((((float)x[0] * T2F[0] * (float)lb_f_idx_0 +
                   (float)x[1] * T2F[1] * (float)lb_f_idx_1) +
                  (float)x[2] * T2F[2] * (float)lb_f_idx_2) +
                 (float)x[3] * T2F[3] * (float)lb_f_idx_3) +
                varargin_1_idx_3) /
               (float)J_z;
  *power =
      (((double)lb_f_idx_0 * omega_m[0] + (double)lb_f_idx_1 * omega_m[1]) +
       (double)lb_f_idx_2 * omega_m[2]) +
      (double)lb_f_idx_3 * omega_m[3];
  /*  Simplified Control Scheme */
  Tx_actual[0] = driver_input * 25.0;
  Tx_actual[1] = driver_input * 25.0;
  Tx_actual[2] = driver_input * 25.0;
  Tx_actual[3] = driver_input * 25.0;
}

/*
 * File trailer for LP_calc.c
 *
 * [EOF]
 */
