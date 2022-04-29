/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Layer_2.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 29-Apr-2022 01:05:09
 */

/* Include Files */
#include "Layer_2.h"
#include <math.h>

/* Function Definitions */
/*
 * Layer 2 Computation
 *  Developer(s):    David Farell
 *                   Demetrius Gulewicz
 *                   Elliot Stockwell
 *                   Tao Sun
 *
 *  Developed for: Purdue Electric Racing
 *  Date:          12/25/2021
 *
 *  Description: This function primarily computes the maximum and minimum
 *               torque that can be requested, based on slip, power, and
 *               transient constraints. Also computes yaw error.
 *
 *
 *  Pass Through Signals
 *  prev_torque - Torque requested to motor in previous time step [Nm][4x1]
 *  driver_input - Proportion Of Total Available Power To Use [Unitless][1x1]
 *  T2F - Convert Torque At The Motor Shaft to Force At The Tire [1/m][1x4]
 *  FY - Tires Forces y Direction, Tire Fixed Coordinates [N][4x1]
 *
 *
 *  Input Only
 *  power_limits_battery - Max & Min Total Battery Power Output [W][2x1]
 *  center_steer_angle - Steering Column Angle From Neutral [rad][1x1]
 *  acker_steer_angles - Angle Tires Are Turned From Neutral [rad][2x1]
 *  Fx_max - Max Tire Forces x Direction, Tire Fixed Coordinates [N][1x4]
 *  omega_w - Angular Velocity Of Tires, Tire Fixed Coordinates [m/s][1x4]
 *  yaw - Yaw Rate z Direction, Vehicle Fixed Coordinates  [rad/s][1x1]
 *  T_brake - Mechanical Brake Torque On Each Tire [Nm][1x4]
 *  yaw_ref - The Desired & Max Allowed Yaw Rate [rad/s][1x2]
 *
 *
 *  Output Only
 *  power_limit_battery - Max OR Min Total Battery Power Output [W][1x1]
 *  omega_m - Motor Shaft Angular Velocity [rad/s][1x4]
 *  x - Equivalent Distances That Fx Acts From CoG [m][1x4]
 *  y - Equivalent Distances That Fy Acts From CoG [m][1x4]
 *  slip_limit - Max Torque Before Tire Slips [Nm][1x4]
 *  torque_limit_motor - Max Torque Motor Can Output [Nm][1x4]
 *  tau_limit_upper - Max Torque Motor Is Allowed To Change To [Nm][1x4]
 *  tau_limit_lower - Min Torque Motor Is Allowed To Change To [Nm][1x4]
 *  yaw_err - Error Between Target & Actual Yaw Rate [rad/s][1x1]
 *  RPM_limit - Max Torque Possible at Current RPM [Nm][1x4]
 *
 *
 *  Constants
 *  global s; % Half track width [m][1x2]
 *  global l; % Wheelbase [m][1x2]
 *  global motor_limit_power; % Absolute Power Limit For Each Motor [W][1x4]
 *  global motor_efficiency; % Motor Efficiency [Unitless][1x4]
 *  global tau; % Motor Response Time Constant [s][1x1]
 *  global t; % Controller Timing [s][1x1]
 *  global gr; % Gear Ratios [Unitless][1x4]
 *  global r; % Tire Radius [m][1x1]
 *  global motor_limit_torque; % Min & Max Torque Request From Motor [Nm][1x2]
 *
 * Arguments    : const double prev_torque[4]
 *                const double *driver_input
 *                const double power_limits_battery[2]
 *                const double acker_steer_angles[2]
 *                const double FY[4]
 *                const double omega_w[4]
 *                const float T2F[4]
 *                const double T_brake[4]
 *                double yaw
 *                const double yaw_ref[2]
 *                const double Fx_max[4]
 *                const double motor_limit_power[4]
 *                const double motor_efficiency[4]
 *                double J_z
 *                const double gr[4]
 *                double RE
 *                double tau
 *                const double motor_limit_torque[2]
 *                double t
 *                const double l[2]
 *                double disk_diameter
 *                const double s[2]
 *                double *power_limit_battery
 *                double omega_m[4]
 *                double x[4]
 *                double y[4]
 *                float slip_limit[4]
 *                double torque_limit_motor[4]
 *                double *tau_limit_upper
 *                double *tau_limit_lower
 *                double rpm_limit[4]
 *                double *yaw_err
 *                double *yaw_err_percent
 * Return Type  : void
 */
void Layer_2(const double prev_torque[4], const double *driver_input,
             const double power_limits_battery[2],
             const double acker_steer_angles[2], const double FY[4],
             const double omega_w[4], const float T2F[4],
             const double T_brake[4], double yaw, const double yaw_ref[2],
             const double Fx_max[4], const double motor_limit_power[4],
             const double motor_efficiency[4], double J_z, const double gr[4],
             double RE, double tau, const double motor_limit_torque[2],
             double t, const double l[2], double disk_diameter,
             const double s[2], double *power_limit_battery, double omega_m[4],
             double x[4], double y[4], float slip_limit[4],
             double torque_limit_motor[4], double *tau_limit_upper,
             double *tau_limit_lower, double rpm_limit[4], double *yaw_err,
             double *yaw_err_percent)
{
  double varargin_1[8];
  double ang_vel_RPM_idx_0;
  double ang_vel_RPM_idx_1;
  double ang_vel_RPM_idx_2;
  double c_lower;
  double c_upper;
  double d;
  double min_yaw_ref;
  double x_2_tmp;
  int k;
  (void)FY;
  (void)J_z;
  (void)disk_diameter;
  /*  Yaw Error */
  /*  Calculate the Yaw Error, Apply Sign Convention */
  if ((yaw_ref[0] == 0.0) && (yaw == 0.0)) {
    *yaw_err = 0.0;
    *yaw_err_percent = 0.0;
  } else {
    min_yaw_ref = fabs(yaw_ref[0]);
    c_upper = fabs(yaw_ref[1]);
    c_lower = yaw_ref[0];
    if (yaw_ref[0] < 0.0) {
      c_lower = -1.0;
    } else if (yaw_ref[0] > 0.0) {
      c_lower = 1.0;
    }
    if (min_yaw_ref > c_upper) {
      min_yaw_ref = c_upper;
    }
    min_yaw_ref *= c_lower;
    *yaw_err = min_yaw_ref - yaw;
    *yaw_err_percent = *yaw_err / min_yaw_ref;
  }
  /*  Moment Arm For Tire Forces */
  /*  Calculate K_1 Vehicle Constants */
  min_yaw_ref = sin(acker_steer_angles[0]);
  c_upper = cos(acker_steer_angles[0]);
  c_lower = sin(acker_steer_angles[1]);
  x_2_tmp = cos(acker_steer_angles[1]);
  x[0] = s[0] * c_upper + l[0] * min_yaw_ref;
  x[1] = -s[0] * x_2_tmp + l[0] * c_lower;
  x[2] = s[1];
  x[3] = -s[1];
  /*  Calculate K_2 Vehicle Constants */
  y[0] = -s[0] * min_yaw_ref + l[0] * c_upper;
  y[1] = s[0] * c_lower + l[0] * x_2_tmp;
  y[2] = -l[1];
  y[3] = -l[1];
  /*  Slip Limit */
  /*  Calculate Torque Limit Due to Power and Slip */
  /*  Power Limit */
  /*  Motor Shaft Speed, RAD/S */
  /*  % If torque_limit_motor ever exceed 30, then the exact value is no longer
   */
  /*  needed. As in, the ceiling is 30 */
  /*  Erase power limit solution above torque limit, for better data */
  /*  visualization */
  /*  RPM Limit */
  /*  This section in practice will probably just be a lookup table, when PER */
  /*  gets the chance to get test results for the motors */
  /*  Motor Shaft Speed, rpm */
  /*  quadratic constants that connect current and max RPM of motors */
  /*  quadratic constants that connnect current and max torque of motors */
  /*  maximum possible current at the current RPM */
  /*  calculate the maximum current possible at the current RPM */
  /*  calculate the maximum torque at the max current */
  /*  Motor Response Limit */
  /*  Calculate Constants for getting limit on motor output torque due to */
  /*  motor response time */
  /*  Note: e_term, c_lower, c_upper are based on tunable parameters, during */
  /*  runtime, they are constant */
  min_yaw_ref = exp(-t / tau);
  c_lower = (1.0 - min_yaw_ref) * motor_limit_torque[0];
  c_upper = (1.0 - min_yaw_ref) * motor_limit_torque[1];
  /*  Max and Min Torque Limits due to motor response time */
  slip_limit[0] = (float)(Fx_max[0] - T_brake[0] / RE) / T2F[0];
  d = omega_w[0] * gr[0];
  omega_m[0] = d;
  x_2_tmp = motor_limit_power[0] * motor_efficiency[0] / d;
  torque_limit_motor[0] = x_2_tmp;
  if (x_2_tmp > 25.0) {
    torque_limit_motor[0] = 25.0;
  } else if ((x_2_tmp < -25.0) && (d > 0.0)) {
    torque_limit_motor[0] = -0.01;
  } else if ((x_2_tmp < -25.0) && (d < 0.0)) {
    torque_limit_motor[0] = 25.0;
  }
  d *= 9.5492965964254;
  if (d < 7000.0) {
    d = 70.0;
  } else {
    d = (66.353 - sqrt(4402.7206089999991 - 1.0668 * (10452.0 - d))) *
        1.8747656542932134;
  }
  d = ((-0.0987 * (d * d) + 45.799 * d) + -129.65) / 100.0;
  rpm_limit[0] = d;
  if (d < 0.0) {
    rpm_limit[0] = 0.0;
  }
  d = prev_torque[0] * min_yaw_ref;
  ang_vel_RPM_idx_0 = d;
  varargin_1[0] = c_upper + d;
  varargin_1[4] = d + 0.01;
  slip_limit[1] = (float)(Fx_max[1] - T_brake[1] / RE) / T2F[1];
  d = omega_w[1] * gr[1];
  omega_m[1] = d;
  x_2_tmp = motor_limit_power[1] * motor_efficiency[1] / d;
  torque_limit_motor[1] = x_2_tmp;
  if (x_2_tmp > 25.0) {
    torque_limit_motor[1] = 25.0;
  } else if ((x_2_tmp < -25.0) && (d > 0.0)) {
    torque_limit_motor[1] = -0.01;
  } else if ((x_2_tmp < -25.0) && (d < 0.0)) {
    torque_limit_motor[1] = 25.0;
  }
  d *= 9.5492965964254;
  if (d < 7000.0) {
    d = 70.0;
  } else {
    d = (66.353 - sqrt(4402.7206089999991 - 1.0668 * (10452.0 - d))) *
        1.8747656542932134;
  }
  d = ((-0.0987 * (d * d) + 45.799 * d) + -129.65) / 100.0;
  rpm_limit[1] = d;
  if (d < 0.0) {
    rpm_limit[1] = 0.0;
  }
  d = prev_torque[1] * min_yaw_ref;
  ang_vel_RPM_idx_1 = d;
  varargin_1[1] = c_upper + d;
  varargin_1[5] = d + 0.01;
  slip_limit[2] = (float)(Fx_max[2] - T_brake[2] / RE) / T2F[2];
  d = omega_w[2] * gr[2];
  omega_m[2] = d;
  x_2_tmp = motor_limit_power[2] * motor_efficiency[2] / d;
  torque_limit_motor[2] = x_2_tmp;
  if (x_2_tmp > 25.0) {
    torque_limit_motor[2] = 25.0;
  } else if ((x_2_tmp < -25.0) && (d > 0.0)) {
    torque_limit_motor[2] = -0.01;
  } else if ((x_2_tmp < -25.0) && (d < 0.0)) {
    torque_limit_motor[2] = 25.0;
  }
  d *= 9.5492965964254;
  if (d < 7000.0) {
    d = 70.0;
  } else {
    d = (66.353 - sqrt(4402.7206089999991 - 1.0668 * (10452.0 - d))) *
        1.8747656542932134;
  }
  d = ((-0.0987 * (d * d) + 45.799 * d) + -129.65) / 100.0;
  rpm_limit[2] = d;
  if (d < 0.0) {
    rpm_limit[2] = 0.0;
  }
  d = prev_torque[2] * min_yaw_ref;
  ang_vel_RPM_idx_2 = d;
  varargin_1[2] = c_upper + d;
  varargin_1[6] = d + 0.01;
  slip_limit[3] = (float)(Fx_max[3] - T_brake[3] / RE) / T2F[3];
  d = omega_w[3] * gr[3];
  omega_m[3] = d;
  x_2_tmp = motor_limit_power[3] * motor_efficiency[3] / d;
  torque_limit_motor[3] = x_2_tmp;
  if (x_2_tmp > 25.0) {
    torque_limit_motor[3] = 25.0;
  } else if ((x_2_tmp < -25.0) && (d > 0.0)) {
    torque_limit_motor[3] = -0.01;
  } else if ((x_2_tmp < -25.0) && (d < 0.0)) {
    torque_limit_motor[3] = 25.0;
  }
  d *= 9.5492965964254;
  if (d < 7000.0) {
    d = 70.0;
  } else {
    d = (66.353 - sqrt(4402.7206089999991 - 1.0668 * (10452.0 - d))) *
        1.8747656542932134;
  }
  d = ((-0.0987 * (d * d) + 45.799 * d) + -129.65) / 100.0;
  rpm_limit[3] = d;
  if (d < 0.0) {
    rpm_limit[3] = 0.0;
  }
  d = prev_torque[3] * min_yaw_ref;
  varargin_1[3] = c_upper + d;
  varargin_1[7] = d + 0.01;
  *tau_limit_upper = varargin_1[0];
  for (k = 0; k < 7; k++) {
    x_2_tmp = varargin_1[k + 1];
    if (*tau_limit_upper < x_2_tmp) {
      *tau_limit_upper = x_2_tmp;
    }
  }
  varargin_1[0] = c_lower + ang_vel_RPM_idx_0;
  varargin_1[4] = ang_vel_RPM_idx_0 - 0.01;
  varargin_1[1] = c_lower + ang_vel_RPM_idx_1;
  varargin_1[5] = ang_vel_RPM_idx_1 - 0.01;
  varargin_1[2] = c_lower + ang_vel_RPM_idx_2;
  varargin_1[6] = ang_vel_RPM_idx_2 - 0.01;
  varargin_1[3] = c_lower + d;
  varargin_1[7] = d - 0.01;
  *tau_limit_lower = varargin_1[0];
  for (k = 0; k < 7; k++) {
    d = varargin_1[k + 1];
    if (*tau_limit_lower > d) {
      *tau_limit_lower = d;
    }
  }
  /*  If driver is not pressing acceleration pedal, no positive torque is
   * allowed */
  if (*driver_input <= 0.0) {
    *power_limit_battery = power_limits_battery[1];
  } else {
    *power_limit_battery = power_limits_battery[0];
  }
}

/*
 * File trailer for Layer_2.c
 *
 * [EOF]
 */
