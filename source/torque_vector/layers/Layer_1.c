/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Layer_1.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 29-Apr-2022 01:46:54
 */

/* Include Files */
#include "Layer_1.h"
#include "pchip.h"
#include "ppval.h"
#include <math.h>

/* Function Definitions */
/*
 * Layer 1 Computation
 *  Developer(s):    David Farell
 *                   Demetrius Gulewicz
 *                   Elliot Stockwell
 *                   Tao Sun
 *
 *  Developed for: Purdue Electric Racing
 *  Date:          12/25/2021
 *
 *  Description: This function primarily computes the reference yaw rate,
 *               and the brake forces & distribution.
 *
 *
 *  Pass Through Signals
 *  driver_input - Proportion Of Total Available Power To Use [Unitless][1x1]
 *  power_limits_battery - Max & Min Total Battery Power Output [W][2x1]
 *  center_steer_angle - Steering Column Angle From Neutral [rad][1x1]
 *  acker_steer_angles - Angle Tires Are Turned From Neutral [rad][2x1]
 *  FY - Tires Forces y Direction, Tire Fixed Coordinates [N][1x4]
 *  omega_w - Angular Velocity Of Tires, Tire Fixed Coordinates [m/s][1x4]
 *  yaw - Yaw Rate z Direction, Vehicle Fixed Coordinates  [rad/s][1x1]
 *
 *
 *  Input Only
 *  Vg - Center Of Gravity Velocity, Vehicle Fixed Coordinates [m/s][1x2]
 *
 *
 *  Output Only
 *  T2F - Convert Torque At The Motor Shaft to Force At The Tire [1/m][1x4]
 *  T_brake - Mechanical Brake Torque On Each Tire [Nm][1x4]
 *  yaw_ref - The Desired & Max Allowed Yaw Rate [rad/s][1x2]
 *
 *
 *  Constants
 *  global g; % Acceleration Due To Gravity [m/s^2][1x1]
 *  global tire_mu; % Maximum Friction Coefficient [Unitless][1x1]
 *  global gr; % Gear ratios [Unitless][1x4]
 *  global gearbox_efficiency; % Gearbox Power Transmission Efficiency
 * [Unitless][1x4] global r; % Tire Radius [m][1x1] global l; % Wheelbase
 * [m][1x2] global K_u; % Understeer Gradient [rad/m/s^2][1x1] global c_factor;
 * % Max Yaw Equation Contributing Factor [Unitless][1x1] global brakeforce_max;
 * % Max Force Brakes Can Exert On Each Tire Axle [N][1x4] global brakepad_mu; %
 * Brakepad Friction Coefficient[Unitless][1x1] global mech_brake; % Value Of
 * driver_input Where Mechanical Braking Begins [Unitless][1x1] global
 * disk_diameter; % Diameter of brake disk [m][1x1] global yaw_factor; %
 * Coefficent to adjust desired yaw magnitude [1x1]
 *
 * Arguments    : double *driver_input
 *                const double power_limits_battery[2]
 *                const double C[4]
 *                const double omega_w[4]
 *                double center_steer_angle
 *                const double acker_steer_angles[2]
 *                const double FY[4]
 *                const double Vg[2]
 *                const double *yaw
 *                double K_u
 *                const double gr[4]
 *                const double gearbox_efficiency[4]
 *                double tau
 *                double c_factor
 *                double J_z
 *                const double l[2]
 *                const double s[2]
 *                double RE
 *                double g
 *                const double tire_mu[4]
 *                const double brakeforce_max[4]
 *                double brakepad_mu
 *                double disk_diameter
 *                double mech_brake
 *                double yaw_factor
 *                double m
 *                float T2F[4]
 *                double T_brake[4]
 *                double yaw_ref[2]
 * Return Type  : void
 */
void Layer_1(double *driver_input, const double power_limits_battery[2],
             const double C[4], const double omega_w[4],
             double center_steer_angle, const double acker_steer_angles[2],
             const double FY[4], const double Vg[2], const double *yaw,
             double K_u, const double gr[4], const double gearbox_efficiency[4],
             double tau, double c_factor, double J_z, const double l[2],
             const double s[2], double RE, double g, const double tire_mu[4],
             const double brakeforce_max[4], double brakepad_mu,
             double disk_diameter, double mech_brake, double yaw_factor,
             double m, float T2F[4], double T_brake[4], double yaw_ref[2])
{
  static const double b_y[10] = {1.25,  1.25,  1.25,  0.55,  0.4,
                                 0.325, 0.275, 0.275, 0.275, 0.275};
  static const double dv[10] = {1.25,  1.25,  1.25,  0.55,  0.4,
                                0.325, 0.275, 0.275, 0.275, 0.275};
  static const double dv1[10] = {1.34, 1.34,  1.34,  1.66, 1.14,
                                 1.24, 0.754, 0.849, 0.85, 0.85};
  static const double x[10] = {0.0,  3.0,  6.0,  9.0,  12.0,
                               15.0, 18.0, 21.0, 25.0, 31.0};
  static const double y[10] = {1.34, 1.34,  1.34,  1.66, 1.14,
                               1.24, 0.754, 0.849, 0.85, 0.85};
  static const signed char iv[10] = {0, 3, 6, 9, 12, 15, 18, 21, 25, 31};
  double pp_coefs[36];
  double b_slopes[10];
  double slopes[10];
  double b_del[9];
  double del[9];
  double absxk;
  double absxk_tmp;
  double scale;
  double t;
  double v_cog;
  double yaw_max;
  int hs;
  int hs3;
  int hs_tmp;
  int k;
  signed char b_h[9];
  signed char h[9];
  signed char i;
  (void)power_limits_battery;
  (void)omega_w;
  (void)acker_steer_angles;
  (void)FY;
  (void)yaw;
  (void)K_u;
  (void)gr;
  (void)gearbox_efficiency;
  (void)tau;
  (void)J_z;
  (void)s;
  (void)RE;
  (void)tire_mu;
  (void)yaw_factor;
  T2F[0] = 29.0F;
  T2F[1] = 29.0F;
  T2F[2] = 35.0F;
  T2F[3] = 35.0F;
  /*  Calculation */
  /*  Convert Torque At The Motor Shaft to Force At The Wheel */
  /*  During Runtime, T2F Is Constant */
  /*  T2F might need to be found by experimentation or something, because the */
  /*  formula used appears to be wrong */
  /*  T2F = single((gr .* gearbox_efficiency) ./ RE); */
  /*  Compute Ku, the understeer gradient */
  /*  Yaw Calculations */
  /*  If yaw_des or yaw_max ever exceed 2, then the exact value is no longer */
  /*  needed. As in, the ceiling of these two variables is 2. */
  scale = 3.3121686421112381E-170;
  absxk_tmp = fabs(Vg[0]);
  if (absxk_tmp > 3.3121686421112381E-170) {
    v_cog = 1.0;
    scale = absxk_tmp;
  } else {
    t = absxk_tmp / 3.3121686421112381E-170;
    v_cog = t * t;
  }
  absxk = fabs(Vg[1]);
  if (absxk > scale) {
    t = scale / absxk;
    v_cog = v_cog * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    v_cog += t * t;
  }
  v_cog = scale * sqrt(v_cog);
  for (k = 0; k < 9; k++) {
    i = (signed char)(iv[k + 1] - iv[k]);
    h[k] = i;
    del[k] = (dv[k + 1] - dv[k]) / (double)i;
  }
  for (k = 0; k < 8; k++) {
    hs_tmp = h[k + 1];
    i = h[k];
    hs = i + hs_tmp;
    hs3 = 3 * hs;
    slopes[k + 1] =
        interiorSlope(del[k], del[k + 1], (double)(i + hs) / (double)hs3,
                      (double)(hs_tmp + hs) / (double)hs3);
  }
  slopes[0] = exteriorSlope(del[0], del[1], h[0], h[1]);
  slopes[9] = exteriorSlope(del[8], del[7], h[8], h[7]);
  for (k = 0; k < 9; k++) {
    i = (signed char)(iv[k + 1] - iv[k]);
    b_h[k] = i;
    b_del[k] = (dv1[k + 1] - dv1[k]) / (double)i;
  }
  for (k = 0; k < 8; k++) {
    hs_tmp = b_h[k + 1];
    i = b_h[k];
    hs = i + hs_tmp;
    hs3 = 3 * hs;
    b_slopes[k + 1] =
        interiorSlope(b_del[k], b_del[k + 1], (double)(i + hs) / (double)hs3,
                      (double)(hs_tmp + hs) / (double)hs3);
  }
  b_slopes[0] = exteriorSlope(b_del[0], b_del[1], b_h[0], b_h[1]);
  b_slopes[9] = exteriorSlope(b_del[8], b_del[7], b_h[8], b_h[7]);
  for (hs = 0; hs < 9; hs++) {
    hs_tmp = b_h[hs];
    scale = b_del[hs];
    t = b_slopes[hs];
    absxk = (scale - t) / (double)hs_tmp;
    scale = (b_slopes[hs + 1] - scale) / (double)hs_tmp;
    pp_coefs[hs] = (scale - absxk) / (double)hs_tmp;
    pp_coefs[hs + 9] = 2.0 * absxk - scale;
    pp_coefs[hs + 18] = t;
    pp_coefs[hs + 27] = y[hs];
  }
  yaw_max = c_factor * ppval(x, pp_coefs, v_cog) * g / v_cog;
  /*  Reference yaw generation is important. Might need to work on getting */
  /*  something that is custom to the vehicle and its behaviour. */
  /*  yaw_des = (yaw_factor * v_cog * center_steer_angle) / ((l(1) + l(2)) + (Ku
   * * v_cog ^ 2)); */
  /*  yaw_max = (c_factor * tire_mu * g) / (v_cog); */
  /*  Control For Large Max Yaw, For Better Viewing */
  if (yaw_max > 3.0) {
    yaw_max = 3.0;
  }
  for (hs = 0; hs < 9; hs++) {
    hs_tmp = h[hs];
    scale = del[hs];
    t = slopes[hs];
    absxk = (scale - t) / (double)hs_tmp;
    scale = (slopes[hs + 1] - scale) / (double)hs_tmp;
    pp_coefs[hs] = (scale - absxk) / (double)hs_tmp;
    pp_coefs[hs + 9] = 2.0 * absxk - scale;
    pp_coefs[hs + 18] = t;
    pp_coefs[hs + 27] = b_y[hs];
  }
  scale = l[0] + l[1];
  yaw_ref[0] =
      ppval(x, pp_coefs, v_cog) * v_cog * center_steer_angle /
      (scale + m / scale * (l[1] / (C[0] + C[1]) - l[0] / (C[2] + C[3])) *
                   (v_cog * v_cog));
  yaw_ref[1] = yaw_max;
  /*  Apply correct sign convention to yaw_ref */
  if (center_steer_angle < 0.0) {
    yaw_ref[1] = -yaw_max;
  }
  /*  Compute brake force and distribution */
  /*  for small brakes (input >= mech_brake), only regen braking */
  /*  for large brakes (input <= mech_brake), regen & mechanical braking */
  /*  note: actual force may not be proportional to the input */
  T_brake[0] = 0.0;
  T_brake[1] = 0.0;
  T_brake[2] = 0.0;
  T_brake[3] = 0.0;
  if (*driver_input <= mech_brake) {
    scale = -(-(*driver_input - mech_brake) / (mech_brake + 1.0));
    T_brake[0] = scale * brakeforce_max[0] * brakepad_mu * disk_diameter;
    T_brake[1] = scale * brakeforce_max[1] * brakepad_mu * disk_diameter;
    T_brake[2] = scale * brakeforce_max[2] * brakepad_mu * disk_diameter;
    T_brake[3] = scale * brakeforce_max[3] * brakepad_mu * disk_diameter;
    *driver_input = -1.0;
  }
  if (Vg[0] < 0.0) {
    T_brake[0] = -T_brake[0];
    T_brake[1] = -T_brake[1];
    T_brake[2] = -T_brake[2];
    T_brake[3] = -T_brake[3];
  }
  if (absxk_tmp < 0.1) {
    T_brake[0] = 0.0;
    T_brake[1] = 0.0;
    T_brake[2] = 0.0;
    T_brake[3] = 0.0;
  }
  if ((*driver_input <= 0.0) && (*driver_input >= mech_brake)) {
    *driver_input = -*driver_input / mech_brake;
    /*  in scale of [-1,0] */
  }
}

/*
 * File trailer for Layer_1.c
 *
 * [EOF]
 */
