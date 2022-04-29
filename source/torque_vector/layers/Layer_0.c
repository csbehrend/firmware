/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Layer_0.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 29-Apr-2022 01:05:09
 */

/* Include Files */
#include "Layer_0.h"
#include "pchip.h"
#include <math.h>

/* Function Definitions */
/*
 * Pass Through Signals
 *  center_steer_angle - Steering Column Angle From Neutral [rad][1x1]
 *  omega_w - Angular Velocity Of Tires, Tire Fixed Coordinates [m/s][1x4]
 *  yaw - Yaw Rate z Direction, Vehicle Fixed Coordinates  [rad/s][1x1]
 *
 *
 *  Input Only
 *  shock_pot_disp - [mm][1x4]
 *  Vx - Center Of Gravity x Velocity, Vehicle Fixed Coordinates [m/s][1x1]
 *  Vy - Center Of Gravity y Velocity, Vehicle Fixed Coordinates [m/s][1x1]
 *  vel - Axle Velocity at Each Tire [m/s][1x6]
 *  SL - Slip Ratio from Tire Block [Unitless][1x4]
 *
 *
 *  Output Only
 *  acker_steer_angles - Angle Tires Are Turned From Neutral [rad][2x1]
 *  Fx_max - Max Tire Forces x Direction, Tire Fixed Coordinates [N][1x4]
 *  FY - Tires Forces y Direction, Tire Fixed Coordinates [N][1x4]
 *  Vg - Center Of Gravity Velocity, Vehicle Fixed Coordinates [m/s][1x2]
 *
 *
 *  Constants
 *  global s; % Half track width [m][1x2]
 *  global l; % Wheelbase [m][1x2]
 *  global RE; % Effective Radius of tire [m][1x1]
 *  global mu_factor; % Coefficient of Friction Modifier
 *  global k_limit; % Max Combined Slip before problems happen
 *
 * Arguments    : double omega_w[4]
 *                const double *center_steer_angle
 *                double Vx
 *                double Vy
 *                const double *yaw
 *                const double FZ[4]
 *                const double SL[4]
 *                double RE
 *                const double s[2]
 *                const double l[2]
 *                const double vel[6]
 *                double mu_factor
 *                double k_limit
 *                double A1
 *                double A2
 *                double B1
 *                double B2
 *                double B3
 *                double C2
 *                double C1
 *                double C3
 *                double C4
 *                double a
 *                double b
 *                double c
 *                double d
 *                double C[4]
 *                double acker_steer_angles[2]
 *                double FY[4]
 *                double Vg[2]
 *                double Fx_max[4]
 *                double n[4]
 *                double R[4]
 *                double SA[4]
 *                double FX[4]
 * Return Type  : void
 */
void Layer_0(double omega_w[4], const double *center_steer_angle, double Vx,
             double Vy, const double *yaw, const double FZ[4],
             const double SL[4], double RE, const double s[2],
             const double l[2], const double vel[6], double mu_factor,
             double k_limit, double A1, double A2, double B1, double B2,
             double B3, double C2, double C1, double C3, double C4, double a,
             double b, double c, double d, double C[4],
             double acker_steer_angles[2], double FY[4], double Vg[2],
             double Fx_max[4], double n[4], double R[4], double SA[4],
             double FX[4])
{
  static const double dv[7] = {0.0,    204.13, 427.04, 668.1,
                               895.72, 1124.4, 1324.4};
  static const double dv1[7] = {0.0,      13757.41, 21278.97, 26666.02,
                                30253.47, 30313.18, 30313.18};
  static const double pp_breaks[7] = {0.0,    204.13, 427.04, 668.1,
                                      895.72, 1124.4, 1324.4};
  double pp_coefs[24];
  double slopes[7];
  double del[6];
  double h[6];
  double b_SL[4];
  double k[4];
  double mu_x[4];
  double mu_y[4];
  double s_bar[4];
  double SA_fl;
  double SA_fr;
  double a_bar_idx_0;
  double a_bar_idx_1;
  double a_bar_idx_2;
  double absxk;
  double b_d;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double d7;
  double d8;
  double dzdxdx;
  double hs;
  double kx_idx_0;
  double kx_idx_1;
  double kx_idx_2;
  double n0_idx_0;
  double n0_idx_1;
  double phi_fl;
  double phi_fr;
  double phi_rl;
  double phi_rr;
  double scale;
  double t;
  double y;
  int high_i;
  int ix;
  int low_i;
  int low_ip1;
  int mid_i;
  bool guard1 = false;
  (void)SL;
  /*  Compute FZ */
  /*  Incomplete */
  /*  Compute Steering Angles */
  /*  Incomplete */
  acker_steer_angles[0] = *center_steer_angle;
  acker_steer_angles[1] = *center_steer_angle;
  /*  Simulation Only */
  /*  This section is only for simulation. IGNORE THIS SECTION */
  /*  Motor Shaft Speed, m/s */
  /*  Motor Shaft Speed, Rad/s */
  /*  omega_w = ((SL .* abs(v_w)) + v_w) ./ RE; */
  omega_w[0] = (vel[0] * cos(acker_steer_angles[0]) +
                vel[1] * sin(acker_steer_angles[0])) /
               RE;
  omega_w[1] = (vel[2] * cos(acker_steer_angles[1]) +
                vel[3] * sin(acker_steer_angles[1])) /
               RE;
  omega_w[2] = vel[4] / RE;
  omega_w[3] = vel[5] / RE;
  /*  Compute Tire Slip */
  /*  Velocity at CoG */
  Vg[0] = Vx;
  Vg[1] = Vy;
  /*  Velocity at each Tire */
  /* V = [V_fl V_fr V_rl V_rr]; */
  /*  Individual slip angles, it blows up at low velocity */
  b_d = fabs(Vx);
  if (b_d > 0.1) {
    hs = s[0] * *yaw;
    dzdxdx = Vy + l[0] * *yaw;
    phi_fl = atan2(dzdxdx, Vx + hs);
    phi_fr = atan2(dzdxdx, Vx - hs);
    hs = s[1] * *yaw;
    dzdxdx = Vy - l[1] * *yaw;
    phi_rl = atan2(dzdxdx, Vx + hs);
    phi_rr = atan2(dzdxdx, Vx - hs);
  } else {
    phi_fl = 0.0;
    phi_fr = 0.0;
    phi_rl = 0.0;
    phi_rr = 0.0;
  }
  /* phi = [phi_fl phi_fr phi_rl phi_rr]; */
  /*  Compute Slip Angle */
  SA_fl = -phi_fl + acker_steer_angles[0];
  SA_fr = -phi_fr + acker_steer_angles[1];
  SA[0] = SA_fl;
  SA[1] = SA_fr;
  SA[2] = -phi_rl;
  SA[3] = -phi_rr;
  /*  Compute Slip Ratio */
  /*  This line is commented out only for simulation. On the vehicle, this */
  /*  line is to be included. */
  scale = 3.3121686421112381E-170;
  absxk = fabs(Vx + *yaw * s[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  phi_fr = fabs(Vy + *yaw * l[0]);
  if (phi_fr > scale) {
    t = scale / phi_fr;
    y = y * t * t + 1.0;
    scale = phi_fr;
  } else {
    t = phi_fr / scale;
    y += t * t;
  }
  y = scale * sqrt(y);
  scale = 3.3121686421112381E-170;
  absxk = fabs(Vx + *yaw * -s[0]);
  if (absxk > 3.3121686421112381E-170) {
    phi_fl = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    phi_fl = t * t;
  }
  if (phi_fr > scale) {
    t = scale / phi_fr;
    phi_fl = phi_fl * t * t + 1.0;
    scale = phi_fr;
  } else {
    t = phi_fr / scale;
    phi_fl += t * t;
  }
  phi_fl = scale * sqrt(phi_fl);
  scale = 3.3121686421112381E-170;
  absxk = fabs(Vx + *yaw * s[1]);
  if (absxk > 3.3121686421112381E-170) {
    hs = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    hs = t * t;
  }
  phi_fr = fabs(Vy + *yaw * -l[1]);
  if (phi_fr > scale) {
    t = scale / phi_fr;
    hs = hs * t * t + 1.0;
    scale = phi_fr;
  } else {
    t = phi_fr / scale;
    hs += t * t;
  }
  hs = scale * sqrt(hs);
  scale = 3.3121686421112381E-170;
  absxk = fabs(Vx + *yaw * -s[1]);
  if (absxk > 3.3121686421112381E-170) {
    dzdxdx = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    dzdxdx = t * t;
  }
  if (phi_fr > scale) {
    t = scale / phi_fr;
    dzdxdx = dzdxdx * t * t + 1.0;
    scale = phi_fr;
  } else {
    t = phi_fr / scale;
    dzdxdx += t * t;
  }
  dzdxdx = scale * sqrt(dzdxdx);
  b_SL[0] = RE * omega_w[0] / (y * cos(SA_fl)) - 1.0;
  b_SL[1] = RE * omega_w[1] / (phi_fl * cos(SA_fr)) - 1.0;
  b_SL[2] = RE * omega_w[2] / (hs * cos(-phi_rl)) - 1.0;
  b_SL[3] = RE * omega_w[3] / (dzdxdx * cos(-phi_rr)) - 1.0;
  /*  Non Dimensional Tire Model */
  /*  C model */
  s_bar[0] = fabs(FZ[0]);
  s_bar[1] = fabs(FZ[1]);
  s_bar[2] = fabs(FZ[2]);
  s_bar[3] = fabs(FZ[3]);
  for (low_i = 0; low_i < 6; low_i++) {
    d1 = dv[low_i + 1] - dv[low_i];
    h[low_i] = d1;
    del[low_i] = (dv1[low_i + 1] - dv1[low_i]) / d1;
  }
  for (low_i = 0; low_i < 5; low_i++) {
    dzdxdx = h[low_i + 1];
    d1 = h[low_i];
    hs = d1 + dzdxdx;
    phi_fr = 3.0 * hs;
    slopes[low_i + 1] = interiorSlope(
        del[low_i], del[low_i + 1], (d1 + hs) / phi_fr, (dzdxdx + hs) / phi_fr);
  }
  slopes[0] = exteriorSlope(del[0], del[1], h[0], h[1]);
  slopes[6] = exteriorSlope(del[5], del[4], h[5], h[4]);
  for (low_i = 0; low_i < 6; low_i++) {
    d1 = del[low_i];
    d2 = slopes[low_i];
    d3 = h[low_i];
    phi_fr = (d1 - d2) / d3;
    dzdxdx = (slopes[low_i + 1] - d1) / d3;
    pp_coefs[low_i] = (dzdxdx - phi_fr) / d3;
    pp_coefs[low_i + 6] = 2.0 * phi_fr - dzdxdx;
    pp_coefs[low_i + 12] = d2;
    pp_coefs[low_i + 18] = dv1[low_i];
  }
  for (ix = 0; ix < 4; ix++) {
    low_i = 0;
    low_ip1 = 2;
    high_i = 7;
    while (high_i > low_ip1) {
      mid_i = ((low_i + high_i) + 1) >> 1;
      if (s_bar[ix] >= pp_breaks[mid_i - 1]) {
        low_i = mid_i - 1;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }
    hs = s_bar[ix] - pp_breaks[low_i];
    C[ix] = hs * (hs * (hs * pp_coefs[low_i] + pp_coefs[low_i + 6]) +
                  pp_coefs[low_i + 12]) +
            pp_coefs[low_i + 18];
  }
  /*  Basic Coefficients */
  /*  Normalized Slip Ratio, Slip Angle */
  /*  C is a function of FZ, but it is not an analytical expression */
  /*  Combined Slip Model Parameters */
  d1 = A1 * FZ[0] + A2;
  kx_idx_0 = d1;
  d2 = FZ[0] * FZ[0];
  d3 = (B1 * d2 + B2 * FZ[0]) + B3;
  mu_x[0] = d3;
  d4 = ((C1 * pow(FZ[0], 3.0) + C2 * d2) + C3 * FZ[0]) + C4;
  mu_y[0] = d4;
  d5 = d3 * FZ[0];
  Fx_max[0] = d5;
  d2 = d1 * b_SL[0] / d5;
  d6 = tan(SA_fl);
  d7 = C[0] * d6 / (d4 * FZ[0]);
  a_bar_idx_0 = d7;
  n0_idx_0 = C[0] * d3 / (d1 * d4);
  k[0] = sqrt(d2 * d2 + d7 * d7);
  n[0] = 1.0;
  d1 = A1 * FZ[1] + A2;
  kx_idx_1 = d1;
  d2 = FZ[1] * FZ[1];
  d3 = (B1 * d2 + B2 * FZ[1]) + B3;
  mu_x[1] = d3;
  d4 = ((C1 * pow(FZ[1], 3.0) + C2 * d2) + C3 * FZ[1]) + C4;
  mu_y[1] = d4;
  d5 = d3 * FZ[1];
  Fx_max[1] = d5;
  d2 = d1 * b_SL[1] / d5;
  d8 = tan(SA_fr);
  d7 = C[1] * d8 / (d4 * FZ[1]);
  a_bar_idx_1 = d7;
  n0_idx_1 = C[1] * d3 / (d1 * d4);
  k[1] = sqrt(d2 * d2 + d7 * d7);
  n[1] = 1.0;
  d1 = A1 * FZ[2] + A2;
  kx_idx_2 = d1;
  d2 = FZ[2] * FZ[2];
  d3 = (B1 * d2 + B2 * FZ[2]) + B3;
  mu_x[2] = d3;
  d4 = ((C1 * pow(FZ[2], 3.0) + C2 * d2) + C3 * FZ[2]) + C4;
  mu_y[2] = d4;
  d5 = d3 * FZ[2];
  Fx_max[2] = d5;
  d2 = d1 * b_SL[2] / d5;
  phi_rl = tan(-phi_rl);
  d7 = C[2] * phi_rl / (d4 * FZ[2]);
  a_bar_idx_2 = d7;
  dzdxdx = C[2] * d3 / (d1 * d4);
  k[2] = sqrt(d2 * d2 + d7 * d7);
  n[2] = 1.0;
  d1 = A1 * FZ[3] + A2;
  d2 = FZ[3] * FZ[3];
  d3 = (B1 * d2 + B2 * FZ[3]) + B3;
  mu_x[3] = d3;
  d4 = ((C1 * pow(FZ[3], 3.0) + C2 * d2) + C3 * FZ[3]) + C4;
  mu_y[3] = d4;
  d5 = d3 * FZ[3];
  d2 = d1 * b_SL[3] / d5;
  SA_fl = tan(-phi_rr);
  d7 = C[3] * SA_fl / (d4 * FZ[3]);
  phi_fr = C[3] * d3 / (d1 * d4);
  SA_fr = d7 * d7;
  k[3] = sqrt(d2 * d2 + SA_fr);
  n[3] = 1.0;
  /*  Compute R, the bridge between tire slip and tire forces */
  /*  Find FX & FY, denominator blows up at no slip */
  for (low_i = 0; low_i < 4; low_i++) {
    if (fabs(k[low_i]) <= 6.2831853071795862) {
      n[0] = 0.5 * (n0_idx_0 + 1.0) - 0.5 * cos(k[0] / 2.0) * (1.0 - n0_idx_0);
      n[1] = 0.5 * (n0_idx_1 + 1.0) - 0.5 * cos(k[1] / 2.0) * (1.0 - n0_idx_1);
      n[2] = 0.5 * (dzdxdx + 1.0) - 0.5 * cos(k[2] / 2.0) * (1.0 - dzdxdx);
      n[3] = 0.5 * (phi_fr + 1.0) - 0.5 * cos(k[3] / 2.0) * (1.0 - phi_fr);
    }
    d2 = k[low_i];
    R[low_i] = a * exp(b * d2) + c * exp(d * d2);
    FY[low_i] = 0.0;
    FX[low_i] = 0.0;
  }
  low_i = 0;
  /*  Max Longitudinal Force */
  /*  Constant during runtime */
  for (low_ip1 = 0; low_ip1 < 4; low_ip1++) {
    low_i = low_ip1;
    phi_fl = tan(SA[low_ip1]);
    d2 = n[low_ip1];
    d4 = b_SL[low_ip1];
    hs = sqrt(d4 * d4 + d2 * d2 * (phi_fl * phi_fl));
    guard1 = false;
    if (fabs(hs) < 0.001) {
      scale = 3.3121686421112381E-170;
      if (b_d > 3.3121686421112381E-170) {
        y = 1.0;
        scale = b_d;
      } else {
        t = b_d / 3.3121686421112381E-170;
        y = t * t;
      }
      absxk = fabs(Vy);
      if (absxk > scale) {
        t = scale / absxk;
        y = y * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
      y = scale * sqrt(y);
      if (y >= 3.0) {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      FX[low_ip1] = mu_x[low_ip1] * FZ[low_ip1] * R[low_ip1] * d4 / hs;
      FY[low_ip1] = d2 * R[low_ip1] * phi_fl * mu_y[low_ip1] * FZ[low_ip1] / hs;
    }
    s_bar[low_ip1] = 1.0;
  }
  if (fabs(k[low_i]) <= 6.2831853071795862) {
    b_d = 0.5 * cos(k_limit / 2.0);
    s_bar[0] = 0.5 * (n0_idx_0 + 1.0) - b_d * (1.0 - n0_idx_0);
    s_bar[1] = 0.5 * (n0_idx_1 + 1.0) - b_d * (1.0 - n0_idx_1);
    s_bar[2] = 0.5 * (dzdxdx + 1.0) - b_d * (1.0 - dzdxdx);
    s_bar[3] = 0.5 * (phi_fr + 1.0) - b_d * (1.0 - phi_fr);
  }
  /*  Compare FY Calculations */
  /*  Incomplete, or will not be implemented */
  b_d = 0.0;
  if (k_limit > fabs(a_bar_idx_0)) {
    b_d = sqrt(k_limit * k_limit - a_bar_idx_0 * a_bar_idx_0);
  }
  b_d = b_d * mu_x[0] * FZ[0] / kx_idx_0;
  phi_fr = a * exp(b * k_limit) + c * exp(d * k_limit);
  Fx_max[0] = Fx_max[0] * phi_fr * b_d /
              sqrt(b_d * b_d + s_bar[0] * s_bar[0] * (d6 * d6));
  FY[0] *= mu_factor;
  b_d = 0.0;
  if (k_limit > fabs(a_bar_idx_1)) {
    b_d = sqrt(k_limit * k_limit - a_bar_idx_1 * a_bar_idx_1);
  }
  b_d = b_d * mu_x[1] * FZ[1] / kx_idx_1;
  Fx_max[1] = Fx_max[1] * phi_fr * b_d /
              sqrt(b_d * b_d + s_bar[1] * s_bar[1] * (d8 * d8));
  FY[1] *= mu_factor;
  b_d = 0.0;
  if (k_limit > fabs(a_bar_idx_2)) {
    b_d = sqrt(k_limit * k_limit - a_bar_idx_2 * a_bar_idx_2);
  }
  b_d = b_d * mu_x[2] * FZ[2] / kx_idx_2;
  Fx_max[2] = Fx_max[2] * phi_fr * b_d /
              sqrt(b_d * b_d + s_bar[2] * s_bar[2] * (phi_rl * phi_rl));
  FY[2] *= mu_factor;
  b_d = 0.0;
  if (k_limit > fabs(d7)) {
    b_d = sqrt(k_limit * k_limit - SA_fr);
  }
  b_d = b_d * d3 * FZ[3] / d1;
  Fx_max[3] = d5 * phi_fr * b_d /
              sqrt(b_d * b_d + s_bar[3] * s_bar[3] * (SA_fl * SA_fl));
  FY[3] *= mu_factor;
}

/*
 * File trailer for Layer_0.c
 *
 * [EOF]
 */
