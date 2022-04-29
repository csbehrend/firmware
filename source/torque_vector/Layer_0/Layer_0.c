/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Layer_0.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 28-Apr-2022 13:32:12
 */

/* Include Files */
#include "Layer_0.h"
#include "tv_data.h"
#include "pchip.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <float.h>
#include <math.h>

/* Function Declarations */
static double rt_powd_snf(double u0, double u1);

static double rt_remd_snf(double u0, double u1);

static double rt_roundd_snf(double u);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double d;
  double d1;
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }
  return y;
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_remd_snf(double u0, double u1)
{
  double q;
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1) || rtIsInf(u0)) {
    y = rtNaN;
  } else if (rtIsInf(u1)) {
    y = u0;
  } else if ((u1 != 0.0) && (u1 != trunc(u1))) {
    q = fabs(u0 / u1);
    if (!(fabs(q - floor(q + 0.5)) > DBL_EPSILON * q)) {
      y = 0.0 * u0;
    } else {
      y = fmod(u0, u1);
    }
  } else {
    y = fmod(u0, u1);
  }
  return y;
}

/*
 * Arguments    : double u
 * Return Type  : double
 */
static double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }
  return y;
}

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
 *
 * Arguments    : double omega_w[4]
 *                const double *center_steer_angle
 *                double Vx
 *                double Vy
 *                const double *yaw
 *                double FZ[4]
 *                double SL[4]
 *                const double vel[6]
 *                double acker_steer_angles[2]
 *                double FY[4]
 *                double Vg[2]
 *                double Fx_max[4]
 * Return Type  : void
 */
void Layer_0(double omega_w[4], const double *center_steer_angle, double Vx,
             double Vy, const double *yaw, double FZ[4], double SL[4],
             const double vel[6], double acker_steer_angles[2], double FY[4],
             double Vg[2], double Fx_max[4])
{
  double C[4];
  double R[4];
  double k[4];
  double SA_fl;
  double SA_fr;
  double V_fl_idx_0;
  double V_rl_idx_0;
  double V_rr_idx_0;
  double a_bar_idx_0;
  double a_bar_idx_1;
  double a_bar_idx_2;
  double b_scale;
  double c_scale;
  double d;
  double d1;
  double d2;
  double d3;
  double d4;
  double d_scale;
  double kx_idx_1;
  double kx_idx_2;
  double mu_x_idx_0;
  double mu_x_idx_1;
  double mu_x_idx_2;
  double phi_fl;
  double phi_fr;
  double phi_rl;
  double phi_rr;
  double scale;
  double v_w_idx_0;
  double v_w_idx_1;
  double y;
  int i;
  signed char n;

  /*  Half track width [m][1x2] */
  /*  Wheelbase [m][1x2] */
  /*  Effective Radius of tire [m][1x1] */
  /*  Coefficient of Friction Modifier */
  /*  Max Combined Slip before problems happen */
  /*  Compute FY & FZ */
  /*  Incomplete */
  /*  Compute Steering Angles */
  /*  Incomplete */
  acker_steer_angles[0] = *center_steer_angle;
  acker_steer_angles[1] = *center_steer_angle;
  /*  Simulation Only */
  /*  This section is only for simulation. IGNORE THIS SECTION */
  /*  Motor Shaft Speed, m/s */
  v_w_idx_0 =
      vel[0] * cos(acker_steer_angles[0]) + vel[1] * sin(acker_steer_angles[0]);
  v_w_idx_1 =
      vel[2] * cos(acker_steer_angles[1]) + vel[3] * sin(acker_steer_angles[1]);
  /*  Motor Shaft Speed, Rad/s */
  phi_fr = 1.0 / FZ[0];
  if (rtIsInf(phi_fr) || rtIsNaN(phi_fr)) {
    FZ[0] = 1.0;
  }
  omega_w[0] = fabs(v_w_idx_0);
  omega_w[0] = (SL[0] * omega_w[0] + v_w_idx_0) / RE;
  phi_fr = 1.0 / FZ[1];
  if (rtIsInf(phi_fr) || rtIsNaN(phi_fr)) {
    FZ[1] = 1.0;
  }
  omega_w[1] = fabs(v_w_idx_1);
  omega_w[1] = (SL[1] * omega_w[1] + v_w_idx_1) / RE;
  phi_fr = 1.0 / FZ[2];
  if (rtIsInf(phi_fr) || rtIsNaN(phi_fr)) {
    FZ[2] = 1.0;
  }
  omega_w[2] = fabs(vel[4]);
  omega_w[2] = (SL[2] * omega_w[2] + vel[4]) / RE;
  phi_fr = 1.0 / FZ[3];
  if (rtIsInf(phi_fr) || rtIsNaN(phi_fr)) {
    FZ[3] = 1.0;
  }
  omega_w[3] = fabs(vel[5]);
  omega_w[3] = (SL[3] * omega_w[3] + vel[5]) / RE;
  /*  Compute Tire Slip */
  /*  Velocity at CoG */
  Vg[0] = Vx;
  Vg[1] = Vy;
  /*  Velocity at each Tire */
  V_fl_idx_0 = Vx + *yaw * s[0];
  mu_x_idx_0 = Vy + *yaw * l[0];
  v_w_idx_1 = Vx + *yaw * -s[0];
  V_rl_idx_0 = Vx + *yaw * s[1];
  a_bar_idx_0 = Vy + *yaw * -l[1];
  V_rr_idx_0 = Vx + *yaw * -s[1];
  /*  Individual slip angles, it blows up at low velocity */
  if (Vx > 0.1) {
    phi_fl = atan(mu_x_idx_0 / V_fl_idx_0);
    phi_fr = atan(mu_x_idx_0 / v_w_idx_1);
    phi_rl = atan(a_bar_idx_0 / V_rl_idx_0);
    phi_rr = atan(a_bar_idx_0 / V_rr_idx_0);
  } else {
    phi_fl = 0.0;
    phi_fr = 0.0;
    phi_rl = 0.0;
    phi_rr = 0.0;
  }
  /*  Compute Slip Angle */
  SA_fl = -phi_fl + acker_steer_angles[0];
  SA_fr = -phi_fr + acker_steer_angles[1];
  /*  Compute Slip Ratio */
  /*  Vector of wheel linear velocity */
  /*  This line is commented out only for simulation. On this vehicle, this */
  /*  line is to be included. */
  scale = 3.3121686421112381E-170;
  b_scale = 3.3121686421112381E-170;
  c_scale = 3.3121686421112381E-170;
  d_scale = 3.3121686421112381E-170;
  phi_fl = fabs(V_fl_idx_0);
  if (phi_fl > 3.3121686421112381E-170) {
    y = 1.0;
    scale = phi_fl;
  } else {
    v_w_idx_0 = phi_fl / 3.3121686421112381E-170;
    y = v_w_idx_0 * v_w_idx_0;
  }
  phi_fl = fabs(v_w_idx_1);
  if (phi_fl > 3.3121686421112381E-170) {
    v_w_idx_1 = 1.0;
    b_scale = phi_fl;
  } else {
    v_w_idx_0 = phi_fl / 3.3121686421112381E-170;
    v_w_idx_1 = v_w_idx_0 * v_w_idx_0;
  }
  phi_fl = fabs(V_rl_idx_0);
  if (phi_fl > 3.3121686421112381E-170) {
    V_fl_idx_0 = 1.0;
    c_scale = phi_fl;
  } else {
    v_w_idx_0 = phi_fl / 3.3121686421112381E-170;
    V_fl_idx_0 = v_w_idx_0 * v_w_idx_0;
  }
  phi_fl = fabs(V_rr_idx_0);
  if (phi_fl > 3.3121686421112381E-170) {
    phi_fr = 1.0;
    d_scale = phi_fl;
  } else {
    v_w_idx_0 = phi_fl / 3.3121686421112381E-170;
    phi_fr = v_w_idx_0 * v_w_idx_0;
  }
  phi_fl = fabs(mu_x_idx_0);
  if (phi_fl > scale) {
    v_w_idx_0 = scale / phi_fl;
    y = y * v_w_idx_0 * v_w_idx_0 + 1.0;
    scale = phi_fl;
  } else {
    v_w_idx_0 = phi_fl / scale;
    y += v_w_idx_0 * v_w_idx_0;
  }
  if (phi_fl > b_scale) {
    v_w_idx_0 = b_scale / phi_fl;
    v_w_idx_1 = v_w_idx_1 * v_w_idx_0 * v_w_idx_0 + 1.0;
    b_scale = phi_fl;
  } else {
    v_w_idx_0 = phi_fl / b_scale;
    v_w_idx_1 += v_w_idx_0 * v_w_idx_0;
  }
  phi_fl = fabs(a_bar_idx_0);
  if (phi_fl > c_scale) {
    v_w_idx_0 = c_scale / phi_fl;
    V_fl_idx_0 = V_fl_idx_0 * v_w_idx_0 * v_w_idx_0 + 1.0;
    c_scale = phi_fl;
  } else {
    v_w_idx_0 = phi_fl / c_scale;
    V_fl_idx_0 += v_w_idx_0 * v_w_idx_0;
  }
  if (phi_fl > d_scale) {
    v_w_idx_0 = d_scale / phi_fl;
    phi_fr = phi_fr * v_w_idx_0 * v_w_idx_0 + 1.0;
    d_scale = phi_fl;
  } else {
    v_w_idx_0 = phi_fl / d_scale;
    phi_fr += v_w_idx_0 * v_w_idx_0;
  }
  y = scale * sqrt(y);
  v_w_idx_1 = b_scale * sqrt(v_w_idx_1);
  V_fl_idx_0 = c_scale * sqrt(V_fl_idx_0);
  phi_fr = d_scale * sqrt(phi_fr);
  SL[0] = RE * omega_w[0] / (y * cos(SA_fl)) - 1.0;
  SL[1] = RE * omega_w[1] / (v_w_idx_1 * cos(SA_fr)) - 1.0;
  SL[2] = RE * omega_w[2] / (V_fl_idx_0 * cos(-phi_rl)) - 1.0;
  SL[3] = RE * omega_w[3] / (phi_fr * cos(-phi_rr)) - 1.0;
  /*  Non Dimensional Tire Model */
  /*  kx model */
  /*  mu_x model */
  /*  mu_y model */
  /*  C model */
  k[0] = fabs(FZ[0]);
  k[1] = fabs(FZ[1]);
  k[2] = fabs(FZ[2]);
  k[3] = fabs(FZ[3]);
  pchip(k, C);
  /*  Non Dimensional Model */
  /*  Basic Coefficients */
  /*  Normalized Slip Ratio, Slip Angle, Camber Angle */
  /*  C is a function of FZ, but it is not an analytical expression */
  /*  Combined Slip Model Parameters */
  d = FZ[0] * FZ[0];
  d1 = (0.0 * d + 50.76 * FZ[0]) + 1526.0;
  d_scale = d1;
  d2 = mu_factor * ((-4.726E-7 * d + 0.001047 * FZ[0]) + 1.855);
  mu_x_idx_0 = d2;
  d3 = mu_factor * (((-2.582E-9 * rt_powd_snf(FZ[0], 3.0) + 5.639E-6 * d) +
                     -0.00423 * FZ[0]) +
                    3.658);
  v_w_idx_0 = d3;
  d4 = d2 * FZ[0];
  Fx_max[0] = d4;
  d = d1 * SL[0] / d4;
  if (rtIsInf(SA_fl) || rtIsNaN(SA_fl)) {
    phi_fr = rtNaN;
  } else {
    phi_fr = rt_remd_snf(SA_fl, 360.0);
    phi_fl = fabs(phi_fr);
    if (phi_fl > 180.0) {
      if (phi_fr > 0.0) {
        phi_fr -= 360.0;
      } else {
        phi_fr += 360.0;
      }
      phi_fl = fabs(phi_fr);
    }
    if (phi_fl <= 45.0) {
      phi_fr *= 0.017453292519943295;
      n = 0;
    } else if (phi_fl <= 135.0) {
      if (phi_fr > 0.0) {
        phi_fr = 0.017453292519943295 * (phi_fr - 90.0);
        n = 1;
      } else {
        phi_fr = 0.017453292519943295 * (phi_fr + 90.0);
        n = -1;
      }
    } else if (phi_fr > 0.0) {
      phi_fr = 0.017453292519943295 * (phi_fr - 180.0);
      n = 2;
    } else {
      phi_fr = 0.017453292519943295 * (phi_fr + 180.0);
      n = -2;
    }
    phi_fr = tan(phi_fr);
    if ((n == 1) || (n == -1)) {
      phi_fl = 1.0 / phi_fr;
      phi_fr = -(1.0 / phi_fr);
      if (rtIsInf(phi_fr) && (n == 1)) {
        phi_fr = phi_fl;
      }
    }
  }
  scale = C[0] * phi_fr / (d3 * FZ[0]);
  a_bar_idx_0 = scale;
  C[0] = C[0] * d2 / (d1 * d3);
  k[0] = sqrt(d * d + scale * scale);
  d = FZ[1] * FZ[1];
  d1 = (0.0 * d + 50.76 * FZ[1]) + 1526.0;
  kx_idx_1 = d1;
  d2 = mu_factor * ((-4.726E-7 * d + 0.001047 * FZ[1]) + 1.855);
  mu_x_idx_1 = d2;
  d3 = mu_factor * (((-2.582E-9 * rt_powd_snf(FZ[1], 3.0) + 5.639E-6 * d) +
                     -0.00423 * FZ[1]) +
                    3.658);
  V_rr_idx_0 = d3;
  d4 = d2 * FZ[1];
  Fx_max[1] = d4;
  d = d1 * SL[1] / d4;
  if (rtIsInf(SA_fr) || rtIsNaN(SA_fr)) {
    phi_fr = rtNaN;
  } else {
    phi_fr = rt_remd_snf(SA_fr, 360.0);
    phi_fl = fabs(phi_fr);
    if (phi_fl > 180.0) {
      if (phi_fr > 0.0) {
        phi_fr -= 360.0;
      } else {
        phi_fr += 360.0;
      }
      phi_fl = fabs(phi_fr);
    }
    if (phi_fl <= 45.0) {
      phi_fr *= 0.017453292519943295;
      n = 0;
    } else if (phi_fl <= 135.0) {
      if (phi_fr > 0.0) {
        phi_fr = 0.017453292519943295 * (phi_fr - 90.0);
        n = 1;
      } else {
        phi_fr = 0.017453292519943295 * (phi_fr + 90.0);
        n = -1;
      }
    } else if (phi_fr > 0.0) {
      phi_fr = 0.017453292519943295 * (phi_fr - 180.0);
      n = 2;
    } else {
      phi_fr = 0.017453292519943295 * (phi_fr + 180.0);
      n = -2;
    }
    phi_fr = tan(phi_fr);
    if ((n == 1) || (n == -1)) {
      phi_fl = 1.0 / phi_fr;
      phi_fr = -(1.0 / phi_fr);
      if (rtIsInf(phi_fr) && (n == 1)) {
        phi_fr = phi_fl;
      }
    }
  }
  scale = C[1] * phi_fr / (d3 * FZ[1]);
  a_bar_idx_1 = scale;
  C[1] = C[1] * d2 / (d1 * d3);
  k[1] = sqrt(d * d + scale * scale);
  d = FZ[2] * FZ[2];
  d1 = (0.0 * d + 50.76 * FZ[2]) + 1526.0;
  kx_idx_2 = d1;
  d2 = mu_factor * ((-4.726E-7 * d + 0.001047 * FZ[2]) + 1.855);
  mu_x_idx_2 = d2;
  d3 = mu_factor * (((-2.582E-9 * rt_powd_snf(FZ[2], 3.0) + 5.639E-6 * d) +
                     -0.00423 * FZ[2]) +
                    3.658);
  b_scale = d3;
  d4 = d2 * FZ[2];
  Fx_max[2] = d4;
  d = d1 * SL[2] / d4;
  if (rtIsInf(-phi_rl) || rtIsNaN(-phi_rl)) {
    phi_fr = rtNaN;
  } else {
    phi_fr = rt_remd_snf(-phi_rl, 360.0);
    phi_fl = fabs(phi_fr);
    if (phi_fl > 180.0) {
      if (phi_fr > 0.0) {
        phi_fr -= 360.0;
      } else {
        phi_fr += 360.0;
      }
      phi_fl = fabs(phi_fr);
    }
    if (phi_fl <= 45.0) {
      phi_fr *= 0.017453292519943295;
      n = 0;
    } else if (phi_fl <= 135.0) {
      if (phi_fr > 0.0) {
        phi_fr = 0.017453292519943295 * (phi_fr - 90.0);
        n = 1;
      } else {
        phi_fr = 0.017453292519943295 * (phi_fr + 90.0);
        n = -1;
      }
    } else if (phi_fr > 0.0) {
      phi_fr = 0.017453292519943295 * (phi_fr - 180.0);
      n = 2;
    } else {
      phi_fr = 0.017453292519943295 * (phi_fr + 180.0);
      n = -2;
    }
    phi_fr = tan(phi_fr);
    if ((n == 1) || (n == -1)) {
      phi_fl = 1.0 / phi_fr;
      phi_fr = -(1.0 / phi_fr);
      if (rtIsInf(phi_fr) && (n == 1)) {
        phi_fr = phi_fl;
      }
    }
  }
  scale = C[2] * phi_fr / (d3 * FZ[2]);
  a_bar_idx_2 = scale;
  C[2] = C[2] * d2 / (d1 * d3);
  k[2] = sqrt(d * d + scale * scale);
  d = FZ[3] * FZ[3];
  d1 = (0.0 * d + 50.76 * FZ[3]) + 1526.0;
  d2 = mu_factor * ((-4.726E-7 * d + 0.001047 * FZ[3]) + 1.855);
  d3 = mu_factor * (((-2.582E-9 * rt_powd_snf(FZ[3], 3.0) + 5.639E-6 * d) +
                     -0.00423 * FZ[3]) +
                    3.658);
  d4 = d2 * FZ[3];
  d = d1 * SL[3] / d4;
  if (rtIsInf(-phi_rr) || rtIsNaN(-phi_rr)) {
    phi_fr = rtNaN;
  } else {
    phi_fr = rt_remd_snf(-phi_rr, 360.0);
    phi_fl = fabs(phi_fr);
    if (phi_fl > 180.0) {
      if (phi_fr > 0.0) {
        phi_fr -= 360.0;
      } else {
        phi_fr += 360.0;
      }
      phi_fl = fabs(phi_fr);
    }
    if (phi_fl <= 45.0) {
      phi_fr *= 0.017453292519943295;
      n = 0;
    } else if (phi_fl <= 135.0) {
      if (phi_fr > 0.0) {
        phi_fr = 0.017453292519943295 * (phi_fr - 90.0);
        n = 1;
      } else {
        phi_fr = 0.017453292519943295 * (phi_fr + 90.0);
        n = -1;
      }
    } else if (phi_fr > 0.0) {
      phi_fr = 0.017453292519943295 * (phi_fr - 180.0);
      n = 2;
    } else {
      phi_fr = 0.017453292519943295 * (phi_fr + 180.0);
      n = -2;
    }
    phi_fr = tan(phi_fr);
    if ((n == 1) || (n == -1)) {
      phi_fl = 1.0 / phi_fr;
      phi_fr = -(1.0 / phi_fr);
      if (rtIsInf(phi_fr) && (n == 1)) {
        phi_fr = phi_fl;
      }
    }
  }
  scale = C[3] * phi_fr / (d3 * FZ[3]);
  C[3] = C[3] * d2 / (d1 * d3);
  c_scale = scale * scale;
  k[3] = sqrt(d * d + c_scale);
  phi_fl = 1.0;
  phi_fr = 1.0;
  V_fl_idx_0 = 1.0;
  V_rl_idx_0 = 1.0;
  /*  Compute R, the bridge between tire slip and tire forces */
  /*  Find FX & FY, denominator blows up at no slip */
  for (i = 0; i < 4; i++) {
    if (fabs(k[i]) <= 6.2831853071795862) {
      phi_fl = 0.5 * (C[0] + 1.0) - 0.5 * cos(k[0] / 2.0) * (1.0 - C[0]);
      phi_fr = 0.5 * (C[1] + 1.0) - 0.5 * cos(k[1] / 2.0) * (1.0 - C[1]);
      V_fl_idx_0 = 0.5 * (C[2] + 1.0) - 0.5 * cos(k[2] / 2.0) * (1.0 - C[2]);
      V_rl_idx_0 = 0.5 * (C[3] + 1.0) - 0.5 * cos(k[3] / 2.0) * (1.0 - C[3]);
    }
    d = k[i];
    R[i] = 1.136 * exp(-0.03333 * d) + -1.136 * exp(-1.032 * d);
  }
  /*  Max Longitudinal Force */
  /*  Constant during runtime */
  if ((fabs(SL[0]) == 0.001) &&
      ((fabs(SA_fl) == 0.1) || (fabs(phi_fl) < 0.001))) {
    FY[0] = 0.0;
  } else {
    y = tan(SA_fl);
    d = sqrt(SL[0] * SL[0] + phi_fl * phi_fl * (y * y));
    if (d < 0.001) {
      FY[0] = 0.0;
    } else {
      /*         FX(i) = (mu_x(i) .* FZ(i) .* R(i) .* SL(i)) ./ (sqrt(SL(i).^2 +
       * ((n(i).^2) .* (tan(SA(i))).^2))); */
      FY[0] = phi_fl * R[0] * y * v_w_idx_0 * FZ[0] / d;
    }
  }
  v_w_idx_0 = 1.0;
  if ((fabs(SL[1]) == 0.001) &&
      ((fabs(SA_fr) == 0.1) || (fabs(phi_fr) < 0.001))) {
    FY[1] = 0.0;
  } else {
    y = tan(SA_fr);
    d = sqrt(SL[1] * SL[1] + phi_fr * phi_fr * (y * y));
    if (d < 0.001) {
      FY[1] = 0.0;
    } else {
      /*         FX(i) = (mu_x(i) .* FZ(i) .* R(i) .* SL(i)) ./ (sqrt(SL(i).^2 +
       * ((n(i).^2) .* (tan(SA(i))).^2))); */
      FY[1] = phi_fr * R[1] * y * V_rr_idx_0 * FZ[1] / d;
    }
  }
  v_w_idx_1 = 1.0;
  if ((fabs(SL[2]) == 0.001) &&
      ((fabs(-phi_rl) == 0.1) || (fabs(V_fl_idx_0) < 0.001))) {
    FY[2] = 0.0;
  } else {
    y = tan(-phi_rl);
    d = sqrt(SL[2] * SL[2] + V_fl_idx_0 * V_fl_idx_0 * (y * y));
    if (d < 0.001) {
      FY[2] = 0.0;
    } else {
      /*         FX(i) = (mu_x(i) .* FZ(i) .* R(i) .* SL(i)) ./ (sqrt(SL(i).^2 +
       * ((n(i).^2) .* (tan(SA(i))).^2))); */
      FY[2] = V_fl_idx_0 * R[2] * y * b_scale * FZ[2] / d;
    }
  }
  V_fl_idx_0 = 1.0;
  if ((fabs(SL[3]) == 0.001) &&
      ((fabs(-phi_rr) == 0.1) || (fabs(V_rl_idx_0) < 0.001))) {
    FY[3] = 0.0;
  } else {
    y = tan(-phi_rr);
    d = sqrt(SL[3] * SL[3] + V_rl_idx_0 * V_rl_idx_0 * (y * y));
    if (d < 0.001) {
      FY[3] = 0.0;
    } else {
      /*         FX(i) = (mu_x(i) .* FZ(i) .* R(i) .* SL(i)) ./ (sqrt(SL(i).^2 +
       * ((n(i).^2) .* (tan(SA(i))).^2))); */
      FY[3] = V_rl_idx_0 * R[3] * y * d3 * FZ[3] / d;
    }
  }
  phi_fr = 1.0;
  if (fabs(k[3]) <= 6.2831853071795862) {
    d = 0.5 * cos(k_limit / 2.0);
    v_w_idx_0 = 0.5 * (C[0] + 1.0) - d * (1.0 - C[0]);
    v_w_idx_1 = 0.5 * (C[1] + 1.0) - d * (1.0 - C[1]);
    V_fl_idx_0 = 0.5 * (C[2] + 1.0) - d * (1.0 - C[2]);
    phi_fr = 0.5 * (C[3] + 1.0) - d * (1.0 - C[3]);
  }
  /*  Compare FY Calculations */
  /*  Incomplete, or will not be implemented */
  /*  This is for simulation */
  d = 0.0;
  if (k_limit > fabs(a_bar_idx_0)) {
    d = sqrt(k_limit * k_limit - a_bar_idx_0 * a_bar_idx_0);
  }
  d = d * mu_x_idx_0 * FZ[0] / d_scale;
  d3 = tan(SA_fl);
  phi_fl = 1.136 * exp(-0.03333 * k_limit) + -1.136 * exp(-1.032 * k_limit);
  Fx_max[0] =
      Fx_max[0] * phi_fl * d / sqrt(d * d + v_w_idx_0 * v_w_idx_0 * (d3 * d3));
  FY[0] = rt_roundd_snf(FY[0] * 100.0) / 100.0;
  d = 0.0;
  if (k_limit > fabs(a_bar_idx_1)) {
    d = sqrt(k_limit * k_limit - a_bar_idx_1 * a_bar_idx_1);
  }
  d = d * mu_x_idx_1 * FZ[1] / kx_idx_1;
  d3 = tan(SA_fr);
  Fx_max[1] =
      Fx_max[1] * phi_fl * d / sqrt(d * d + v_w_idx_1 * v_w_idx_1 * (d3 * d3));
  FY[1] = rt_roundd_snf(FY[1] * 100.0) / 100.0;
  d = 0.0;
  if (k_limit > fabs(a_bar_idx_2)) {
    d = sqrt(k_limit * k_limit - a_bar_idx_2 * a_bar_idx_2);
  }
  d = d * mu_x_idx_2 * FZ[2] / kx_idx_2;
  d3 = tan(-phi_rl);
  Fx_max[2] = Fx_max[2] * phi_fl * d /
              sqrt(d * d + V_fl_idx_0 * V_fl_idx_0 * (d3 * d3));
  FY[2] = rt_roundd_snf(FY[2] * 100.0) / 100.0;
  d = 0.0;
  if (k_limit > fabs(scale)) {
    d = sqrt(k_limit * k_limit - c_scale);
  }
  d = d * d2 * FZ[3] / d1;
  d3 = tan(-phi_rr);
  Fx_max[3] = d4 * phi_fl * d / sqrt(d * d + phi_fr * phi_fr * (d3 * d3));
  FY[3] = rt_roundd_snf(FY[3] * 100.0) / 100.0;
}

/*
 * File trailer for Layer_0.c
 *
 * [EOF]
 */
