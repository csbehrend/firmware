
#include "Layer_0.h"
#include "Layer_1.h"
#include "Layer_2.h"
#include "LP_calc.h"
#include "TorqueVector.h"

static void argInit_1x2_real_T(double result[2]);
static void argInit_1x4_real32_T(float result[4]);
static void argInit_1x4_real_T(double result[4]);
static void argInit_1x6_real_T(double result[6]);
static float argInit_real32_T(void);
static double argInit_real_T(void);
static void main_LP_calc(void);
static void main_Layer_0(void);
static void main_Layer_1(void);
static void main_Layer_2(void);

void TorqueVector_main()
{
    static VehicleConfiguration_t per_2022 = {
        .half_track_width = {0.87, 0.8}
    };

    static int state = 0;
    switch (state)
    {
    case 0:
        main_Layer_0();
        state++;
        break;
    
    case 1:
        main_Layer_1();
        state++;
        break;

    case 2:
        main_Layer_2();
        state++;
        break;

    case 3:
        main_LP_calc();
        state++;
        break;
    default:
        state++;
        if (state == 5)
            state = 0;
        break;
    }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_LP_calc(void)
{
  double Tx2[4];
  double Tx_actual[4];
  double lb_adjust[4];
  double prev_torque_tmp[4];
  double ub_adjust[4];
  double M_max[2];
  double dv[2];
  double driver_input_tmp;
  double power;
  double typed;
  double yaw_accel;
  double zero_crossing;
  float T2F_tmp[4];
  float b_T2F_tmp[4];
  float lb[4];
  float lb_plus[4];
  float ub[4];
  float ub_neg[4];
  float bigM_flag;
  bool gh[4];
  /* Initialize function 'LP_calc' input arguments. */
  /* Initialize function input argument 'prev_torque'. */
  argInit_1x4_real_T(prev_torque_tmp);
  driver_input_tmp = argInit_real_T();
  /* Initialize function input argument 'FY'. */
  /* Initialize function input argument 'omega_m'. */
  /* Initialize function input argument 'x'. */
  /* Initialize function input argument 'y'. */
  /* Initialize function input argument 'T2F'. */
  argInit_1x4_real32_T(T2F_tmp);
  /* Initialize function input argument 'slip_limit'. */
  /* Initialize function input argument 'power_limit'. */
  /* Initialize function input argument 'tau_limit_upper'. */
  /* Initialize function input argument 'tau_limit_lower'. */
  /* Initialize function input argument 'rpm_limit'. */
  /* Initialize function input argument 'motor_efficiency'. */
  /* Initialize function input argument 'motor_limit_torque'. */
  /* Call the entry-point 'LP_calc'. */
  argInit_1x2_real_T(dv);
  b_T2F_tmp[0] = T2F_tmp[0];
  b_T2F_tmp[1] = T2F_tmp[1];
  b_T2F_tmp[2] = T2F_tmp[2];
  b_T2F_tmp[3] = T2F_tmp[3];
  LP_calc(prev_torque_tmp, driver_input_tmp, driver_input_tmp, prev_torque_tmp,
          prev_torque_tmp, prev_torque_tmp, prev_torque_tmp, b_T2F_tmp, T2F_tmp,
          prev_torque_tmp, prev_torque_tmp, prev_torque_tmp, prev_torque_tmp,
          driver_input_tmp, driver_input_tmp, prev_torque_tmp, dv,
          driver_input_tmp, Tx_actual, lb, ub, M_max, &power, &bigM_flag,
          &typed, &zero_crossing, &yaw_accel, ub_adjust, lb_adjust, lb_plus,
          ub_neg, Tx2, gh);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_Layer_0(void)
{
  double dv[6];
  double C[4];
  double FX[4];
  double FY[4];
  double Fx_max[4];
  double R[4];
  double SA[4];
  double b_omega_w_tmp[4];
  double n[4];
  double omega_w_tmp[4];
  double Vg[2];
  double acker_steer_angles[2];
  double s_tmp[2];
  double center_steer_angle_tmp;
  /* Initialize function 'Layer_0' input arguments. */
  /* Initialize function input argument 'omega_w'. */
  argInit_1x4_real_T(omega_w_tmp);
  center_steer_angle_tmp = argInit_real_T();
  /* Initialize function input argument 'FZ'. */
  /* Initialize function input argument 'SL'. */
  /* Initialize function input argument 's'. */
  argInit_1x2_real_T(s_tmp);
  /* Initialize function input argument 'l'. */
  /* Initialize function input argument 'vel'. */
  /* Call the entry-point 'Layer_0'. */
  argInit_1x6_real_T(dv);
  b_omega_w_tmp[0] = omega_w_tmp[0];
  b_omega_w_tmp[1] = omega_w_tmp[1];
  b_omega_w_tmp[2] = omega_w_tmp[2];
  b_omega_w_tmp[3] = omega_w_tmp[3];
  Layer_0(
      omega_w_tmp, &center_steer_angle_tmp, center_steer_angle_tmp,
      center_steer_angle_tmp, &center_steer_angle_tmp, b_omega_w_tmp,
      b_omega_w_tmp, center_steer_angle_tmp, s_tmp, s_tmp, dv,
      center_steer_angle_tmp, center_steer_angle_tmp, center_steer_angle_tmp,
      center_steer_angle_tmp, center_steer_angle_tmp, center_steer_angle_tmp,
      center_steer_angle_tmp, center_steer_angle_tmp, center_steer_angle_tmp,
      center_steer_angle_tmp, center_steer_angle_tmp, center_steer_angle_tmp,
      center_steer_angle_tmp, center_steer_angle_tmp, center_steer_angle_tmp, C,
      acker_steer_angles, FY, Vg, Fx_max, n, R, SA, FX);
    asm("bkpt");
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_Layer_1(void)
{
  double C_tmp[4];
  double T_brake[4];
  double acker_steer_angles_tmp[2];
  double power_limits_battery[2];
  double yaw_ref[2];
  double driver_input;
  double driver_input_tmp;
  float T2F[4];
  /* Initialize function 'Layer_1' input arguments. */
  driver_input_tmp = argInit_real_T();
  /* Initialize function input argument 'power_limits_battery'. */
  /* Initialize function input argument 'C'. */
  argInit_1x4_real_T(C_tmp);
  /* Initialize function input argument 'omega_w'. */
  /* Initialize function input argument 'acker_steer_angles'. */
  argInit_1x2_real_T(acker_steer_angles_tmp);
  /* Initialize function input argument 'FY'. */
  /* Initialize function input argument 'Vg'. */
  /* Initialize function input argument 'gr'. */
  /* Initialize function input argument 'gearbox_efficiency'. */
  /* Initialize function input argument 'l'. */
  /* Initialize function input argument 's'. */
  /* Initialize function input argument 'tire_mu'. */
  /* Initialize function input argument 'brakeforce_max'. */
  /* Call the entry-point 'Layer_1'. */
  argInit_1x2_real_T(power_limits_battery);
  driver_input = driver_input_tmp;
  Layer_1(&driver_input, power_limits_battery, C_tmp, C_tmp, driver_input_tmp,
          acker_steer_angles_tmp, C_tmp, acker_steer_angles_tmp,
          &driver_input_tmp, driver_input_tmp, C_tmp, C_tmp, driver_input_tmp,
          driver_input_tmp, driver_input_tmp, acker_steer_angles_tmp,
          acker_steer_angles_tmp, driver_input_tmp, driver_input_tmp, C_tmp,
          C_tmp, driver_input_tmp, driver_input_tmp, driver_input_tmp,
          driver_input_tmp, driver_input_tmp, T2F, T_brake, yaw_ref);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_Layer_2(void)
{
  double omega_m[4];
  double prev_torque_tmp[4];
  double rpm_limit[4];
  double torque_limit_motor[4];
  double x[4];
  double y[4];
  double acker_steer_angles_tmp[2];
  double dv[2];
  double driver_input_tmp;
  double power_limit_battery;
  double tau_limit_lower;
  double tau_limit_upper;
  double yaw_err;
  double yaw_err_percent;
  float T2F[4];
  float slip_limit[4];
  /* Initialize function 'Layer_2' input arguments. */
  /* Initialize function input argument 'prev_torque'. */
  argInit_1x4_real_T(prev_torque_tmp);
  driver_input_tmp = argInit_real_T();
  /* Initialize function input argument 'power_limits_battery'. */
  /* Initialize function input argument 'acker_steer_angles'. */
  argInit_1x2_real_T(acker_steer_angles_tmp);
  /* Initialize function input argument 'FY'. */
  /* Initialize function input argument 'omega_w'. */
  /* Initialize function input argument 'T2F'. */
  /* Initialize function input argument 'T_brake'. */
  /* Initialize function input argument 'yaw_ref'. */
  /* Initialize function input argument 'Fx_max'. */
  /* Initialize function input argument 'motor_limit_power'. */
  /* Initialize function input argument 'motor_efficiency'. */
  /* Initialize function input argument 'gr'. */
  /* Initialize function input argument 'motor_limit_torque'. */
  /* Initialize function input argument 'l'. */
  /* Initialize function input argument 's'. */
  /* Call the entry-point 'Layer_2'. */
  argInit_1x4_real32_T(T2F);
  argInit_1x2_real_T(dv);
  Layer_2(prev_torque_tmp, &driver_input_tmp, dv, acker_steer_angles_tmp,
          prev_torque_tmp, prev_torque_tmp, T2F, prev_torque_tmp,
          driver_input_tmp, acker_steer_angles_tmp, prev_torque_tmp,
          prev_torque_tmp, prev_torque_tmp, driver_input_tmp, prev_torque_tmp,
          driver_input_tmp, driver_input_tmp, acker_steer_angles_tmp,
          driver_input_tmp, acker_steer_angles_tmp, driver_input_tmp,
          acker_steer_angles_tmp, &power_limit_battery, omega_m, x, y,
          slip_limit, torque_limit_motor, &tau_limit_upper, &tau_limit_lower,
          rpm_limit, &yaw_err, &yaw_err_percent);
}


static void argInit_1x2_real_T(double result[2])
{
  int idx1;
  for (idx1 = 0; idx1 < 2; idx1++) {
    result[idx1] = 0.0;
  }
}

static void argInit_1x4_real32_T(float result[4])
{
  int idx1;
  for (idx1 = 0; idx1 < 4; idx1++) {
    result[idx1] = 0.0F;
  }
}

static void argInit_1x4_real_T(double result[4])
{
  int idx1;
  for (idx1 = 0; idx1 < 4; idx1++) {
    result[idx1] = 0.0;
  }
}

static void argInit_1x6_real_T(double result[6])
{
  int idx1;
  for (idx1 = 0; idx1 < 6; idx1++) {
    result[idx1] = 0.0;
  }
}

/*
 * Arguments    : void
 * Return Type  : float
 */
static float argInit_real32_T(void)
{
  return 0.0F;
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}